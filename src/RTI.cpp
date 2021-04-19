
#include <string>

// MOAB
#include "MBTagConventions.hpp"
#include "moab/GeomTopoTool.hpp"

// Double-down
#include "double-down/RTI.hpp"

void error(void* dum, RTCError code, const char* str) {
  if (code != RTC_ERROR_NONE) {
    std::cout << "Error occured" << std::endl;
    std::string msg(str);
    std::cout << msg << std::endl;
  }
}

RayTracingInterface::RayTracingInterface(moab::Interface *mbi) : GTT(std::make_shared<moab::GeomTopoTool>(mbi)), MBI(mbi) {}

RayTracingInterface::RayTracingInterface(std::shared_ptr<moab::GeomTopoTool> gtt) : MBI(gtt->get_moab_instance()), GTT(gtt) {}

moab::ErrorCode RayTracingInterface::init()
{
  moab::ErrorCode rval;

  // here we assume that the MOAB file is alredy loaded
  g_device = rtcNewDevice(NULL);

  rtcSetDeviceErrorFunction(g_device, (RTCErrorFunction)error, NULL);

  mdam = std::shared_ptr<MBDirectAccess>(new MBDirectAccess(MBI));

  // detemine how many volumes are in the MOAB file
  moab::Range vols;
  rval = get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get MOAB volumes");

  if (vols.size() == 0) {
    std::cerr << "No volumes found in the MOAB instance.\n";
    return moab::MB_SUCCESS;
  }

  // create an Embree geometry instance for each surface
  for (auto vol : vols) {
    allocateTriangleBuffer(vol);
    createBVH(vol);
  } // end volume loop

  return moab::MB_SUCCESS;
}

moab::ErrorCode RayTracingInterface::load_file(std::string filename) {
  moab::ErrorCode rval = MBI->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load the specified MOAB file: " << filename);
  return rval;
}

moab::ErrorCode
RayTracingInterface::get_obb(moab::EntityHandle volume,
                             std::array<double, 3>& center,
                             std::array<double, 3>& axis0,
                             std::array<double, 3>& axis1,
                             std::array<double, 3>& axis2)
{
  std::array<double, 3> llc;
  std::array<double, 3> urc;
  moab::ErrorCode rval = get_bbox(volume, llc, urc);
  MB_CHK_SET_ERR(rval, "Failed to get bounding box");

  center[0] = 0.5 * (llc[0] + urc[0]);
  center[1] = 0.5 * (llc[1] + urc[1]);
  center[2] = 0.5 * (llc[2] + urc[2]);

  std::array<double, 3> width({0.5 * (urc[0] - llc[0]),
                               0.5 * (urc[1] - llc[1]),
                               0.5 * (urc[2] - llc[2])});

  // bounding boxes from Embree are axis-aligned
  axis0 = {width[0], 0, 0};
  axis1 = {0, width[1], 0};
  axis2 = {0, 0, width[2]};

  return moab::MB_SUCCESS;
}

moab::ErrorCode
RayTracingInterface::get_bbox(moab::EntityHandle volume,
                              std::array<double, 3>& llc,
                              std::array<double, 3>& urc)
{
  RTCBounds bounds;

  rtcGetSceneBounds(scene_map.at(volume), &bounds);

  llc[0] = bounds.lower_x;
  llc[1] = bounds.lower_y;
  llc[2] = bounds.lower_z;
  urc[0] = bounds.upper_x;
  urc[1] = bounds.upper_y;
  urc[2] = bounds.upper_z;

  return moab::MB_SUCCESS;
}


moab::ErrorCode
RayTracingInterface::allocateTriangleBuffer(moab::EntityHandle volume)
{
  moab::ErrorCode rval;

  // get volume surfaces
  moab::Range surfs;
  rval = MBI->get_child_meshsets(volume, surfs);
  MB_CHK_SET_ERR(rval, "Failed to get the volume sufaces");

  // allocate triangle storage for this volume
  int num_vol_tris = 0;
  for (moab::Range::iterator j = surfs.begin(); j != surfs.end(); j++) {
    int num_surf_tris;
    rval = MBI->get_number_entities_by_type(*j, moab::MBTRI, num_surf_tris);
    MB_CHK_SET_ERR(rval, "Failed to get triangle count");
    num_vol_tris += num_surf_tris;
  }

  // TODO: investigate memory usage here
  std::vector<DblTri> emtris(num_vol_tris);
  buffer_storage.store(volume, std::move(emtris));

  return MB_SUCCESS;
}

moab::ErrorCode RayTracingInterface::createBVH(moab::EntityHandle volume)
{
  moab::ErrorCode rval;

  if (GTT->dimension(volume) != 3) {
    MB_CHK_SET_ERR(MB_FAILURE, "This entity is not a volume. "
    "BVHs can only be created and deleted by volume in double-down.");
  }

  // add buffer storage if it isn't already there
  if (!buffer_storage.is_storing(volume)) { allocateTriangleBuffer(volume); }

  // add new scene to our vector
  RTCScene scene = rtcNewScene(g_device);
  rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(scene,RTC_BUILD_QUALITY_HIGH);

  // add this scene to the map from MOAB volumes to Embree scenes
  scene_map[volume] = scene;

  auto& tri_buffer = buffer_storage.retrieve_buffer(volume);
  auto emtris = tri_buffer.data();

  // get volume surfaces
  moab::Range surfs;
  rval = MBI->get_child_meshsets(volume, surfs);
  MB_CHK_SET_ERR(rval, "Failed to get the volume sufaces");

  // make sure that all triangles are available
  // before constructing the BVH
  mdam->update();

  // index to keep track of where we're adding triangles in the storage buffer
  int buffer_start = 0;
  for (moab::Range::iterator j = surfs.begin(); j != surfs.end(); j++) {
    moab::EntityHandle this_surf = *j;

    // find the surface sense
    int sense;
    rval = GTT->get_sense(this_surf, volume, sense);
    MB_CHK_SET_ERR(rval, "Failed to get sense for surface");

    // get all triangles on this surface
    moab::Range tris;
    rval = MBI->get_entities_by_type(this_surf, moab::MBTRI, tris);
    MB_CHK_SET_ERR(rval, "Failed to get surface triangles");

    int num_tris = tris.size();

    // create a new geometry for the volumeume's scene
    // EMBREE_FIXME: check if geometry gets properly committed
    RTCGeometry geom_0 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_USER);
    unsigned int emsurf = rtcAttachGeometry(scene, geom_0);

    // build settings for the Embree scene
    rtcSetGeometryBuildQuality(geom_0,RTC_BUILD_QUALITY_HIGH);
    rtcSetGeometryUserPrimitiveCount(geom_0, num_tris);
    rtcSetGeometryTimeStepCount(geom_0,1);

    // get the pointer into the buffer at the right location
    DblTri* buff_ptr = emtris + buffer_start;

    // set the data for this Embree geometry using that pointer
    rtcSetGeometryUserData(geom_0, buff_ptr);

    // set data for all triangles of this surfaces
    for (int k = 0; k < num_tris; k++) {
      buff_ptr[k].mdam = mdam.get();
      buff_ptr[k].handle = tris[k];
      buff_ptr[k].surf = this_surf;
      buff_ptr[k].geomID = emsurf;
      buff_ptr[k].sense = sense;
    }

    // advance the buffer offset by the number of triangles in this surface
    buffer_start += num_tris;

    // point the geometry BVH builder to the custom triangle bounding, intersection, and occlusion functions
    rtcSetGeometryBoundsFunction(geom_0,(RTCBoundsFunction)&DblTriBounds, NULL);
    rtcSetGeometryIntersectFunction (geom_0, (RTCIntersectFunctionN)&MBDblTriIntersectFunc);
    rtcSetGeometryOccludedFunction (geom_0, (RTCOccludedFunctionN)&DblTriOccludedFunc);

    // add the geometry to the Embree device
    rtcCommitGeometry(geom_0);
    } // end surface loop

    // commit the scene to the device (ready for use)
    rtcCommitScene(scene);

    return moab::MB_SUCCESS;
}

void RayTracingInterface::deleteBVH(moab::EntityHandle volume)
{

  if (GTT->dimension(volume) != 3) {
    MB_CHK_SET_ERR_CONT(MB_FAILURE, "This entity is not a volume. "
    "BVHs can only be created and deleted by volume in double-down.");
    return;
  }

  int ent_dim = GTT->dimension(volume);
  moab::Range surfs;

  // remove the scene from Embree
  rtcReleaseScene(scene_map[volume]);
  // remove the volume entry from the internal mapping structures
  buffer_storage.free_storage(volume);
  scene_map.erase(volume);
}

moab::ErrorCode
RayTracingInterface::get_normal(moab::EntityHandle surface,
                                const double loc[3],
                                double angle[3],
                                const moab::GeomQueryTool::RayHistory* history)
{
  moab::ErrorCode rval;
  moab::EntityHandle facet;

  // use the last triangle hit if the history is provided
  if (history) {
    rval = history->get_last_intersection(facet);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the last intersection from the ray history");
  // if no history is present, find the closest triangle to this location
  } else {
    // perform a closest location search

    // get the one of the volumes for this surface
    moab::Range parent_vols;
    rval = MBI->get_parent_meshsets(surface, parent_vols);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get parent volumes of the surface");
    assert(parent_vols.size() != 0);

    moab::EntityHandle vol = parent_vols[0];

    // arbitrarily use one of the parent volumes for the lookup
    const std::vector<DblTri>& buffer = buffer_storage.retrieve_buffer(vol);

    moab::EntityHandle surf, closest_facet;
    double dist;
    closest(vol, loc, dist, &surf, &closest_facet);

    RTCDPointQuery point_query;
    point_query.set_radius(inf);
    point_query.time = 0.f;
    point_query.set_point(loc);

    RTCPointQueryContext pq_context;
    rtcInitPointQueryContext(&pq_context);

    if (surf == 0) {
      MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to locate a nearest point.");
    }

    // TODO: restrict the search for the closest triangle to the surface provided
    if (surf != surface) {
      MB_CHK_SET_ERR(moab::MB_FAILURE, "Nearest point was not on the correct surface.");
    }

    facet = closest_facet;
  }

  moab::CartVect coords[3];
  const moab::EntityHandle *conn;
  int len;
  rval = MBI->get_connectivity(facet, conn, len);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get connectivity of triangle: " << facet);

  if (3 != len) {
    MB_CHK_SET_ERR_CONT(rval, "Incorrect connectivity length for a triangle: " << len);
  }

  rval = MBI->get_coords(conn, 3, &(coords[0][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get coords for triangle: " << facet);

  moab::CartVect normal(0.0);

  coords[1] -= coords[0];
  coords[2] -= coords[0];
  normal = coords[1] * coords[2];
  normal.normalize();

  angle[0] = normal[0];
  angle[1] = normal[1];
  angle[2] = normal[2];

  return rval;
}

// sum area of elements in surface
moab::ErrorCode
RayTracingInterface::measure_area(moab::EntityHandle surface,
                                  double& result)
{
  // get triangles in surface
  moab::Range triangles;
  moab::ErrorCode rval = MBI->get_entities_by_dimension( surface, 2, triangles );
  MB_CHK_SET_ERR(rval, "Failed to get the surface entities");
  if (!triangles.all_of_type(MBTRI)) {
    std::cout << "WARNING: Surface " << surface  // todo: use geomtopotool to get id by entity handle
              << " contains non-triangle elements. Area calculation may be incorrect."
              << std::endl;
    triangles.clear();
    rval = MBI->get_entities_by_type( surface, MBTRI, triangles );
    MB_CHK_SET_ERR(rval, "Failed to the surface's triangle entities");
  }

  // calculate sum of area of triangles
  result = 0.0;
  const moab::EntityHandle *conn;
  int len;
  moab::CartVect coords[3];
  for (moab::Range::iterator j = triangles.begin(); j != triangles.end(); ++j) {
    rval = MBI->get_connectivity( *j, conn, len, true );
    MB_CHK_SET_ERR(rval, "Failed to get the current triangle's connectivity");
    if(3 != len) {
      MB_SET_ERR(MB_FAILURE, "Incorrect connectivity length for triangle");
    }
    rval = MBI->get_coords( conn, 3, coords[0].array() );
    MB_CHK_SET_ERR(rval, "Failed to get the current triangle's vertex coordinates");

    // calculated area using cross product of triangle edges
    moab::CartVect v1 = coords[1] - coords[0];
    moab::CartVect v2 = coords[2] - coords[0];
    moab::CartVect xp = v1 * v2;
    result += xp.length();
  }

  result *= 0.5;
  return MB_SUCCESS;
}

moab::ErrorCode
RayTracingInterface::measure_volume(moab::EntityHandle volume,
                                    double& result)
{
  moab::ErrorCode rval;
  std::vector<moab::EntityHandle> surfaces;
  result = 0.0;

   // don't try to calculate volume of implicit complement
  if (GTT->is_implicit_complement(volume)) {
    result = 1.0;
    return MB_SUCCESS;
  }

  // get surfaces from volume
  rval = MBI->get_child_meshsets( volume, surfaces );
  MB_CHK_SET_ERR(rval, "Failed to get the volume's child surfaces");

  // get surface senses
  std::vector<int> senses( surfaces.size() );
  rval = GTT->get_surface_senses( volume, surfaces.size(), &surfaces[0], &senses[0] );
  MB_CHK_SET_ERR(rval, "Failed to retrieve surface-volume sense data. Cannot calculate volume");

  for (unsigned i = 0; i < surfaces.size(); ++i) {
    // skip non-manifold surfaces
    if (!senses[i]) { continue; }

    // get triangles in surface
    moab::Range triangles;
    rval = MBI->get_entities_by_dimension(surfaces[i], 2, triangles);
    MB_CHK_SET_ERR(rval, "Failed to get the surface triangles");

    if (!triangles.all_of_type(MBTRI)) {
      std::cout << "WARNING: Surface " << surfaces[i]  // todo: use geomtopotool to get id by entity handle
                << " contains non-triangle elements. Volume calculation may be incorrect."
                << std::endl;
      triangles.clear();
      rval = MBI->get_entities_by_type(surfaces[i], MBTRI, triangles);
      MB_CHK_SET_ERR(rval, "Failed to get the surface triangles");
    }

    // calculate signed volume beneath surface (x 6.0)
    double surf_sum = 0.0;
    const moab::EntityHandle *conn;
    int len;

    moab::CartVect coords[3];
    for (moab::Range::iterator j = triangles.begin(); j != triangles.end(); ++j) {
      rval = MBI->get_connectivity( *j, conn, len, true );
      MB_CHK_SET_ERR(rval, "Failed to get the connectivity of the current triangle");
      if(3 != len) { MB_SET_ERR(MB_FAILURE, "Incorrect connectivity length for triangle"); }
      rval = MBI->get_coords( conn, 3, coords[0].array() );
      MB_CHK_SET_ERR(rval, "Failed to get the coordinates of the current triangle's vertices");

      coords[1] -= coords[0];
      coords[2] -= coords[0];
      surf_sum += (coords[0] % (coords[1] * coords[2]));
    }

    result += senses[i] * surf_sum;
  }
  result /= 6.0;

  return MB_SUCCESS;
}

moab::ErrorCode
RayTracingInterface::closest_to_location(moab::EntityHandle volume,
                                         const double point[3],
                                         double& result,
                                         moab::EntityHandle* closest_surf)
{
  closest(volume, point, result, closest_surf, nullptr);
  return moab::MB_SUCCESS;
}

void RayTracingInterface::closest(moab::EntityHandle volume,
                                  const double loc[3],
                                  double& result,
                                  moab::EntityHandle* surface,
                                  moab::EntityHandle* facet)
{

  const std::vector<DblTri>& buffer = buffer_storage.retrieve_buffer(volume);

  // create point query object, a dual representation of the single and double precision query,
  // sets parameters of the closest to location query
  RTCDPointQuery point_query;
  point_query.set_radius(inf);
  point_query.time = 0.f;
  point_query.set_point(loc);

  // create a point query context (Embree needs to initialize this object)
  RTCPointQueryContext pq_context;
  rtcInitPointQueryContext(&pq_context);

  RTCScene scene = scene_map[volume];

  // perform the point query on the Embree scene
  rtcPointQuery(scene, &point_query, &pq_context,
                (RTCPointQueryFunction)DblTriPointQueryFunc, (void*)&scene);

  // handle case where no triangle is found
  if (point_query.geomID == RTC_INVALID_GEOMETRY_ID) {
    if (surface) { surface = 0; }
    if (facet) { facet = 0; }
    result = inf;
    return;
  }

  // set the result using the double-precision value of the query result
  result = point_query.dradius;

  // determine what scene
  RTCGeometry g = rtcGetGeometry(scene, point_query.geomID);
  // look up the triangle buffer array we stored on this geometry
  void* tris_i = rtcGetGeometryUserData(g);
  // find the DblTri in the array using the primitive ID as an index
  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[point_query.primID];

  // set the outgoing surface and triangle data
  if (surface) { *surface = this_tri.surf; }
  if (facet) { *facet = this_tri.handle; }
}

void RayTracingInterface::shutdown() {
  // release the device
  if (g_device) { rtcReleaseDevice (g_device); }
}

void RayTracingInterface::fire(moab::EntityHandle volume, RTCDRayHit &rayhit)
{
  {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene_map[volume],&context,(RTCRayHit*)&rayhit);
    rayhit.hit.Ng_x = -rayhit.hit.Ng_x;
    rayhit.hit.Ng_y = -rayhit.hit.Ng_y;
    rayhit.hit.Ng_z = -rayhit.hit.Ng_z;
  }

}

// point_in_volume_slow, including poly_solid_angle helper subroutine
// are adapted from "Point in Polyhedron Testing Using Spherical Polygons", Paulo Cezar
// Pinto Carvalho and Paulo Roma Cavalcanti, _Graphics Gems V_, pg. 42.  Original algorithm
// was described in "An Efficient Point In Polyhedron Algorithm", Jeff Lane, Bob Magedson,
// and Mike Rarick, _Computer Vision, Graphics, and Image Processing 26_, pg. 118-225, 1984.

// helper function for point_in_volume_slow.  calculate area of a polygon
// projected into a unit-sphere space
moab::ErrorCode RayTracingInterface::poly_solid_angle(moab::EntityHandle face,
                                                      const moab::CartVect& point,
                                                      double& solid_angle)
{
  ErrorCode rval;

  // Get connectivity
  const moab::EntityHandle* conn;
  int len;
  rval = MBI->get_connectivity( face, conn, len, true );
  MB_CHK_SET_ERR(rval, "Failed to get the connectivity of the polygon");

  // Allocate space to store vertices
  moab::CartVect coords_static[4];
  std::vector<CartVect> coords_dynamic;
  moab::CartVect* coords = coords_static;
  if ((unsigned)len > (sizeof(coords_static)/sizeof(coords_static[0]))) {
    coords_dynamic.resize(len);
    coords = &coords_dynamic[0];
  }

  // get coordinates
  rval = MBI->get_coords( conn, len, coords->array() );
  MB_CHK_SET_ERR(rval, "Failed to get the coordinates of the polygon vertices");

  // calculate normal
  moab::CartVect norm(0.0), v1, v0 = coords[1] - coords[0];
  for (int i = 2; i < len; ++i) {
    v1 = coords[i] - coords[0];
    norm += v0 * v1;
    v0 = v1;
  }

  // calculate area
  double s, ang;
  solid_angle = 0.0;
  moab::CartVect r, n1, n2, b, a = coords[len-1] - coords[0];
  for (int i = 0; i < len; ++i) {
    r = coords[i] - point;
    b = coords[(i+1)%len] - coords[i];
    n1 = a * r; // = norm1 (magnitude is important)
    n2 = r * b; // = norm2 (magnitude is important)
    s = (n1 % n2) / (n1.length() * n2.length()); // = cos(angle between norm1,norm2)
    ang = s <= -1.0 ? M_PI : s >= 1.0 ? 0.0 : acos(s); // = acos(s)
    s = (b * a) % norm; // =orientation of triangle wrt point
    solid_angle += s > 0.0 ? M_PI - ang : M_PI + ang;
    a = -b;
  }

  solid_angle -= M_PI * (len - 2);
  if ((norm % r) > 0)
    solid_angle = -solid_angle;
  return MB_SUCCESS;
}

moab::ErrorCode
RayTracingInterface::point_in_volume_slow(moab::EntityHandle volume,
                                          const double xyz[3],
                                          int& result)
{
  moab::ErrorCode rval;
  moab::Range faces;
  std::vector<moab::EntityHandle> surfs;
  std::vector<int> senses;
  double sum = 0.0;
  const moab::CartVect point(xyz);

  rval = MBI->get_child_meshsets( volume, surfs );
  MB_CHK_SET_ERR(rval, "Failed to get the volume's child surfaces");

  senses.resize( surfs.size() );
  rval = GTT->get_surface_senses( volume, surfs.size(), &surfs[0], &senses[0] );
  MB_CHK_SET_ERR(rval, "Failed to get the volume's surface senses");

  for (unsigned i = 0; i < surfs.size(); ++i) {
    if (!senses[i])  // skip non-manifold surfaces
      continue;

    double surf_area = 0.0, face_area;
    faces.clear();
    rval = MBI->get_entities_by_dimension( surfs[i], 2, faces );
    MB_CHK_SET_ERR(rval, "Failed to get the surface entities by dimension");

    for (moab::Range::iterator j = faces.begin(); j != faces.end(); ++j) {
      rval = poly_solid_angle( *j, point, face_area );
      MB_CHK_SET_ERR(rval, "Failed to determin the polygon's solid angle");

      surf_area += face_area;
    }

    sum += senses[i] * surf_area;
  }

  result = fabs(sum) > 2.0*M_PI;
  return MB_SUCCESS;
}

moab::ErrorCode
RayTracingInterface::point_in_volume(const moab::EntityHandle volume,
                                     const double xyz[3],
                                     int& result,
                                     const double *uvw,
                                     const moab::GeomQueryTool::RayHistory *history,
                                     double overlap_tol)
{
  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;

  RTCScene scene = scene_map[volume];

  // use the direction if provided, otherwise use any random direction
  std::array<double, 3> dir{0.5, 0.5, 0.5};
  if (uvw) {
    dir = {uvw[0], uvw[1], uvw[2]};
  }

  // struct that holds all info about the intersection
  MBRayHit mbrayhit;

  // set information about the ray
  MBRay& mbray = mbrayhit.ray;
  mbray.set_org(xyz);
  mbray.set_dir(dir.data());
  mbray.rf_type = RayFireType::PIV;
  mbray.orientation = 1;
  mbray.mask = -1; // no mask
  mbray.tnear = 0.0; // no minimum distance befor the intersection
  mbray.set_len(dist_limit);
  // set the history if present
  if (history) { mbray.rh = history; }

  // set initial hit data
  MBHit& mbhit = mbrayhit.hit;
  mbhit.geomID = RTC_INVALID_GEOMETRY_ID;
  mbhit.primID = RTC_INVALID_GEOMETRY_ID;

  // fire ray
  {
   RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene,&context,(RTCRayHit*)&mbrayhit);
    // triangle normal conventions are the flipped in Embree compared to MOAB
    // TODO: reverse the initial triangle normals so we don't have to do these ops here.
    mbhit.Ng_x = -mbhit.Ng_x;
    mbhit.Ng_y = -mbhit.Ng_y;
    mbhit.Ng_z = -mbhit.Ng_z;
  }

  Vec3da ray_dir(dir.data());
  Vec3da tri_norm(mbrayhit.hit.dNg[0], mbrayhit.hit.dNg[1], mbrayhit.hit.dNg[2]);

  // use the triangle normal vs. ray direction to determine if this is an entering
  // or exiting intersection
  if (mbrayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
    result = dot(ray_dir, tri_norm) > 0.0 ? 1 : 0;
  }
  else {
    result = 0;
  }

  return moab::MB_SUCCESS;
}

void
RayTracingInterface::boundary_case(moab::EntityHandle volume,
                                   int& result,
                                   double u,
                                   double v,
                                   double w,
                                   moab::EntityHandle facet,
                                   moab::EntityHandle surface)
{
  moab::EntityHandle rval;

  // TODO: Improve this check for direction
  if (u <= 1.0 && v <= 1.0 && w <= 1.0) {

    Vec3da uvw(u, v, w);
    Vec3da coords[3], normal(0.0);
    std::vector<moab::EntityHandle> conn;
    int sense_out;

    rval = MBI->get_connectivity(&facet, 1, conn);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get facet connectivity");

    rval = MBI->get_coords(&conn[0], 1, &(coords[0][0]));
    MB_CHK_SET_ERR_CONT(rval, "Failed to get vertex coordinates");
    rval = MBI->get_coords(&conn[1], 1, &(coords[1][0]));
    MB_CHK_SET_ERR_CONT(rval, "Failed to get vertex coordinates");
    rval = MBI->get_coords(&conn[2], 1, &(coords[2][0]));
    MB_CHK_SET_ERR_CONT(rval, "Failed to get vertex coordinates");

    rval = GTT->get_sense(surface, volume, sense_out);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the surface sense");

    // vectors along two of the triangle edges
    coords[1] -= coords[0];
    coords[2] -= coords[0];

    normal = sense_out * cross(coords[1], coords[2]);

    double sense = dot(uvw, normal);

    if ( sense < 0.0 ) {
      result = 1;     // inside or entering
    } else  if ( sense > 0.0 ) {
      result = 0;     // outside or leaving
    } else  if ( sense == 0.0 ) {
      result = -1;    // tangent, therefore on boundary
    } else {
      result = -1;    // failure
      MB_SET_ERR_CONT(moab::MB_FAILURE);
    }
  } else {
    result = -1;
  }
}

moab::ErrorCode
RayTracingInterface::test_volume_boundary(const moab::EntityHandle volume,
                                          const moab::EntityHandle surface,
                                          const double xyz[3],
                                          const double uvw[3],
                                          int& result,
                                          const moab::GeomQueryTool::RayHistory* history)
{
  moab::ErrorCode rval;

  int dir;
  moab::EntityHandle last_facet_hit = 0;
  if (history) { history->get_last_intersection(last_facet_hit); }

  if (last_facet_hit != 0) {
    boundary_case(volume, dir, uvw[0], uvw[1], uvw[2], last_facet_hit, surface);
  } else {
    // find nearest facet
    moab::EntityHandle surf, facet;
    double dist;
    closest(volume, xyz, dist, &surf, &facet);
    boundary_case(volume, dir, uvw[0], uvw[1], uvw[2], facet, surface);
  }

  result = dir;
  return moab::MB_SUCCESS;
}

moab::ErrorCode
RayTracingInterface::ray_fire(const moab::EntityHandle volume,
                              const double point[3],
                              const double dir[3],
                              moab::EntityHandle& next_surf,
                              double& next_surf_dist,
                              moab::GeomQueryTool::RayHistory* history,
                              double user_dist_limit,
                              int ray_orientation,
                              void* dum)
{

  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;
  if (user_dist_limit > 0) { dist_limit = user_dist_limit; }

  next_surf = 0;
  next_surf_dist = huge_val;

  RTCScene scene = scene_map[volume];

  // struct that stores information about the ray and hit
  MBRayHit rayhit;

  // set information for the ray
  MBRay& mbray  = rayhit.ray;
  mbray.set_org(point);
  mbray.set_dir(dir);
  mbray.tnear = 0.0;
  mbray.set_len(dist_limit);
  mbray.rf_type = RayFireType::RF;
  mbray.orientation = ray_orientation;
  mbray.mask = -1; // no mask
  if (history) { mbray.rh = history; }

  // set initial hit information (no geom or primitive)
  MBHit& mbhit = rayhit.hit;
  mbhit.geomID = RTC_INVALID_GEOMETRY_ID;
  mbhit.primID = RTC_INVALID_GEOMETRY_ID;

  // fire ray
  {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene,&context,(RTCRayHit*)&rayhit);
    mbhit.Ng_x = -mbhit.Ng_x;
    mbhit.Ng_y = -mbhit.Ng_y;
    mbhit.Ng_z = -mbhit.Ng_z;
  }

  // check behind the ray origin for intersections
  MBRayHit neg_rayhit;

  MBRay& neg_ray = neg_rayhit.ray;
  neg_ray.set_org(point);
  neg_ray.set_dir(-mbray.ddir); // fire in the opposite direction
  neg_ray.tnear = 0.0;
  neg_ray.rf_type = RayFireType::RF;
  neg_ray.orientation = -ray_orientation; // opposite ray orientation to match flipped direction
  neg_ray.set_len(overlap_thickness);
  if (history) { neg_ray.rh = history; }

  // set initial hit information (no geom or primitive)
  MBHit& neg_hit = neg_rayhit.hit;
  neg_hit.geomID = RTC_INVALID_GEOMETRY_ID;
  neg_hit.primID = RTC_INVALID_GEOMETRY_ID;

  // fire ray in negative direction
  if (overlap_thickness > 0.0) {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene,&context,(RTCRayHit*)&neg_ray);
    neg_hit.Ng_x = -neg_hit.Ng_x;
    neg_hit.Ng_y = -neg_hit.Ng_y;
    neg_hit.Ng_z = -neg_hit.Ng_z;
  }

  bool use_neg_intersection = false;
  // If an RTI is found at negative distance, perform a PMT to see if the
  // particle is inside an overlap.
  if(neg_hit.geomID != RTC_INVALID_GEOMETRY_ID) {
    moab::ErrorCode rval;
    // get the next volume
    std::vector<moab::EntityHandle> vols;
    moab::EntityHandle nx_vol;
    rval = MBI->get_parent_meshsets( neg_hit.surf_handle, vols );
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the parent meshsets");
    if (2 != vols.size()) {
      MB_CHK_SET_ERR_CONT(moab::MB_FAILURE, "Invaid number of parent volumes found");
    }
    if(vols.front() == volume) {
      nx_vol = vols.back();
    } else {
      nx_vol = vols.front();
    }
    int result = 0;
    point_in_volume( nx_vol, point, result, dir, history );
    MB_CHK_SET_ERR_CONT(rval, "Point in volume query failed");
    if (1 == result) use_neg_intersection = true;
  }

  if(use_neg_intersection && neg_hit.geomID != RTC_INVALID_GEOMETRY_ID) {
    next_surf_dist = 0;
    next_surf = neg_hit.surf_handle;
  }
  else if ( mbhit.geomID != RTC_INVALID_GEOMETRY_ID) {
    next_surf_dist = mbray.dtfar;
    next_surf = mbhit.surf_handle;
  }
  else {
    next_surf_dist = 1E37;
    next_surf = 0;
  }

  if (history) {
    if(use_neg_intersection) {
      history->add_entity(neg_hit.prim_handle);
    }
    else {
      history->add_entity(mbhit.prim_handle);
    }
  }

  return MB_SUCCESS;
}

moab::ErrorCode
RayTracingInterface::get_vols(moab::Range& vols)
{
  moab::ErrorCode rval;

  // retrieve vols using the geom tag
  moab::Tag geom_tag;
  rval = MBI->tag_get_handle("GEOM_DIMENSION", geom_tag);
  MB_CHK_SET_ERR(rval, "Failed to get the geometry dimension tag");

  int val = 3;
  const void *dum = &val;
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_tag, &dum, 1, vols);
  MB_CHK_SET_ERR(rval, "Failed to get all surface sets in the model");

  return moab::MB_SUCCESS;
}

bool
RayTracingInterface::has_bvh() const {
  return scene_map.size() > 0;
}
