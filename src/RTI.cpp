
#include <string>

// MOAB
#include "MBTagConventions.hpp"
#include "moab/GeomTopoTool.hpp"

// Local
#include "RTI.hpp"
#include "AABB.h"


void error(void* dum, RTCError code, const char* str) {
  if (code != RTC_ERROR_NONE) {
    std::cout << "Error occured" << std::endl;
    std::string msg(str);
    std::cout << msg << std::endl;
  }
}

struct Node
{
  virtual float sah() = 0;
};



  struct InnerNode : public Node
  {
    AABB bounds[2];
    Node* children[2];

    InnerNode() {
      bounds[0] = bounds[1] = empty;
      children[0] = children[1] = nullptr;
    }


    float sah() {
      return 1.0f + (area(bounds[0])*children[0]->sah() + area(bounds[1])*children[1]->sah())/area(merge(bounds[0],bounds[1]));
    }

    static void* create  (RTCThreadLocalAllocator alloc, unsigned int numChildren, void * userPtr)
    {
      //      assert(numChildren == 2);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(InnerNode),16);
      return (void*) new (ptr) InnerNode;
    }

    static void  setChildren  (void* nodePtr, void** childPtr, unsigned int numChildren, void * userPtr)
    {
      //      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr[i];
    }

    static void  setBounds  (void* nodePtr, const RTCBounds** bounds, unsigned int numChildren, void* userPtr)
    {
      //      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->bounds[i] = *(const AABB*) bounds[i];
    }
  };

  struct LeafNode : public Node
  {
    std::vector<unsigned> ids;
    AABB bounds;

    LeafNode (std::vector<unsigned> ids, const AABB& bounds)
      : ids(ids), bounds(bounds) {
    }

    float sah() {
      return 1.0f;
    }

    static void* create (RTCThreadLocalAllocator alloc, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr)
    {
      //      assert(numPrims == 1);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(LeafNode),16);
      if (numPrims == 0) {
        return NULL;
      }
      else {
        std::vector<unsigned> ids;
        for (int i = 0; i < numPrims; i++) {
          ids.emplace_back(prims[i].primID);
        }
        return (void*) new (ptr) LeafNode(std::move(ids),*(AABB*)prims);
      }
    }
  };

template<typename T>
struct StackItemT {

  inline static void xchg(StackItemT& a, StackItemT& b) {
    std::swap(a,b);
  }

  inline friend void sort(StackItemT& s1, StackItemT& s2) {
    if (s1.dist < s2.dist) { xchg(s2, s1); }
  }

  inline friend void sort(StackItemT& s1, StackItemT& s2, StackItemT& s3) {
    if (s2.dist < s1.dist) xchg(s2,s1);
    if (s3.dist < s2.dist) xchg(s3,s2);
    if (s2.dist < s1.dist) xchg(s2,s1);
  }

  inline friend void sort(StackItemT& s1, StackItemT& s2, StackItemT& s3, StackItemT& s4) {
    if (s2.dist < s1.dist) xchg(s2,s1);
    if (s4.dist < s3.dist) xchg(s4,s3);
    if (s3.dist < s1.dist) xchg(s3,s1);
    if (s4.dist < s2.dist) xchg(s4,s2);
    if (s3.dist < s2.dist) xchg(s3,s2);
  }

public:
  T ptr;
  unsigned dist;

};

typedef StackItemT<Node*> StackItem;

moab::ErrorCode RayTracingInterface::init(std::string filename, bool closest_enabled)
{

  closest_enabled_ = closest_enabled;

  moab::ErrorCode rval;

  if ("" == filename && NULL == MBI) {
    return moab::MB_FAILURE;
  } else if (!MBI) {
    MBI = new moab::Core();
    rval = MBI->load_file(filename.c_str());
    MB_CHK_SET_ERR(rval, "Failed to load the specified MOAB file");
  }

  g_device = rtcNewDevice(NULL);

  rtcSetDeviceErrorFunction(g_device, (RTCErrorFunction)error, NULL);

  mdam = new MBDirectAccess(MBI);

  moab::Range vols;
  rval = get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get MOAB volumes");

  std::cout << "Found " << vols.size() << " volumes." << std::endl;
  // set the EH offset
  sceneOffset = *vols.begin();

  GTT = std::unique_ptr<moab::GeomTopoTool>(new moab::GeomTopoTool(MBI));
  // create an Embree geometry instance for each surface
  for (moab::Range::iterator i = vols.begin(); i != vols.end(); i++) {
    moab::EntityHandle vol = *i;

    // add to scenes vec
    RTCScene scene = rtcNewScene(g_device);
    rtcSetSceneFlags(scene,RTC_SCENE_FLAG_ROBUST ); // EMBREE_FIXME: set proper scene flags
    rtcSetSceneBuildQuality(scene,RTC_BUILD_QUALITY_HIGH); // EMBREE_FIXME: set proper build quality

    scenes.push_back(scene);
    scene_map[vol] = scene;

    // get volume surfaces
    moab::Range surfs;
    rval = MBI->get_child_meshsets(vol, surfs);
    MB_CHK_SET_ERR(rval, "Failed to get the volume sufaces");

    int num_vol_tris = 0;
    // allocate triangle buffer
    for (moab::Range::iterator j = surfs.begin(); j != surfs.end(); j++) {
      int num_surf_tris;
      rval = MBI->get_number_entities_by_type(*j, moab::MBTRI, num_surf_tris);
      MB_CHK_SET_ERR(rval, "Failed to get triangle count");
      num_vol_tris += num_surf_tris;
    }

    std::shared_ptr<DblTri> emtris((DblTri*) malloc(num_vol_tris*sizeof(DblTri)));

    buffer_storage.store(vol, num_vol_tris, emtris);

    int buffer_start = 0;
    for (moab::Range::iterator j = surfs.begin(); j != surfs.end(); j++) {
      moab::EntityHandle this_surf = *j;
      // find the surface sense
      int sense;
      rval = GTT->get_sense(this_surf, vol, sense);
      MB_CHK_SET_ERR(rval, "Failed to get sense for surface");

      // get all triangles on this surface
      moab::Range tris;
      rval = MBI->get_entities_by_type(this_surf, moab::MBTRI, tris);
      MB_CHK_SET_ERR(rval, "Failed to get surface triangles");

      int num_tris = tris.size();

      // create a new geometry for the volume's scene
      RTCGeometry geom_0 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_USER); // EMBREE_FIXME: check if geometry gets properly committed
      rtcSetGeometryBuildQuality(geom_0,RTC_BUILD_QUALITY_HIGH);
      rtcSetGeometryUserPrimitiveCount(geom_0,num_tris);
      rtcSetGeometryTimeStepCount(geom_0,1);
      unsigned int emsurf = rtcAttachGeometry(scene,geom_0);

      DblTri* buff_ptr = emtris.get() + buffer_start;

      rtcSetGeometryUserData(geom_0, buff_ptr);

      for (int k = 0; k < num_tris; k++) {
        buff_ptr[k].moab_instance = MBI;
        buff_ptr[k].mdam = mdam;
        buff_ptr[k].handle = tris[k];
        buff_ptr[k].surf = this_surf;
        buff_ptr[k].geomID = emsurf;
        buff_ptr[k].sense = sense;
      } // end tris loop

      buffer_start += num_tris;

      rtcSetGeometryBoundsFunction(geom_0,(RTCBoundsFunction)&DblTriBounds, NULL);
      rtcSetGeometryIntersectFunction (geom_0, (RTCIntersectFunctionN)&MBDblTriIntersectFunc);
      rtcSetGeometryOccludedFunction (geom_0, (RTCOccludedFunctionN)&DblTriOccludedFunc);
      // rtcSetGeometryPointQueryFunction(geom_0,(RTCPointQueryFunction)DblTriPointQueryFunc);
      rtcCommitGeometry(geom_0);

    } // end surface loop

    rtcCommitScene(scene);

  } // end volume loop

  return moab::MB_SUCCESS;
}

void RayTracingInterface::buildBVH(moab::EntityHandle vol) {

  RTCDevice device = rtcNewDevice (NULL);

  RTCBVH bvh = rtcNewBVH(device);

  /* settings for BVH build */
  const size_t extraSpace = 1000000;
  RTCBuildArguments settings;
  settings.byteSize = sizeof(settings);
  settings.buildQuality = RTC_BUILD_QUALITY_HIGH;
  settings.maxBranchingFactor = 2;
  settings.maxDepth = 1024;
  settings.sahBlockSize = 1;
  settings.minLeafSize = 1;
  settings.maxLeafSize = 20;
  settings.traversalCost = 1.0f;
  settings.intersectionCost = 1.0f;

  std::pair<int, std::shared_ptr<DblTri>> buffer = buffer_storage.retrieve_buffer(vol);

  std::vector<RTCBuildPrimitive> prims;
  prims.resize(buffer.first);

  for(int i = 0; i < buffer.first; i++) {
    DblTri dtri = buffer.second.get()[i];

    RTCBounds bounds = DblTriBounds((moab::Interface*)dtri.moab_instance,
                                    dtri.handle);
    RTCBuildPrimitive prim;
    prim.lower_x = bounds.lower_x;
    prim.lower_y = bounds.lower_y;
    prim.lower_z = bounds.lower_z;
    prim.geomID = 0;
    prim.upper_x = bounds.upper_x;
    prim.upper_y = bounds.upper_y;
    prim.upper_z = bounds.upper_z;
    prim.primID = i;
    prims[i] = prim;
  }

  settings.bvh = bvh;
  settings.primitives = prims.data();
  settings.primitiveCount = prims.size();
  settings.createNode = InnerNode::create;
  settings.setNodeChildren = InnerNode::setChildren;
  settings.setNodeBounds = InnerNode::setBounds;
  settings.createLeaf = LeafNode::create;
  settings.splitPrimitive = NULL;
  settings.buildProgress = NULL;
  settings.userPtr = NULL;
  settings.primitiveArrayCapacity = extraSpace;
  // EMBREE_FIXME: calculate capacity properly = extraSpace;

  Node * root = (Node*) rtcBuildBVH(&settings);

  root_map[vol] = root;

}

void RayTracingInterface::get_normal(moab::EntityHandle surf, const double loc[3],
                                     double angle[3], moab::EntityHandle facet) {

  moab::ErrorCode rval;

  if (facet == 0) {
     MB_CHK_SET_ERR_CONT(rval, "Can't get a normal without a history yet");
     return;
  }


  Vec3da coords[3];
  const moab::EntityHandle *conn;
  int len;
  rval = MBI->get_connectivity(facet, conn, len);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get connectivity of triangle: " << facet);

  if (3 != len) {
    MB_CHK_SET_ERR_CONT(rval, "Incorrect connectivity length for a triangle: " << len);
  }

  rval = MBI->get_coords(conn, 3, &(coords[0][0]));
  MB_CHK_SET_ERR_CONT(rval, "Failed to get coords for triangle: " << facet);

  Vec3da normal(0.0);

  coords[1] -= coords[0];
  coords[2] -= coords[0];
  normal += cross(coords[1], coords[2]);

  angle[0] = normal[0];
  angle[1] = normal[1];
  angle[2] = normal[2];
}

void RayTracingInterface::closest(moab::EntityHandle vol, const double loc[3],
                                  double &result, moab::EntityHandle* surface,
                                  moab::EntityHandle* facet) {

  std::pair<int, std::shared_ptr<DblTri>> buffer = buffer_storage.retrieve_buffer(vol);

  RTCDPointQuery point_query;
  point_query.set_radius(inf);
  point_query.time = 0.f;
  point_query.set_point(loc);

  RTCPointQueryContext pq_context;
  rtcInitPointQueryContext(&pq_context);

  RTCScene scene = scenes[vol-sceneOffset];

  rtcPointQuery(scene, &point_query, &pq_context,
                (RTCPointQueryFunction)DblTriPointQueryFunc, (void*)&scene);

  // handle not found case
  if (point_query.geomID == RTC_INVALID_GEOMETRY_ID) {
    if (surface) { surface = 0; }
    if (facet) { facet = 0; }
    result = inf;
    return;
  }

  result = point_query.dradius;
  const DblTri& this_tri = buffer.second.get()[point_query.primID];
  if (surface) {
    *surface = this_tri.surf;
  }
  if (facet) {
    *facet = this_tri.handle;
  }

  return;

}

void RayTracingInterface::shutdown() {
  for(auto s : scenes) { rtcReleaseScene(s); }

  if(MBI) { delete MBI; }
  delete mdam;

  rtcReleaseDevice (g_device);
}

void RayTracingInterface::fire(moab::EntityHandle vol, RTCDRayHit &rayhit) {

  {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scenes[vol-sceneOffset],&context,(RTCRayHit*)&rayhit);
    rayhit.hit.Ng_x = -rayhit.hit.Ng_x; // EMBREE_FIXME: only correct for triangles,quads, and subdivision surfaces
    rayhit.hit.Ng_y = -rayhit.hit.Ng_y;
    rayhit.hit.Ng_z = -rayhit.hit.Ng_z;
  }

}

void RayTracingInterface::dag_point_in_volume(const moab::EntityHandle volume,
                                              const double xyz[3],
                                              int& result,
                                              const double *uvw,
                                              moab::GeomQueryTool::RayHistory *history,
                                              double overlap_tol,
                                              moab::EntityHandle imp_comp) {
  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;

   RTCScene scene = scene_map[volume];

  double dir[3];
  if (uvw) {
    dir[0] = uvw[0];
    dir[1] = uvw[1];
    dir[2] = uvw[2];
  } else {
    dir[0] = 0.5;
    dir[1] = 0.5;
    dir[2] = 0.5;
  }

  MBRayHit mbrayhit;

  MBRay& mbray = mbrayhit.ray;
  mbray.set_org(xyz);
  mbray.set_dir(dir);
  mbray.rf_type = RayFireType::PIV;
  mbray.orientation = 1;
  mbray.mask = -1;
  mbray.tnear = 0.0;
  mbray.set_len(dist_limit);
  if (history) { mbray.rh = history; }

  MBHit& mbhit = mbrayhit.hit;
  mbhit.geomID = RTC_INVALID_GEOMETRY_ID;
  mbhit.primID = RTC_INVALID_GEOMETRY_ID;

  // fire ray
  {
   RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene,&context,(RTCRayHit*)&mbrayhit);
    mbhit.Ng_x = -mbhit.Ng_x; // EMBREE_FIXME: only correct for triangles,quads, and subdivision surfaces
    mbhit.Ng_y = -mbhit.Ng_y;
    mbhit.Ng_z = -mbhit.Ng_z;
  }

  Vec3da ray_dir(dir);
  Vec3da tri_norm(mbrayhit.hit.dNg[0], mbrayhit.hit.dNg[1], mbrayhit.hit.dNg[2]);

  if (mbrayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
    result = dot(ray_dir, tri_norm) > 0.0 ? 1 : 0;
  }
  else {
    result = 0;
  }

//   if (overlap_tol != 0.0) {
//     MBRayAccumulate aray;
//     aray.set_org(xyz);
//     aray.set_dir(dir);
//     aray.instID = volume;
//     aray.set_len(dist_limit);
//     aray.rf_type = RayFireType::ACCUM;
//     aray.sum = 0;
//     aray.num_hit = 0;

//     {
//       RTCIntersectContext context;
//       rtcInitIntersectContext(&context);
//       rtcIntersect1(scene,&context,&*((RTCRay*)&aray));
//       *((RTCRay*)&aray).hit.Ng_x = -*((RTCRay*)&aray).hit.Ng_x; // EMBREE_FIXME: only correct for triangles,quads, and subdivision surfaces
//       *((RTCRay*)&aray).hit.Ng_y = -*((RTCRay*)&aray).hit.Ng_y;
//       *((RTCRay*)&aray).hit.Ng_z = -*((RTCRay*)&aray).hit.Ng_z;
//     }
//     if (aray.num_hit == 0) { result = 0; return; }
//     int hits = aray.num_hit;

//     aray.geomID = -1;
//     aray.primID = -1;
//     aray.set_len(dist_limit);
//     aray.tnear = 0.0;
//     aray.set_dir(aray.ddir*-1);
//     {
//       RTCIntersectContext context;
//       rtcInitIntersectContext(&context);
//       rtcIntersect1(scene,&context,&*((RTCRay*)&aray));
//       *((RTCRay*)&aray).hit.Ng_x = -*((RTCRay*)&aray).hit.Ng_x; // EMBREE_FIXME: only correct for triangles,quads, and subdivision surfaces
//       *((RTCRay*)&aray).hit.Ng_y = -*((RTCRay*)&aray).hit.Ng_y;
//       *((RTCRay*)&aray).hit.Ng_z = -*((RTCRay*)&aray).hit.Ng_z;
//     }
//     if (aray.num_hit == hits) { result = 0; return; }
//     // inside/outside depends on the sum
//     if      (0 < aray.sum)                                          result = 0; // pt is outside (for all vols)
//     else if (0 > aray.sum)                                          result = 1; // pt is inside  (for all vols)
//     else if (imp_comp && imp_comp == volume)                        result = 1;
//     else                                                            result = 0;
//   }

}


void RayTracingInterface::boundary_case(moab::EntityHandle volume,
                                        int& result,
                                        double u,
                                        double v,
                                        double w,
                                        moab::EntityHandle facet,
                                        moab::EntityHandle surface) {

  moab::EntityHandle rval;

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

void RayTracingInterface::test_volume_boundary(const moab::EntityHandle volume,
                                               const moab::EntityHandle surface,
                                               const double xyz[3], const double uvw[3], int& result,
                                               const moab::GeomQueryTool::RayHistory* history) {

  moab::ErrorCode rval;
  int dir;

  // if (history && history->prev_facets.size()) {
  //   boundary_case(volume, dir, uvw[0], uvw[1], uvw[2], history->prev_facets.back(), surface);
  // } else {
    // find nearest facet
    moab::EntityHandle surf, facet;
    double dist;
    closest(volume, xyz, dist, &surf, &facet);

    boundary_case(volume, dir, uvw[0], uvw[1], uvw[2], facet, surface);
    //  }

  result = dir;
}

void RayTracingInterface::dag_ray_fire(const moab::EntityHandle volume,
                                       const double point[3],
                                       const double dir[3],
                                       moab::EntityHandle& next_surf,
                                       double& next_surf_dist,
                                       moab::GeomQueryTool::RayHistory* history,
                                       double user_dist_limit,
                                       int ray_orientation) {

  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;
  if (user_dist_limit > 0) { dist_limit = user_dist_limit; }

  next_surf = 0; next_surf_dist = huge_val;

  RTCScene scene = scene_map[volume];

  MBRayHit rayhit;

  MBRay& mbray  = rayhit.ray;
  mbray.set_org(point);
  mbray.set_dir(dir);
  mbray.tnear = 0.0;
  mbray.set_len(dist_limit);
  mbray.rf_type = RayFireType::RF;
  mbray.orientation = ray_orientation;
  mbray.mask = -1;
  if (history) { mbray.rh = history; }


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
  double neg_ray_len = 1e-03;

  MBRayHit neg_rayhit;

  MBRay& neg_ray = neg_rayhit.ray;
  neg_ray.set_org(point);
  neg_ray.set_dir(-mbray.ddir);
  neg_ray.tnear = 0.0;
  neg_ray.rf_type = RayFireType::RF;
  neg_ray.orientation = ray_orientation;
  if (history) { neg_ray.rh = history; }
  neg_ray.set_len(neg_ray_len);

  MBHit& neg_hit = neg_rayhit.hit;
  neg_hit.geomID = RTC_INVALID_GEOMETRY_ID;
  neg_hit.primID = RTC_INVALID_GEOMETRY_ID;

  // fire ray in negative direction
  {
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
    dag_point_in_volume( nx_vol, point, result, dir, history );
    MB_CHK_SET_ERR_CONT(rval, "Point in volume query failed");
    if (1==result) use_neg_intersection = true;
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

  if(history) {
    if(use_neg_intersection) {
      history->add_entity(neg_hit.prim_handle);
    }
    else {
      history->add_entity(mbhit.prim_handle);
    }
  }

}

moab::ErrorCode RayTracingInterface::get_vols(moab::Range& vols) {
  moab::ErrorCode rval;

  // retrieve vols
  moab::Tag geom_tag;
  rval = MBI->tag_get_handle("GEOM_DIMENSION", geom_tag);
  MB_CHK_SET_ERR(rval, "Failed to get the geometry dimension tag");

  int val = 3;
  const void *dum = &val;
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_tag, &dum, 1, vols);
  MB_CHK_SET_ERR(rval, "Failed to get all surface sets in the model");

  return moab::MB_SUCCESS;
}
