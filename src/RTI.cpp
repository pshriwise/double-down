
// MOAB
#include "MBTagConventions.hpp"

// Local
#include "RTI.hpp"

moab::ErrorCode RayTracingInterface::init(std::string filename) {
  moab::ErrorCode rval;
  
  if ("" == filename && NULL == MBI) {
    return moab::MB_FAILURE; 
  } else if (!MBI) {
    MBI = new moab::Core();
    rval = MBI->load_file(filename.c_str());
    MB_CHK_SET_ERR(rval, "Failed to load the specified MOAB file");
  }
    
  rtcInit();

  moab::Range vols;
  rval = get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get MOAB volumes.");
  
  // set the EH offset
  sceneOffset = *vols.begin();
  
  // create an Embree geometry instance for each surface
  for (moab::Range::iterator i = vols.begin();
       i != vols.end(); i++) {
    moab::EntityHandle vol = *i;
    
    // add to scenes vec
    RTCScene scene = rtcNewScene(RTC_SCENE_ROBUST , RTC_INTERSECT1);
    scenes.push_back(scene);
    
    // get volume surfaces
    moab::Range surfs;
    rval = MBI->get_child_meshsets(vol, surfs);
    MB_CHK_SET_ERR(rval, "Failed to get the volume sufaces.");

    for (moab::Range::iterator j = surfs.begin();
         j != surfs.end(); j++) {

      moab::EntityHandle this_surf = *j;
      
      // get all triangles on this surface
      moab::Range tris;
      rval = MBI->get_entities_by_type(this_surf, moab::MBTRI, tris);
      MB_CHK_SET_ERR(rval, "Failed to get surface triangles.");

      int num_tris = tris.size();

      // create a new geometry for the volume's scene
      unsigned int emsurf = rtcNewUserGeometry(scene, num_tris);

      DblTri* emtris = (DblTri*) malloc(num_tris*sizeof(DblTri));

      rtcSetUserData(scene, emsurf, emtris);
      
      tri_buffers.push_back(emtris);
      for (int k = 0; k < num_tris; k++) {
        emtris[k].moab_instance = MBI;
        emtris[k].handle = tris[k];
        emtris[k].geomID = emsurf;
      } // end tris loop

      rtcSetIntersectionFilterFunction(scene, emsurf, (RTCFilterFunc)&DblTriIntersectFunc);
      rtcSetBoundsFunction(scene, emsurf, (RTCBoundsFunc)&DblTriBounds);
      rtcSetIntersectFunction(scene, emsurf, (RTCIntersectFunc)&DblTriIntersectFunc);
      rtcSetOccludedFunction(scene, emsurf, (RTCOccludedFunc)&DblTriOccludedFunc);
    
    } // end surface loop

    rtcCommit(scene);

  } // end volume loop

  return moab::MB_SUCCESS;
}
void RayTracingInterface::shutdown() {
  for(auto s : scenes) { rtcDeleteScene(s); }

  for(auto b : tri_buffers) { free(b); }

  if(MBI) { delete MBI; }
  
  rtcExit();
}

void RayTracingInterface::fire(moab::EntityHandle vol, RTCDRay &ray) {

  rtcIntersect(scenes[vol-sceneOffset], *((RTCRay*)&ray));
  
}

void RayTracingInterface::dag_ray_fire(const moab::EntityHandle volume,
                                       const double point[3],
                                       const double dir[3],
                                       moab::EntityHandle& next_surf,
                                       double& next_surf_dist,
                                       void* history,
                                       double user_dist_limit,
                                       int ray_orientation) {
  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;
  if (user_dist_limit > 0) { dist_limit = user_dist_limit; }

  // don't recreate these every call
  std::vector<double>       dists;
  std::vector<moab::EntityHandle> surfs;
  std::vector<moab::EntityHandle> facets;

  // setup the ray
  RTCDRay ray;
  ray.set_org(point);
  ray.set_dir(dir);
  ray.set_len(dist_limit);
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.rf_type = RayFireType::RF;

  // fire ray
  RTCScene scene = scenes[volume - sceneOffset];
  rtcIntersect(scene, *((RTCRay*)&ray));

  next_surf_dist = ray.dtfar;
  next_surf = ray.geomID;
  
  
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
