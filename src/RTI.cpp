
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
      rtcSetIntersectFunction(scene, emsurf, (RTCIntersectFunc)&MBDblTriIntersectFunc);
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
                                       int& next_surf,
                                       double& next_surf_dist,
                                       moab::GeomQueryTool::RayHistory* history,
                                       double user_dist_limit,
                                       int ray_orientation) {

  const double huge_val = std::numeric_limits<double>::max();
  double dist_limit = huge_val;
  if (user_dist_limit > 0) { dist_limit = user_dist_limit; }

  RTCScene scene = scenes[volume - sceneOffset];
  
  MBRay mbray;
  mbray.set_org(point);
  mbray.set_dir(dir);

  mbray.tnear = 0.0;
  mbray.set_len(dist_limit);
  mbray.geomID = RTC_INVALID_GEOMETRY_ID;
  mbray.primID = RTC_INVALID_GEOMETRY_ID;
  mbray.rf_type = RayFireType::RF;
  mbray.orientation = ray_orientation;
  mbray.mask = -1;
  
  if (history) { mbray.rh = history; }
    
  // fire ray
  rtcIntersect(scene, *((RTCRay*)&mbray));

  // check behind the ray origin for intersections
  double neg_ray_len = 1e-03;

  MBRay neg_ray;
  neg_ray.set_org(point);
  neg_ray.set_dir(-mbray.ddir);
  neg_ray.tnear = 0.0;
  neg_ray.geomID = -1;
  neg_ray.primID = -1;
  neg_ray.rf_type = RayFireType::RF;
  neg_ray.orientation = -ray_orientation;
  if (history) { neg_ray.rh = history; }
  neg_ray.tfar = neg_ray_len;
  
  rtcIntersect(scene, *((RTCRay*)&neg_ray));

  bool use_neg_intersection = false;
  // If an RTI is found at negative distance, perform a PMT to see if the
  // particle is inside an overlap.
  if(neg_ray.geomID != RTC_INVALID_GEOMETRY_ID) {
    moab::ErrorCode rval;
    // get the next volume
    std::vector<moab::EntityHandle> vols;
    moab::EntityHandle nx_vol;
    rval = MBI->get_parent_meshsets( neg_ray.surf_handle, vols );
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
    //    rval = point_in_volume( nx_vol, point, result, dir, history );
    //    MB_CHK_SET_ERR_CONT(rval, "Point in volume query failed");
    if (1==result) use_neg_intersection = true;
  }

  if(use_neg_intersection) {
    next_surf_dist = 0;
    next_surf = neg_ray.geomID;
  }
  else if ( mbray.geomID != RTC_INVALID_GEOMETRY_ID) {
    next_surf_dist = mbray.tfar;
    next_surf = mbray.geomID;
  }
  else {
    next_surf_dist = 1E37;
    next_surf = 0;
  }

  if(history) {
    if(use_neg_intersection) {
      history->add_entity(neg_ray.prim_handle);
    }
    else { 
      history->add_entity(mbray.prim_handle);
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
