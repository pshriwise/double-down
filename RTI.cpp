
// MOAB
#include "MBTagConventions.hpp"

// Local
#include "RTI.hpp"

moab::ErrorCode RayTracingInterface::init(std::string filename) {

  rtcInit();

  MBI = new moab::Core();
  
  moab::ErrorCode rval;

  rval = MBI->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load the specified MOAB file");

  // retrieve surfaces
  moab::Tag geom_tag;
  rval = MBI->tag_get_handle("GEOM_DIMENSION", geom_tag);
  MB_CHK_SET_ERR(rval, "Failed to get the geometry dimension tag");

  moab::Range vols;
  int val = 3;
  const void *dum = &val;
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_tag, &dum, 1, vols);
  MB_CHK_SET_ERR(rval, "Failed to get all surface sets in the model");

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
}
void RayTracingInterface::shutdown() {
  for(auto s : scenes) { rtcDeleteScene(s); }

  for(auto b : tri_buffers) { free(b); }

  if(MBI) { delete MBI; }
  
  rtcExit();
}
