
#include "RTI.hpp"
#include "MBTagConventions.hpp"

moab::ErrorCode RayTracingInterface::init(std::string filename) {

  // rtcInit();

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
    /// rtcScene this_scene = rtcNewScene(RTC_SCENE_ROBUST,RTC_INTERSECT1)); \\\
    /// scenes.push_back(this_scene); \\\
    
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
      /// unsigned int emsurf = rtcNewUserGeometry(scene, num_tris); \\\

      DblTri* emtris = (DblTri*) malloc(num_tris*sizeof(DblTri));
      tri_buffers.push_back(emtris);
      for (int k = 0; k < num_tris; k++) {
        emtris[k].moab_instance = MBI;
        emtris[k].handle = tris[k];
        /// tris[k].geomID = emsurf; \\\
      }

      
    } // end tris loop

      /// rtcSetIntersectionFilterFunction(scene, tri_geom, (RTCFilterFunc)&intersectionFilter); \\\
      /// rtcSetBoundsFunction(scene, tri_geom, (RTCBoundsFunc)&DblTriBounds);   \\\
      /// rtcSetIntersectFunction(scene, tri_geom, (RTCIntersectFunc)&DblTriIntersectFunc); \\\
      /// rtcSetOccludedFunction(scene, tri_geom, (RTCOccludedFunc)&DblTriOccludedFunc); \\\
    
    } // end surface loop

    /// rtcCommitScene(this_scene); \\\

  } // end volume loop
}
void RayTracingInterface::shutdown() {
  ///  for(auto s : scenes) { \\\
    /// rtcDeleteScene(scene); \\\
  ///  } \\\

  for(int i = 0; i < tri_buffers.size(); i++) {
    free(tri_buffers[i]);
  }
  

  if(MBI) { delete MBI; }
  
  /// rtcExit(); \\                             \

}
