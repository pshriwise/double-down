
#include "RTI.hpp"
#include "MBTagConventions.hpp"

moab::ErrorCode RayTracingInterface::init(std::string filename) {

  // rtcInit();

  MBI = new moab::Core();
  
  moab::ErrorCode rval;

  rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load the specified MOAB file");

  // retrieve surfaces
  moab::Tag geom_tag;
  rval = MBI->tag_get_handle(moab::GEOMETRY_DIMENSION_TAG_NAME);
  MB_CHK_SET_ERR(rval, "Failed to get the geometry dimension tag");

  moab::Range vols;
  int val = 3;
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_tag, &val, vols);
  MB_CHK_SET_ERR(rval, "Failed to get all surface sets in the model");

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

    for (moab::Range::iterator j surfs.begin();
         j != surfs.end(); j++) {

      moab::EntityHandle this_surf = *j;
      
      // get all triangles on this surface
      moab::Range tris;
      rval = MBI->get_entities_by_type(this_surf, moab::MBTRI, tris);
      MB_CHK_SET_ERR(rval, "Failed to get surface triangles.");

      int num_tris = tris.size();

      // create a new geometry for the volume's scene
      /// unsigned int emsurf = rtcNewUserGeometry(scene, num_tris); \\\

      DblTri* tris = (DblTri*) malloc(num_tris*sizeof(DblTri));

      for (int k = 0; k < num_tris; k++) {
        tris[k].moab_instance = MBI;
        tris[k].handle = tris[k];
        tris[k].geomID = emsurf;
      }

      /// rtcSetIntersectionFilterFunction(scene, tri_geom, (RTCFilterFunc)&intersectionFilter); \\\
      /// rtcSetBoundsFunction(scene, tri_geom, (RTCBoundsFunc)&DblTriBounds);   \\\
      /// rtcSetIntersectFunction(scene, tri_geom, (RTCIntersectFunc)&DblTriIntersectFunc); \\\
      /// rtcSetOccludedFunction(scene, tri_geom, (RTCOccludedFunc)&DblTriOccludedFunc); \\\
      
    } // surface loop
  } // volume loop
  
}


