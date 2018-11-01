#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"

void rtc::init()
{
  rtcInit(NULL);
}


void rtc::commit_scene(moab::EntityHandle vol)
{
  /* commit the scene */
  rtcCommit(scenes[vol-sceneOffset]);
}

// TO-DO: allow passing of Embree flags here
void rtc::create_scene(moab::EntityHandle vol) 
{
  /* create scene */
  scenes[vol-sceneOffset] = rtcNewScene(RTC_SCENE_ROBUST,RTC_INTERSECT1);
}

void rtc::shutdown()
{
  /* delete the scene */
  rtcDeleteScene(g_scene); // deleting only one scene???

  /* done with ray tracing */
  rtcExit();
}

// TO-DO: create this function, but for an arbitrary 2D MOAB element
DblTri* rtc::add_Dtriangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense) { 

  size_t num_tris = triangles_eh.size();
  RTCScene scene = scenes[vol-sceneOffset];
  
  unsigned int tri_geom = rtcNewUserGeometry(scene, num_tris);

  DblTri* tris = (DblTri*) malloc(num_tris*sizeof(DblTri));
  rtcSetUserData(scene,tri_geom,tris);
  for(size_t i = 0; i < num_tris; i++) {
    tris[i].moab_instance = (void*)MBI;
    tris[i].handle = triangles_eh[i];
    tris[i].geomID = tri_geom;
    tris[i].sense = sense;
  }
  
  rtcSetIntersectionFilterFunction(scene, tri_geom, (RTCFilterFunc)&intersectionFilter);
  rtcSetBoundsFunction(scene, tri_geom, (RTCBoundsFunc)&DblTriBounds);  
  rtcSetIntersectFunction(scene, tri_geom, (RTCIntersectFunc)&DblTriIntersectFunc);
  rtcSetOccludedFunction(scene, tri_geom, (RTCOccludedFunc)&DblTriOccludedFunc);

  return tris;
}
