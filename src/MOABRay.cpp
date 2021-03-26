

#include "double-down/ray.h"
#include "double-down/primitives.hpp"
#include "double-down/MOABRay.h"

bool in_facets(MBRay ray, moab::EntityHandle tri) {
  if (ray.rh) {
    return ray.rh->in_history(tri);
  } else {
    return false;
  }
}

void backface_cull(MBRayHit &rayhit, void*) {
  MBRay& ray = rayhit.ray;
  MBHit& hit = rayhit.hit;
  if( in_facets(ray, hit.prim_handle) ) {
    hit.geomID = RTC_INVALID_GEOMETRY_ID;
    hit.primID = RTC_INVALID_GEOMETRY_ID;
  }

  if(rayhit.dot_prod() < 0.0) {
    hit.geomID = RTC_INVALID_GEOMETRY_ID;
    hit.primID = RTC_INVALID_GEOMETRY_ID;
  }

  return;
}

void frontface_cull(MBRayHit &rayhit, void*) {

  MBRay& ray = rayhit.ray;
  MBHit& hit = rayhit.hit;

  if( in_facets(ray, hit.prim_handle) ) {
    hit.geomID = RTC_INVALID_GEOMETRY_ID;
    hit.primID = RTC_INVALID_GEOMETRY_ID;
  }

  if(rayhit.dot_prod() >= 0.0) {
    hit.geomID = RTC_INVALID_GEOMETRY_ID;
    hit.primID = RTC_INVALID_GEOMETRY_ID;
  }

  return;
}

void count_hits(MBRayHitAccumulate* rayhit) {

  MBRayAccumulate& ray = rayhit->ray;
  MBHit& hit = rayhit->hit;

  if (rayhit->dot_prod() > 0.0) {
    ray.sum -= 1; // leaving
  }
  else {
    ray.sum += 1; // entering
  }

  hit.geomID = RTC_INVALID_GEOMETRY_ID;
  hit.primID = RTC_INVALID_GEOMETRY_ID;
  ray.num_hit++;

  return;
}

void MBDblTriIntersectFunc(RTCIntersectFunctionNArguments* args) {

  void* tris_i = args->geometryUserPtr;
  size_t item = args->primID;
  MBRayHit* rayhit = (MBRayHit*)args->rayhit;
  MBRay& ray = rayhit->ray;
  MBHit& hit = rayhit->hit;

  MBRay orig_ray = ray;
  MBHit orig_hit = hit;
  DblTriIntersectFunc(args);

  if (hit.geomID == RTC_INVALID_GEOMETRY_ID) {
    ray = orig_ray;
    hit = orig_hit;
    return;
  }

  const DblTri* tris = (const DblTri*) tris_i;
  const DblTri& this_tri = tris[item];

  hit.prim_handle = this_tri.handle;
  hit.surf_handle = this_tri.surf;

  if (ray.rf_type == RayFireType::RF || ray.rf_type == RayFireType::PIV) {

    if (ray.rf_type == RayFireType::RF) {
      if (ray.orientation == 1) {
        backface_cull(*rayhit);
      } else if (ray.orientation == -1) {
        frontface_cull(*rayhit);
      }
    }
  } else if (ray.rf_type == RayFireType::ACCUM) {
    count_hits((MBRayHitAccumulate*)&ray);
  }

 if (hit.geomID == RTC_INVALID_GEOMETRY_ID) {
   ray = orig_ray;
   hit = orig_hit;
 }

return;
}
