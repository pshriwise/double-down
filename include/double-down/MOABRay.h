
#ifndef DD_RAY_FUNCS_H
#define DD_RAY_FUNCS_H

// Embree
#include "embree3/rtcore.h"

// MOAB
#include "moab/GeomQueryTool.hpp"

// Double-down
#include "ray.h"

struct MBHit : RTCDHit {
  inline MBHit() {
    geomID = RTC_INVALID_GEOMETRY_ID;
    primID = RTC_INVALID_GEOMETRY_ID;
    surf_handle = 0;
    prim_handle = 0;
  }

  // MOAB/DAGMC-specific information
  moab::EntityHandle surf_handle, prim_handle;
};

struct MBRay : RTCDRay {
  inline MBRay() {
    tnear = 0.0;
    tfar  = 1.E37;
    mask  = -1;
    orientation = 0;
    rh = NULL;
  }
  int orientation;
  const moab::GeomQueryTool::RayHistory* rh;
};

struct MBRayHit {
  struct MBRay ray;
  struct MBHit hit;

  double dot_prod() {
    return dot(ray.ddir, hit.dNg);
  }

};

struct MBRayAccumulate : MBRay {
  inline MBRayAccumulate() {
    tnear = 0.0;
    tfar  = 1.E37;
    mask  = -1;
    sum = 0;
    num_hit = 0;
  }

  int sum, num_hit;
};

struct MBRayHitAccumulate {
  struct MBRayAccumulate ray;
  struct MBHit hit;

  double dot_prod() {
    return dot(ray.ddir, hit.dNg);
  }
};


double dot_prod(RTCDRay ray);

bool in_facets(MBRay ray, moab::EntityHandle tri);

void backface_cull(MBRayHit &rayhit, void* = NULL);

void frontface_cull(MBRayHit &rayhit, void* = NULL);

void count_hits(MBRayAccumulate* ray);

void MBDblTriIntersectFunc(RTCIntersectFunctionNArguments* args);

#endif
