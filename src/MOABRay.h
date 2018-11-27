
#ifndef DD_RAY_FUNCS_H
#define DD_RAY_FUNCS_H

// MOAB
#include "moab/GeomQueryTool.hpp"

// local
#include "ray.h"

struct MBRay : RTCDRay {
  inline MBRay() {
    tnear = 0.0;
    tfar  = 1.E37;
    mask  = -1;
    geomID = RTC_INVALID_GEOMETRY_ID;
    primID = RTC_INVALID_GEOMETRY_ID;
    surf_handle = 0;
    prim_handle = 0;
    rh = NULL;
    orientation = 0;
  }

  moab::EntityHandle surf_handle, prim_handle;
  
  moab::GeomQueryTool::RayHistory* rh;

  int orientation;
};

struct MBRayAccumulate : MBRay {
  inline MBRayAccumulate() {
    tnear = 0.0;
    tfar  = 1.E37;
    mask  = -1;
    geomID = RTC_INVALID_GEOMETRY_ID;
    primID = RTC_INVALID_GEOMETRY_ID;
    sum = 0;
    num_hit = 0;
  }

  int sum, num_hit;
};

double dot_prod(RTCDRay ray);

bool in_facets(MBRay ray, moab::EntityHandle tri);

void backface_cull(MBRay &ray, void* = NULL);

void frontface_cull(MBRay &ray, void* = NULL);

void count_hits(MBRayAccumulate* ray);

void MBDblTriIntersectFunc(void* tris_i, MBRay& ray, size_t item);

#endif
