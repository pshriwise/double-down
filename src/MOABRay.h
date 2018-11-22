
#ifndef DD_RAY_FUNCS_H
#define DD_RAY_FUNCS_H

// MOAB
#include "moab/GeomQueryTool.hpp"

// local
#include "ray.h"

struct MBRayAccumulate : RTCDRay {
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

struct MBRay : RTCDRay {
  inline MBRay() {
    tnear = 0.0;
    tfar  = 1.E37;
    mask  = -1;
    geomID = RTC_INVALID_GEOMETRY_ID;
    primID = RTC_INVALID_GEOMETRY_ID;
  }

  moab::GeomQueryTool::RayHistory rh;
};

double dot_prod(RTCDRay ray);

bool in_facets(MBRay ray, moab::EntityHandle tri);

void backface_cull(MBRay &ray, void*);

void frontface_cull(MBRay &ray, void*);

void count_hits(MBRayAccumulate &ray, void*);

#endif
