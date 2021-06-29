
#ifndef DD_RAY_FUNCS_H
#define DD_RAY_FUNCS_H

// Embree
#include "embree3/rtcore.h"

// MOAB
#include "moab/GeomQueryTool.hpp"

// Double-down
#include "ray.h"

/*! Extension of the single/double precision ray hit to include MOAB handles */
struct MBHit : RTCDHit {
  inline MBHit() {
    geomID = RTC_INVALID_GEOMETRY_ID;
    primID = RTC_INVALID_GEOMETRY_ID;
    surf_handle = 0;
    prim_handle = 0;
  }

  // Member variables
  moab::EntityHandle surf_handle; //!< Handle of the MOAB surface hit
  moab::EntityHandle prim_handle; //!< Handle of the MOAB triangle hit
};

/*! Extension of the single/couble precision ray to include an orientation and RayHistory */
struct MBRay : RTCDRay {
  inline MBRay() {
    tnear = 0.0;
    tfar  = 1.E37;
    mask  = -1;
    orientation = 0;
    rh = NULL;
  }

  // Member variables
  int orientation; //!< Ray direction vs. triangle normal orientation to count as a hit (1 for forward, -1 for reverse)
  const moab::GeomQueryTool::RayHistory* rh; //!< RayHistory containing triangles to ignore
};

/*! Struct combining the MBRay and MBHit into a struct that can be passed to Embree's rtcIntersect */
struct MBRayHit {
  struct MBRay ray;
  struct MBHit hit;

  //! \brief Compute the dot product of the ray direction and triangle normal for the current hit
  double dot_prod() {
    return dot(ray.ddir, hit.dNg);
  }

};

/*! Extension of the MOAB ray to accumulate all triangle hits for robust point containment checks */
struct MBRayAccumulate : MBRay {
  inline MBRayAccumulate() {
    tnear = 0.0;
    tfar  = 1.E37;
    mask  = -1;
    sum = 0;
    num_hit = 0;
  }

  int sum; //!< Current sum of the entering/exiting intersection results (+1 for entering, -1 for exiting)
  int num_hit; //!< Number of triangles intersected
};

/*! Struct combining MBRayAccumulate and MBHit that can be passed to Embree's rtcIntersect */
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
