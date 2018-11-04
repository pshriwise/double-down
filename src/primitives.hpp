
#ifndef DD_PRIMITIVES_H
#define DD_PRIMITIVES_H

#include "embree2/rtcore.h"
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "ray.h"
#include "TriangleIntersectors.h"

struct DblTri {
  void* moab_instance;
  moab::EntityHandle handle;
  unsigned int geomID;
  uint8_t sense;
};

void DblTriBounds(void* tris_i, size_t item, RTCBounds& bounds_o);

void DblTriIntersectFunc(void* tris_i, RTCDRay& ray, size_t item);

void DblTriOccludedFunc(void* tris_i, RTCDRay& ray, size_t item);

#endif
