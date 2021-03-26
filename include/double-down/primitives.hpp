
#ifndef DD_PRIMITIVES_H
#define DD_PRIMITIVES_H

#include "embree3/rtcore.h"
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "ray.h"
#include "MOABDirectAccess.h"

struct DblTri {
  void* mdam;
  moab::EntityHandle handle;
  unsigned int geomID;
  moab::EntityHandle surf;
  int sense;
};

inline RTCBounds DblTriBounds(MBDirectAccess* mdam, moab::EntityHandle tri_handle) {

  std::array<Vec3da, 3> coords = mdam->get_coords(tri_handle);

  double bump_val = 5e-03;

  RTCBounds bounds_o;
  bounds_o.lower_x = std::min(coords[0][0],std::min(coords[1][0],coords[2][0]));
  bounds_o.lower_y = std::min(coords[0][1],std::min(coords[1][1],coords[2][1]));
  bounds_o.lower_z = std::min(coords[0][2],std::min(coords[1][2],coords[2][2]));

  bounds_o.upper_x = std::max(coords[0][0],std::max(coords[1][0],coords[2][0]));
  bounds_o.upper_y = std::max(coords[0][1],std::max(coords[1][1],coords[2][1]));
  bounds_o.upper_z = std::max(coords[0][2],std::max(coords[1][2],coords[2][2]));

  bounds_o.lower_x -= bump_val; bounds_o.lower_y -= bump_val; bounds_o.lower_z -= bump_val;
  bounds_o.upper_x += bump_val; bounds_o.upper_y += bump_val; bounds_o.upper_z += bump_val;

  return bounds_o;

}


void DblTriBounds(const RTCBoundsFunctionArguments* args);

void DblTriIntersectFunc(RTCIntersectFunctionNArguments* args);

void DblTriOccludedFunc(RTCOccludedFunctionNArguments* args);

bool DblTriPointQueryFunc(RTCPointQueryFunctionArguments* args);

double DblTriClosestFunc(const DblTri& tri, const double loc[3]);

#endif
