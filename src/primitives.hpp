
#ifndef DD_PRIMITIVES_H
#define DD_PRIMITIVES_H

#include "embree3/rtcore.h"
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "ray.h"
#include "TriangleIntersectors.h"

struct DblTri {
  void* moab_instance;
  moab::EntityHandle handle;
  unsigned int geomID;
  moab::EntityHandle surf;
  int sense;
};

inline RTCBounds DblTriBounds(const moab::Interface *mbi, moab::EntityHandle tri_handle) {
  moab::ErrorCode rval;

  std::vector<moab::EntityHandle> conn;
  rval = mbi->get_connectivity(&tri_handle, 1, conn);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity");

  if (conn.size() != 3) {
    throw std::length_error("Incorrect number of coordinates returned for a triangle entity.");
  }

  moab::CartVect coords[3];
  rval = mbi->get_coords(&conn[0], 1, coords[0].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[1], 1, coords[1].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");
  rval = mbi->get_coords(&conn[2], 1, coords[2].array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get vertext coordinates");

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
