
#ifndef DD_PRIMITIVES_H
#define DD_PRIMITIVES_H

#include <array>


// MOAB
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

// Double-down
#include "ray.h"
#include "MOABDirectAccess.h"

// Embree
#include "double_down/embree_interface.hpp"
namespace double_down {

/*! Structure with triangle information used in Embree */
struct DblTri {
  void* mdam;
  moab::EntityHandle handle;
  unsigned int geomID;
  moab::EntityHandle surf;
  int sense;
};

/*! Structure for linking primitive data with surfaces */
struct UserData {
    double bump; //!< bounding box bump value for this set of triangles
    DblTri* tri_ptr; //!< Set of triangles in the buffer
};

//! \brief Function returning the extended bounds of a double precision MOAB triangle
inline RTCBounds DblTriBounds(MBDirectAccess* mdam,
                              moab::EntityHandle tri_handle,
                              double bump_val = 5e-03) {

  std::array<Vec3da, 3> coords = mdam->get_coords(tri_handle);

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

//! \brief Function returning the extended bounds of a double precision MOAB triangle
void DblTriBounds(const RTCBoundsFunctionArguments* args);

//! \brief Function for intersecting a ray with a double-precision triangle
void DblTriIntersectFunc(RTCIntersectFunctionNArguments* args);

//! \brief Function for intersecting a ray with a double-precision triangle
void DblTriOccludedFunc(RTCOccludedFunctionNArguments* args);

//! \brief Function for determining the closest point on a triangle to the ray origin
bool DblTriPointQueryFunc(RTCPointQueryFunctionArguments* args);

//! \brief Function returning the distance to the nearest point on a triangle from the provided location
double DblTriClosestFunc(const DblTri& tri, const double loc[3]);

} // end namespace double_down

#endif
