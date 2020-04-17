#ifndef _MBDIRECTACCESS_
#define _MBDIRECTACCESS_

#include <memory>

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "Vec3da.h"


using namespace moab;

class MBDirectAccess {

  // constructor
public:
  MBDirectAccess(Interface* mbi);

  inline std::array<moab::CartVect, 3> get_mb_coords(const EntityHandle& tri) {

    // determine the correct index to use
    int idx = 0;
    auto fe = first_elements_[idx];
    while(true) {
      if (tri - fe.first < fe.second) { break; }
      idx++;
      fe = first_elements_[idx];
    }

    size_t conn_idx = element_stride * (tri - fe.first);
    size_t i0 = vconn[idx][conn_idx] - 1;
    size_t i1 = vconn[idx][conn_idx + 1] - 1;
    size_t i2 = vconn[idx][conn_idx + 2] - 1;


    moab::CartVect v0(tx[idx][i0], ty[idx][i0], tz[idx][i0]);
    moab::CartVect v1(tx[idx][i1], ty[idx][i1], tz[idx][i1]);
    moab::CartVect v2(tx[idx][i2], ty[idx][i2], tz[idx][i2]);

    return {v0, v1, v2};
  }

  inline std::array<Vec3da, 3> get_coords(const EntityHandle& tri) {

    // determine the correct index to use
    int idx = 0;
    auto fe = first_elements_[idx];
    while(true) {
      if (tri - fe.first < fe.second) { break; }
      idx++;
      fe = first_elements_[idx];
    }

    size_t conn_idx = element_stride * (tri - fe.first);
    size_t i0 = vconn[idx][conn_idx] - 1;
    size_t i1 = vconn[idx][conn_idx + 1] - 1;
    size_t i2 = vconn[idx][conn_idx + 2] - 1;


    Vec3da v0(tx[idx][i0], ty[idx][i0], tz[idx][i0]);
    Vec3da v1(tx[idx][i1], ty[idx][i1], tz[idx][i1]);
    Vec3da v2(tx[idx][i2], ty[idx][i2], tz[idx][i2]);

    return {v0, v1, v2};
  }

  /* OLD get_coords IMPLEMENTATION - ASSUMES ONE CONTIGUOUS BLOCK OF MEMORY */

  // std::array<Vec3da, 3> get_coords(const EntityHandle& tri) {

  //   size_t conn_idx = element_stride * (tri - first_element);
  //   size_t i0 = conn[conn_idx] - 1;
  //   size_t i1 = conn[conn_idx + 1] - 1;
  //   size_t i2 = conn[conn_idx + 2] - 1;


  //   Vec3da v0(x[i0], y[i0], z[i0]);
  //   Vec3da v1(x[i1], y[i1], z[i1]);
  //   Vec3da v2(x[i2], y[i2], z[i2]);

  //   return {v0, v1, v2};
  // }

private:

  Interface* mbi;

  int num_elements {-1};
  int num_vertices {-1};

  int element_stride {-1};

  std::vector<std::pair<EntityHandle, size_t>> first_elements_;

  std::vector<const EntityHandle*> vconn;

  std::vector<double*> tx, ty, tz;

};

#endif // include guard
