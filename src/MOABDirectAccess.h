#ifndef _MBDIRECTACCESS_
#define _MBDIRECTACCESS_

#include <memory>

#include "moab/Core.hpp"

#include "Vec3da.h"


using namespace moab;

class MBDirectAccess {

  // constructor
public:
  MBDirectAccess(Interface* mbi);

  std::array<Vec3da, 3> get_coords(const EntityHandle& tri) {
    size_t conn_idx = element_stride * (tri - first_element);

    size_t i0 = conn[conn_idx] - 1;
    size_t i1 = conn[conn_idx + 1] - 1;
    size_t i2 = conn[conn_idx + 2] - 1;

    Vec3da v0(x[i0], y[i0], z[i0]);
    Vec3da v1(x[i1], y[i1], z[i1]);
    Vec3da v2(x[i2], y[i2], z[i2]);

    return {v0, v1, v2};
  }

private:

  Interface* mbi;

  int num_elements {-1};
  int num_vertices {-1};

  int element_stride {-1};

  EntityHandle first_element{0};

  const EntityHandle *conn;

  const double *x, *y, *z;
};

#endif // include guard
