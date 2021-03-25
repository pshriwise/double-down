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

  //! \brief Initialize internal structures
  void setup();

  //! \brief Reset internal data structures, but maintain MOAB isntance
  void clear();

  //! \brief Update internal data structures to account for changes in the MOAB instance
  void update();

  inline bool accessible(EntityHandle tri) {
    // determine the correct index to use
    int idx = 0;
    auto fe = first_elements_[idx];
    while(true) {
      if (tri - fe.first < fe.second) { break; }
      idx++;
      if (idx >= first_elements_.size()) { return false; }
      fe = first_elements_[idx];
    }
    return true;
  }

  inline std::array<moab::CartVect, 3> get_mb_coords(const EntityHandle& tri) {

    // determine the correct index to use
    int idx = 0;
    auto fe = first_elements_[idx];
    while(true) {
      if (tri - fe.first < fe.second) { break; }
      idx++;
      fe = first_elements_[idx];
    }

    size_t conn_idx = element_stride_ * (tri - fe.first);
    size_t i0 = vconn_[idx][conn_idx] - 1;
    size_t i1 = vconn_[idx][conn_idx + 1] - 1;
    size_t i2 = vconn_[idx][conn_idx + 2] - 1;

    moab::CartVect v0(tx_[idx][i0], ty_[idx][i0], tz_[idx][i0]);
    moab::CartVect v1(tx_[idx][i1], ty_[idx][i1], tz_[idx][i1]);
    moab::CartVect v2(tx_[idx][i2], ty_[idx][i2], tz_[idx][i2]);

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

    size_t conn_idx = element_stride_ * (tri - fe.first);
    size_t i0 = vconn_[idx][conn_idx] - 1;
    size_t i1 = vconn_[idx][conn_idx + 1] - 1;
    size_t i2 = vconn_[idx][conn_idx + 2] - 1;

    Vec3da v0(tx_[idx][i0], ty_[idx][i0], tz_[idx][i0]);
    Vec3da v1(tx_[idx][i1], ty_[idx][i1], tz_[idx][i1]);
    Vec3da v2(tx_[idx][i2], ty_[idx][i2], tz_[idx][i2]);

    return {v0, v1, v2};
  }

  // Accessors
  inline int n_elements() { return num_elements_; }
  inline int n_vertices() { return num_vertices_; }
  inline int stride() { return element_stride_;}

private:
  Interface* mbi {nullptr};

  int num_elements_ {-1};
  int num_vertices_ {-1};

  int element_stride_ {-1};

  std::vector<std::pair<EntityHandle, size_t>> first_elements_;

  std::vector<const EntityHandle*> vconn_;

  std::vector<double*> tx_, ty_, tz_;
};

#endif // include guard
