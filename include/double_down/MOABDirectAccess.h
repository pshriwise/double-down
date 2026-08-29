#ifndef _MBDIRECTACCESS_
#define _MBDIRECTACCESS_

#include <array>
#include <memory>

// MOAB
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

// Double-down
#include "Vec3da.h"

using namespace moab;
using namespace double_down;

/*! Class to manage direct access of triangle connectivity and coordinates */
class MBDirectAccess {

public:
  // constructor
  MBDirectAccess(Interface* mbi);

  //! \brief Initialize internal structures
  void setup();

  //! \brief Reset internal data structures, but maintain MOAB isntance
  void clear();

  //! \brief Update internal data structures to account for changes in the MOAB instance
  void update();

  //! \brief Check that a triangle is part of the managed coordinates here
  inline bool accessible(EntityHandle tri) {
    // determine the correct index to use
    size_t idx = 0;
    auto fe = first_elements_[idx];
    while(true) {
      if (tri - fe.first < fe.second) { break; }
      idx++;
      if (idx >= first_elements_.size()) { return false; }
      fe = first_elements_[idx];
    }
    return true;
  }

  //! \brief Map a vertex handle to (sequence index, local offset) in the coordinate arrays
  inline std::pair<size_t, size_t> vertex_index(EntityHandle vert) const {
    size_t idx = 0;
    auto fv = first_vertices_[idx];
    while (vert - fv.first >= fv.second) {
      idx++;
      if (idx >= first_vertices_.size())
        throw std::runtime_error("Vertex handle not found in MBDirectAccess data");
      fv = first_vertices_[idx];
    }
    return {idx, vert - fv.first};
  }

  //! \brief Get the coordinates of a triangle as MOAB CartVect's
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
    const auto [s0, o0] = vertex_index(vconn_[idx][conn_idx]);
    const auto [s1, o1] = vertex_index(vconn_[idx][conn_idx + 1]);
    const auto [s2, o2] = vertex_index(vconn_[idx][conn_idx + 2]);

    moab::CartVect v0(tx_[s0][o0], ty_[s0][o0], tz_[s0][o0]);
    moab::CartVect v1(tx_[s1][o1], ty_[s1][o1], tz_[s1][o1]);
    moab::CartVect v2(tx_[s2][o2], ty_[s2][o2], tz_[s2][o2]);

    return {v0, v1, v2};
  }

  //! \brief Get the coordinates of a triangle as Vec3da's
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
    const auto [s0, o0] = vertex_index(vconn_[idx][conn_idx]);
    const auto [s1, o1] = vertex_index(vconn_[idx][conn_idx + 1]);
    const auto [s2, o2] = vertex_index(vconn_[idx][conn_idx + 2]);

    Vec3da v0(tx_[s0][o0], ty_[s0][o0], tz_[s0][o0]);
    Vec3da v1(tx_[s1][o1], ty_[s1][o1], tz_[s1][o1]);
    Vec3da v2(tx_[s2][o2], ty_[s2][o2], tz_[s2][o2]);

    return {v0, v1, v2};
  }

  // Accessors
  //! \brief return the number of elements being managed
  inline int n_elements() { return num_elements_; }
  //! \brief return the number of vertices being managed
  inline int n_vertices() { return num_vertices_; }
  //! \brief return the stride between elements in the coordinate arrays
  inline int stride() { return element_stride_;}

private:
  Interface* mbi {nullptr}; //!< MOAB instance for the managed data
  int num_elements_ {-1}; //!< Number of elements in the manager
  int num_vertices_ {-1}; //!< Number of vertices in the manager
  int element_stride_ {-1}; //!< Number of vertices used by each element
  std::vector<std::pair<EntityHandle, size_t>> first_elements_; //!< Pairs of first element and length pairs for contiguous blocks of memory
  std::vector<std::pair<EntityHandle, size_t>> first_vertices_;
  std::vector<const EntityHandle*> vconn_; //!< Storage array(s) for the connectivity array
  std::vector<double*> tx_; //!< Storage array(s) for vertex x coordinates
  std::vector<double*> ty_; //!< Storage array(s) for vertex y coordinates
  std::vector<double*> tz_; //!< Storage array(s) for vertex z coordinates
};

#endif // include guard
