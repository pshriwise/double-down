#ifndef DD_RTI_H
#define DD_RTI_H

#include "embree3/rtcore.h"
#include "embree3/rtcore_ray.h"

#include "moab/Core.hpp"
#include "moab/GeomQueryTool.hpp"
#include "moab/GeomTopoTool.hpp"

#include "primitives.hpp"
#include "MOABRay.h"
#include "MOABDirectAccess.h"

#include <unordered_map>
#include <memory>

class Node;

class RayTracingInterface {

  class DblTriStorage {
  public:
    bool is_storing(moab::EntityHandle vol)
    {
      return storage_.find(vol) != storage_.end();
    }

    void store(moab::EntityHandle vol, std::vector<DblTri>&& buffer)
    {
      if (storage_.find(vol) != storage_.end()) { return; }
      storage_[vol] = buffer;
    }

    std::vector<DblTri>& retrieve_buffer(moab::EntityHandle vol)
    {
      return storage_.at(vol);
    }

    const std::vector<DblTri>& retrieve_buffer(moab::EntityHandle vol) const
    {
      return storage_.at(vol);
    }

    void free_storage(moab::EntityHandle vol)
    {
      if (is_storing(vol)) storage_.erase(vol);
    }

    void clear()
    {
      storage_.clear();
    }

  private:
    std::unordered_map<moab::EntityHandle, std::vector<DblTri>> storage_;

  };

  public:
  RayTracingInterface(moab::Interface* MBI);

  RayTracingInterface(std::shared_ptr<moab::GeomTopoTool> GTT);

  RayTracingInterface() : MBI(nullptr) { }

  ~RayTracingInterface() { shutdown(); buffer_storage.clear(); }

  // Public Functions
  moab::ErrorCode load_file(std::string filename);
  moab::ErrorCode init(std::string filename = "");
  void set_offset(moab::Range &vols);
  void create_scene(moab::EntityHandle vol);
  void commit_scene(moab::EntityHandle vol);
  void finalise_scene();
  void shutdown();
  void create_vertex_map(moab::Interface* MBI);

  void add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense);

  moab::ErrorCode point_in_volume(const moab::EntityHandle volume,
                                  const double xyz[3],
                                  int& result,
                                  const double *uvw,
                                  const moab::GeomQueryTool::RayHistory *history,
                                  double overlap_tol = 0.0);

  moab::ErrorCode point_in_volume_slow(moab::EntityHandle volume,
                                       const double xyz[3],
                                       int& result);

  moab::ErrorCode poly_solid_angle( moab::EntityHandle face,
                                    const moab::CartVect& point,
                                    double& area );

  void boundary_case(moab::EntityHandle volume,
                     int& result,
                     double u,
                     double v,
                     double w,
                     moab::EntityHandle facet,
                     moab::EntityHandle surface);


  moab::ErrorCode test_volume_boundary(const moab::EntityHandle volume,
                                       const moab::EntityHandle surface,
                                       const double xyz[3],
                                       const double uvw[3],
                                       int& result,
                                       const moab::GeomQueryTool::RayHistory* history = 0);

  moab::ErrorCode ray_fire(const moab::EntityHandle volume,
                           const double point[3],
                           const double dir[3],
                           moab::EntityHandle& next_surf,
                           double& next_surf_dist,
                           moab::GeomQueryTool::RayHistory* history = 0,
                           double user_dist_limit = 0,
                           int ray_orientation = 1,
                           void* dum=NULL);

  bool point_in_vol(float coordinate[3], float dir[3]);

  moab::ErrorCode get_obb(moab::EntityHandle vol,
                          std::array<double, 3>& center,
                          std::array<double, 3>& axis0,
                          std::array<double, 3>& axis1,
                          std::array<double, 3>& axis2);

  moab::ErrorCode get_bbox(moab::EntityHandle vol,
                           double llc[3],
                           double urc[3]);

  moab::ErrorCode get_vols(moab::Range& vols);
  void fire(moab::EntityHandle vol, RTCDRayHit &rayhit);

  /// Allocates space for triangle reference information for the volume
  moab::ErrorCode
  allocateTriangleBuffer(moab::EntityHandle vol);

  moab::ErrorCode
  createBVH(moab::EntityHandle vol);

  void
  deleteBVH(moab::EntityHandle vol);

  moab::ErrorCode
  closest_to_location(moab::EntityHandle volume,
                      const double point[3],
                      double & result,
                      moab::EntityHandle* closest_surf = 0);

  void closest(moab::EntityHandle vol, const double loc[3],
               double &result, moab::EntityHandle* surface = 0, moab::EntityHandle* facet = 0);

  moab::ErrorCode get_normal(moab::EntityHandle surf,
                             const double loc[3],
                             double angle[3],
                             const moab::GeomQueryTool::RayHistory* history = 0);

  moab::ErrorCode
  measure_volume(moab::EntityHandle volume,
                 double& result);

  moab::ErrorCode
  measure_area(moab::EntityHandle surface,
               double& result);

  bool
  has_bvh() const;

  // inline functions
  inline
  moab::GeomTopoTool * gttool() { return GTT.get(); }

  inline
  double get_numerical_precision() { return numerical_precision; }

  inline
  double get_overlap_thickness() { return overlap_thickness; }

  inline
  void set_numerical_precision(double val) { numerical_precision = val; }

  inline
  void set_overlap_thickness(double val) { overlap_thickness = val; }

  inline std::shared_ptr<MBDirectAccess> direct_access_manager() { return mdam; }

  // Member variables
  private:
  std::shared_ptr<moab::GeomTopoTool> GTT;
  moab::Interface* MBI;
  std::shared_ptr<MBDirectAccess> mdam;
  DblTriStorage buffer_storage;
  std::unordered_map<moab::EntityHandle, RTCScene> scene_map;

  std::unordered_map<moab::EntityHandle, Node*> root_map;

  // a couple values we never touch really
  double numerical_precision {1E-3};
  double overlap_thickness {0.0};

  RTCDevice g_device {nullptr};
};

#endif
