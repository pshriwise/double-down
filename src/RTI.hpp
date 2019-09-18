#ifndef DD_RTI_H
#define DD_RTI_H

#include "embree3/rtcore.h"
#include "embree3/rtcore_ray.h"

#include "moab/Core.hpp"
#include "moab/GeomQueryTool.hpp"

#include "primitives.hpp"
#include "MOABRay.h"

#include <unordered_map>
#include <memory>

class Node;

class RayTracingInterface {

  class DblTriStorage {
  private:
    std::unordered_map<moab::EntityHandle, std::pair<int, std::shared_ptr<DblTri>>> storage_;

  public:
    bool is_storing(moab::EntityHandle vol) {
      return storage_.count(vol);
    }

    bool is_storing(std::shared_ptr<DblTri> ptr) {
      for (auto it : storage_) {
        if (it.second.second == ptr) return true;
      }
      return false;
    }

    void store(moab::EntityHandle vol, int size, std::shared_ptr<DblTri> ptr) {
      if (!is_storing(ptr)) {
        storage_[vol] = {size, ptr};
      }
    }

    std::pair<int, std::shared_ptr<DblTri>> retrieve_buffer(moab::EntityHandle vol) {
      return storage_[vol];
    }

    void clear() {
      storage_.clear();
    }

  };

  public:
  RayTracingInterface(moab::Interface* mbi) : MBI(mbi) { }

  RayTracingInterface() : MBI(NULL) { }

  ~RayTracingInterface() { buffer_storage.clear(); }

  // Public Functions
  moab::ErrorCode init(std::string filename = "", bool closest_enabled=true);
  void set_offset(moab::Range &vols);
  void create_scene(moab::EntityHandle vol);
  void commit_scene(moab::EntityHandle vol);
  void finalise_scene();
  void shutdown();
  void create_vertex_map(moab::Interface* MBI);

  void add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense);

  void dag_point_in_volume(const moab::EntityHandle volume,
                           const double xyz[3],
                           int& result,
                           const double *uvw,
                           moab::GeomQueryTool::RayHistory *history,
                           double overlap_tol = 0.0,
                           moab::EntityHandle = 0);


  void boundary_case(moab::EntityHandle volume,
                     int& result,
                     double u,
                     double v,
                     double w,
                     moab::EntityHandle facet,
                     moab::EntityHandle surface);


  void test_volume_boundary(const moab::EntityHandle volume,
                                                 const moab::EntityHandle surface,
                                                 const double xyz[3],
                                                 const double uvw[3],
                                                 int& result,
                            const moab::GeomQueryTool::RayHistory* history = 0);

  void ray_fire(moab::EntityHandle volume, const double origin[3],
                const double dir[3], RayFireType filt_func, double tnear,
                int &em_surf, double &dist_to_hit, float norm[3]);

  void dag_ray_fire(const moab::EntityHandle volume,
                    const double point[3],
                    const double dir[3],
                    moab::EntityHandle& next_surf,
                    double& next_surf_dist,
                    moab::GeomQueryTool::RayHistory* history = NULL,
                    double user_dist_limit = 0,
                    int ray_orientation = 1);

  bool point_in_vol(float coordinate[3], float dir[3]);

  moab::ErrorCode get_vols(moab::Range& vols);
  void fire(moab::EntityHandle vol, RTCDRayHit &rayhit);

  void buildBVH(moab::EntityHandle vol);

  void closest(moab::EntityHandle vol, const double loc[3],
               double &result, moab::EntityHandle* surface = 0, moab::EntityHandle* facet = 0);

  void get_normal(moab::EntityHandle surf, const double loc[3],
                  double angle[3], moab::EntityHandle facet = 0);


  // Member variables
  private:
  bool closest_enabled_{true};
  moab::Interface* MBI;
  std::unique_ptr<moab::GeomTopoTool> GTT;
  DblTriStorage buffer_storage;
  std::unordered_map<moab::EntityHandle, RTCScene> scene_map;
  std::unordered_map<moab::EntityHandle, std::vector<std::shared_ptr<DblTri>>> tri_ref_storage;
  std::vector<RTCScene> scenes;
  moab::EntityHandle sceneOffset;
  std::unordered_map<moab::EntityHandle, Node*> root_map;
  RTCDevice g_device;
};

#endif
