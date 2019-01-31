
#ifndef DD_RTI_H
#define DD_RTI_H

#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"

#include "moab/Core.hpp"
#include "moab/GeomQueryTool.hpp"

#include "primitives.hpp"
#include "MOABRay.h"

class RayTracingInterface {

  class DblTriStorage {
  private:
    std::map<moab::EntityHandle, std::pair<int, DblTri*>> storage_;

  public:
    bool is_storing(moab::EntityHandle vol) {
      return storage_.count(vol);
    }

    bool is_storing(DblTri* ptr) {
      for (auto it : storage_) {
        if (it.second.second == ptr) return true;
      }
      return false;
    }

    void store(moab::EntityHandle vol, int size, DblTri* ptr) {
      if (!is_storing(ptr)) {
        storage_[vol] = {size, ptr};
      }
    }

    std::pair<int, DblTri*> retrieve_buffer(moab::EntityHandle vol) {
      return storage_[vol];
    }

    void clear() {
      for (auto it : storage_) {
        free(it.second.second);
      }
      storage_.clear();
    }

  };

  public:
  RayTracingInterface(moab::Interface* mbi) : MBI(mbi) { }

  RayTracingInterface() : MBI(NULL) { }

  ~RayTracingInterface() { buffer_storage.clear(); }

  // Public Functions
  moab::ErrorCode init(std::string filename = "");
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
  void fire(moab::EntityHandle vol, RTCDRay &ray);

  // Member variables
  private:
  moab::Interface* MBI;
  DblTriStorage buffer_storage;
  std::map<moab::EntityHandle, RTCScene> scene_map;
  std::map<moab::EntityHandle, std::vector<DblTri*>> tri_ref_storage;
  std::vector<RTCScene> scenes;
  moab::EntityHandle sceneOffset;

};

#endif
