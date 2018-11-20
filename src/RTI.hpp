
#ifndef DD_RTI_H
#define DD_RTI_H

#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"

#include "moab/Core.hpp"

#include "primitives.hpp"

class RayTracingInterface{

  public:
  // Public Functions
  moab::ErrorCode init(std::string filename);
  void set_offset(moab::Range &vols);
  void create_scene(moab::EntityHandle vol);
  void commit_scene(moab::EntityHandle vol);
  void finalise_scene();
  void shutdown();
  void create_vertex_map(moab::Interface* MBI);

  void add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense);
  void ray_fire(moab::EntityHandle volume, const double origin[3],
                const double dir[3], RayFireType filt_func, double tnear,
                int &em_surf, double &dist_to_hit, float norm[3]);

  void dag_ray_fire(const moab::EntityHandle volume,
                    const double point[3],
                    const double dir[3],
                    moab::EntityHandle& next_surf,
                    double& next_surf_dist,
                    void* history,
                    double user_dist_limit,
                    int ray_orientation);
  
  bool point_in_vol(float coordinate[3], float dir[3]);

  moab::ErrorCode get_vols(moab::Range& vols);
  void fire(moab::EntityHandle vol, RTCDRay &ray);

  // Member variables
  private:
  moab::Interface* MBI;
  std::vector<DblTri*> tri_buffers;
  std::vector<RTCScene> scenes;
  moab::EntityHandle sceneOffset;
  
};

#endif
