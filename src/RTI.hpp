#pragma once

#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"

#include "moab/Core.hpp"

#include "primitives.hpp"
#include "ray.h"

class RayTracingInterface{

  private:
  RTCScene g_scene;

  std::map<moab::EntityHandle,RTCScene> dag_vol_map;
  std::vector<DblTri*> tri_buffers;
  std::map<moab::EntityHandle,int> global_vertex_map;
  std::vector<RTCScene> scenes;

  moab::EntityHandle sceneOffset;


  public:
  enum rf_type { RF, PIV };
  moab::Interface* MBI;
  void *vertex_buffer_ptr;
  int vertex_buffer_size;
  moab::ErrorCode init(std::string filename);
  
  void set_offset(moab::Range &vols);
  void create_scene(moab::EntityHandle vol);
  void commit_scene(moab::EntityHandle vol);
  void finalise_scene();
  void shutdown();
  rf_type ray_fire_type;
  void create_vertex_map(moab::Interface* MBI);

  void add_triangles(moab::Interface* MBI, moab::EntityHandle vol, moab::Range triangles_eh, int sense);
  void ray_fire(moab::EntityHandle volume, const double origin[3],
                const double dir[3], rf_type filt_func, double tnear,
                int &em_surf, double &dist_to_hit, float norm[3]);

  bool point_in_vol(float coordinate[3], float dir[3]);

  moab::ErrorCode get_vols(moab::Range& vols);
  void fire(moab::EntityHandle vol, RTCDRay &ray);

};
