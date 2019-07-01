

#include <iostream>

#include "RTI.hpp"
#include "test_utils.hpp"


int main() {

  // create new MOAB instance
  moab::Interface* MBI = new moab::Core();

  moab::ErrorCode rval;

  rval = MBI->load_file("../test_files/sphere.h5m");
  MB_CHK_SET_ERR(rval, "Failed to load test file");

  std::unique_ptr<RayTracingInterface> RTI(new RayTracingInterface(MBI));

  rval = RTI->init();
  MB_CHK_SET_ERR(rval, "Failed to initialize the RTI.");

  moab::Range vols;
  rval = RTI->get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get volumes from the RTI.");

  moab::EntityHandle sphere_vol = vols[0];

  double dtfar = 100000.0;

  // MBRay ray;
  // ray.set_org({0.0, 0.0, 0.0});
  // ray.set_dir({1.0, 0.0, 0.0});
  // ray.tnear = 0.0;
  // ray.set_len(dtfar);

  // MBHit hit;

  // MBRayHit ray_hit;
  // ray_hit.ray = ray;
  // ray_hit.hit = hit;

  // RTI->fire(sphere_vol, ray_hit);

  // if (ray_hit.ray.dtfar == dtfar)  { return 1; }
  // if (ray_hit.hit.surf_handle == 0) { return 1; }

  return 0;
}
