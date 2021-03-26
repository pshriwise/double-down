#include <iostream>

#include "double-down/RTI.hpp"
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

  // fire a test ray
  double org[3] = {0.0, 0.0, 0.0};
  double dir[3] = {1.0, 0.0, 0.0};

  double dist = 0.0;
  moab::EntityHandle surf;

  RTI->ray_fire(sphere_vol, org, dir, surf, dist);

  if (dist == 0.0) { return 1; }
  if (surf == 0) { return 1; }

  // tear down the BVH for this volume
  RTI->deleteBVH(sphere_vol);

  // rebuild the BVH for this volume
  RTI->createBVH(sphere_vol);

  // ray fire again to make sure this works
  surf = 0;
  RTI->ray_fire(sphere_vol, org, dir, surf, dist);

  if (dist == 0.0) { return 1; }
  if (surf == 0) { return 1; }

  // clear the MDAM
  auto MDAM = RTI->direct_access_manager();
  MDAM->clear();

  if (MDAM->n_elements() > 0) { return 1; }

  MDAM->setup();

  if (MDAM->n_elements() <= 0) { return 1; }

  // ray fire again to make sure this works
  surf = 0;
  RTI->ray_fire(sphere_vol, org, dir, surf, dist);

  if (dist == 0.0) { return 1; }
  if (surf == 0) { return 1; }

  MDAM->update();

  // ray fire again to make sure this works
  surf = 0;
  RTI->ray_fire(sphere_vol, org, dir, surf, dist);

  if (dist == 0.0) { return 1; }
  if (surf == 0) { return 1; }

  return 0;
}
