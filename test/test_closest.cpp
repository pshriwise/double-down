

#include <iostream>

#include "test_utils.hpp"

#include "double-down/RTI.hpp"


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

  std::cout << "Running closest test" << std::endl;
  RTI->closest(sphere_vol, org, dist);

  std::cout << dist << std::endl;
  if (dist > 10.0) { return 1; }

  return 0;
}
