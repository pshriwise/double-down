#include <cstdlib>
#include <iostream>
#include <math.h>

#include "test_utils.hpp"

#include "double_down/RTI.hpp"

using namespace double_down;

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
  for (int i = 0; i < 1000; i++) {
    std::cout << org[0] << " " << org[1] << " " << org[2] << std::endl;
    moab::EntityHandle volume = 0;
    RTI->find_volume(org, volume, dir);
    if (volume != sphere_vol) { return 1; }

    // test this without a direction supplied
    volume = 0;
    RTI->find_volume(org, volume);
    if (volume != sphere_vol) { return 1; }

    // sample a random point in the sphere volume
    double mu = -1.0 + 2.0 * (double)std::rand() / (double)RAND_MAX;
    double phi = 2.0 * M_PI * (double)std::rand() / (double)RAND_MAX;
    double r = 9.9 * (double)std::rand() / (double)RAND_MAX;
    org[0] = sqrt(1-mu*mu)*cos(phi)*r;
    org[1] = sqrt(1-mu*mu)*sin(phi)*r;
    org[2] = r*mu;
  }

  return 0;
}
