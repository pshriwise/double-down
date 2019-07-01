

#include <iostream>

#include "RTI.hpp"
#include "test_utils.hpp"


int main() {

  // create new MOAB instance
  moab::Interface* MBI = new moab::Core();

  moab::ErrorCode rval;

  rval = MBI->load_file("../test_files/sphere.h5m");
  MB_CHK_SET_ERR(rval, "Failed to load test file");

  RayTracingInterface* RTI = new RayTracingInterface(MBI);

  rval = RTI->init();
  MB_CHK_SET_ERR(rval, "Failed to initialize the RTI.");


  return 0;
}
