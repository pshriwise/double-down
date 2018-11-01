#include <string>
#include <iostream>
#include <cassert>

#include "moab/ProgOptions.hpp"
#include "RTI.hpp"
#include "ray.h"

int main(int argc, char** argv) {

  ProgOptions po("Sample program for using the double-down interface to Embree.");

  std::string filename;
  po.addRequiredArg<std::string>("filename", "MOAB surface mesh to fire rays on.", &filename);

  po.parseCommandLine(argc, argv);
  
  RayTracingInterface* RTI = new RayTracingInterface();

  moab::ErrorCode rval;
  rval = RTI->init(filename);
  MB_CHK_SET_ERR(rval, "Failed to initialize the RayTracingInterface.");

  moab::Range vols;
  rval = RTI->get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get volumes from the RTI.");
  
  for(int i = 0; i < 10000000 ; i++) {
    // setup ray
    RTCDRay ray;
    double org[3] = {0.0, 0.0, 0.0};
    ray.set_org(org);
    moab::CartVect dir = {1.0, 1.0, 1.0};
    dir.normalize(); // unit len distance always
    ray.set_dir(dir.array());
    double len = 1e17;
    ray.set_len(len);
  
    // fire ray
    RTI->fire(vols.front(), ray);
  }
  
  delete RTI;
  return 0;
}
