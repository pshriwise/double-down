#include <string>
#include <iostream>

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
  
  RTCDRay ray;

  double org[3] = {0.0, 0.0, 0.0};
  ray.set_org(org);
  double dir[3] = {0.5, 0.5, 0.5};
  ray.set_dir(dir);
  double len = 1e17;
  ray.set_len(len);
  
  moab::Range vols;
  rval = RTI->get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get volumes from the RTI.");

  RTI->fire(vols.front(), ray);

  std::cout << "Ray Distance: " << ray.dtfar << std::endl;
  
  delete RTI;
  return 0;
}


