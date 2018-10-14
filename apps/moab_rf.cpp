


#include "moab/ProgOptions.hpp"
#include "RTI.hpp"


#include <string>
#include <iostream>

int main(int argc, char** argv) {

  ProgOptions po("Sample program for using the double-down interface to Embree.");

  std::string filename;
  po.addRequiredArg<std::string>("filename", "MOAB surface mesh to fire rays on.", &filename);

  po.parseCommandLine(argc, argv);
  
  RayTracingInterface* RTI = new RayTracingInterface();

  moab::ErrorCode rval;
  rval = RTI->init(filename);
  MB_CHK_SET_ERR(rval, "Failed to initialize the RayTracingInterface.");
  
  // setup Embree instance

  return 0;
}


