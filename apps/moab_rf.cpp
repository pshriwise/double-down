


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
  // setup Embree instance
  
  
  return 0;
}


