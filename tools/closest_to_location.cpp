

#include <iostream>

// MOAB
#include "moab/ProgOptions.hpp"

// double-down
#include "double-down/RTI.hpp"


int main(int argc, char** argv) {

  ProgOptions po("Program to find closest surface to a location");

  po.addRequiredArg<std::string>("filename", "MOAB surface mesh");
  po.addRequiredArg<int>("volume_id", "Volume ID used to query the nearest location");
  // these have to be strings b/c ProgOptions won't parse a negative value correctly
  po.addOpt<double>(",x", "X coordinate to query");
  po.addOpt<double>(",y", "Y coordinate to query");
  po.addOpt<double>(",z", "Z coordinate to query");

  po.parseCommandLine(argc, argv);

  // create new MOAB instance
  moab::Interface* MBI = new moab::Core();

  moab::ErrorCode rval;

  rval = MBI->load_file(po.getReqArg<std::string>("filename").c_str());
  MB_CHK_SET_ERR(rval, "Failed to load test file");

  std::unique_ptr<RayTracingInterface> RTI(new RayTracingInterface(MBI));

  rval = RTI->init();
  MB_CHK_SET_ERR(rval, "Failed to initialize the RTI.");


  moab::EntityHandle vol = RTI->gttool()->entity_by_id(3, po.getReqArg<int>("volume_id"));

  // fire a test ray
  double org[3] = {0.0, 0.0, 0.0};

  // get values from command line if present
  po.getOpt<double>(",x", org);
  po.getOpt<double>(",y", org + 1);
  po.getOpt<double>(",z", org + 2);

  EntityHandle surf, facet;
  double dist = 0.0;
  RTI->closest(vol, org, dist, &surf, &facet);

  if (surf == 0) {
    std::cout << "No facet found!" << std::endl;
    return 1;
  }

  int surf_id = RTI->gttool()->global_id(surf);

  std::cout << "========================================================\n";
  std::cout << "Query location: " << org[0] << ", " << org[1] << ", " << org[2] << std::endl;
  std::cout << "Distance to nearest triangle: " << dist << std::endl;
  std::cout << "Surface handle: " << surf << std::endl;
  std::cout << "Surface ID: " << surf_id << std::endl;
  std::cout << "Triangle handle: " << facet << std::endl;
  std::cout << "========================================================\n";

  return 0;
}
