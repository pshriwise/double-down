

#include "moab/Core.hpp"
#include "moab/ProgOptions.hpp"

#include <string>
#include <iostream>

int main(int argc, char** argv) {

  ProgOptions po("Sample program for using the double-down interface to Embree.");

  std::string filename;
  po.addRequiredArg<std::string>("filename", "MOAB surface mesh to fire rays on.", &filename);

  po.parseCommandLine(argc, argv);
  
  // create a new MOAB instance
  moab::Interface* mbi = new moab::Core();

  moab::ErrorCode rval;

  rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load the specified MOAB file.");

  
  
  return 0;
}
