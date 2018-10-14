

#include "moab/Core.hpp"
#include "test_utils.hpp"

void write_test_mesh() {

  moab::Interface* MBI = new moab::Core();

  double c[9] = {0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0};
  moab::Range verts;
  moab::ErrorCode rval;
  rval = MBI->create_vertices(c, 3, verts);
  MB_CHK_ERR_CONT(rval);

  moab::EntityHandle conn[3] = {verts[0], verts[1], verts[2]};
  
  moab::EntityHandle t;
  rval = MBI->create_element(moab::MBTRI, conn, 1, t);
  MB_CHK_ERR_CONT(rval);

  rval = MBI->write_file("test_mesh.h5m");
  MB_CHK_ERR_CONT(rval);

  return;
}
