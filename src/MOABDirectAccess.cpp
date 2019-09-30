
#include "MOABDirectAccess.h"

#include "moab/Range.hpp"

MBDirectAccess::MBDirectAccess(std::shared_ptr<Interface> mbi) : mbi(mbi) {
  ErrorCode rval;

  Range tris;
  rval = mbi->get_entities_by_dimension(0, 2, tris, true);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get all elements of dimension 2 (tris)");

  // only supporting triangle elements for now
  if (!tris.all_of_type(MBTRI)) { throw std::runtime_error("Not all 2D elements are triangles"); }

  Range verts;
  rval = mbi->get_entities_by_dimension(0, 0, verts, true);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get all elements of dimension 0 (vertices)");

  if (tris.psize() != 1 || verts.psize() != 1) {
    throw std::runtime_error("EntityHandle space is not contiguous");
  }

  // set connectivity pointer, element stride and the number of elements
  EntityHandle* conntmp;
  rval = mbi->connect_iterate(tris.begin(), tris.end(), conntmp, element_stride, num_elements);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get direct access to triangle elements");
  // set const pointers
  conn = &(*conntmp);

  // set vertex coordinate pointers
  double * xtmp, * ytmp, * ztmp;
  rval = mbi->coords_iterate(verts.begin(), verts.end(), xtmp, ytmp, ztmp, num_vertices);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get direct access to vertex elements");
  // set const pointers
  x = &(*xtmp); y = &(*ytmp); z = &(*ztmp);

  first_element = tris.front();

}
