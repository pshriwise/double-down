#include <sstream>

#include "MOABDirectAccess.h"

#include "moab/Range.hpp"

MBDirectAccess::MBDirectAccess(Interface* mbi) : mbi(mbi) {
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

  moab::Range::iterator tris_it = tris.begin();
  while(tris_it != tris.end()) {

    // set connectivity pointer, element stride and the number of elements
    EntityHandle* conntmp;
    rval = mbi->connect_iterate(tris_it, tris.end(), conntmp, element_stride, num_elements);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get direct access to triangle elements");
    // if (num_elements != (int)tris.size()) {
    //   std::stringstream msg;
    //   msg << "Incorrect number of elements (" << num_elements << ") added to the direct access manager. Expected " << tris.size() << ".";
    //   throw std::out_of_range(msg.str());
    // }

    // set const pointers
    vconn.push_back(conntmp);
    conn = &(*conntmp);

    first_elements_.push_back({*tris_it, num_elements});
    first_element = tris.front();
    tris_it += num_elements;
  }

  moab::Range::iterator verts_it = verts.begin();
  while (verts_it != verts.end()) {
    // set vertex coordinate pointers
    double * xtmp, * ytmp, * ztmp;
    rval = mbi->coords_iterate(verts_it, verts.end(), xtmp, ytmp, ztmp, num_vertices);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get direct access to vertex elements");
    // if (num_vertices != (int)verts.size()) {
    //   std::stringstream msg;
    //   msg << "Incorrect number of vertices (" << num_vertices << ") added to the direct access manager. Expected " << verts.size() << ".";
    //   throw std::out_of_range(msg.str());
    // }

    // set const pointers
    x = &(*xtmp); y = &(*ytmp); z = &(*ztmp);

    tx.push_back(&(*xtmp));
    ty.push_back(&(*ytmp));
    tz.push_back(&(*ztmp));

    verts_it += num_vertices;
  }

}
