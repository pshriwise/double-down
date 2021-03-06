#include <sstream>

#include "moab/Range.hpp"

#include "MOABDirectAccess.h"

MBDirectAccess::MBDirectAccess(Interface* mbi) : mbi(mbi) {
  ErrorCode rval;

  // setup triangles
  Range tris;
  rval = mbi->get_entities_by_dimension(0, 2, tris, true);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get all elements of dimension 2 (tris)");
  num_elements = tris.size();

  // only supporting triangle elements for now
  if (!tris.all_of_type(MBTRI)) { throw std::runtime_error("Not all 2D elements are triangles"); }

  moab::Range::iterator tris_it = tris.begin();
  while(tris_it != tris.end()) {
    // set connectivity pointer, element stride and the number of elements
    EntityHandle* conntmp;
    int n_elements;
    rval = mbi->connect_iterate(tris_it, tris.end(), conntmp, element_stride, n_elements);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get direct access to triangle elements");

    // set const pointers
    vconn.push_back(conntmp);
    first_elements_.push_back({*tris_it, n_elements});

    // move iterator forward
    tris_it += n_elements;
  }

  // setup vertices
  Range verts;
  rval = mbi->get_entities_by_dimension(0, 0, verts, true);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get all elements of dimension 0 (vertices)");
  num_vertices = verts.size();

  moab::Range::iterator verts_it = verts.begin();
  while (verts_it != verts.end()) {
    // set vertex coordinate pointers
    double * xtmp, * ytmp, * ztmp;
    int n_vertices;
    rval = mbi->coords_iterate(verts_it, verts.end(), xtmp, ytmp, ztmp, n_vertices);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get direct access to vertex elements");

    tx.push_back(&(*xtmp));
    ty.push_back(&(*ytmp));
    tz.push_back(&(*ztmp));

    // move iterator forward
    verts_it += n_vertices;
  }

}
