
#include "ray.h"
#include "ray_funcs.h"

double dot_prod(MBRay ray) {
  moab::CartVect ray_dir(ray.dir[0], ray.dir[1], ray.dir[2]);
  moab::CartVect tri_norm(ray.Ng[0], ray.Ng[1], ray.Ng[2]);

  return ray_dir % tri_norm;
}

bool in_facets(MBRay ray, moab::EntityHandle tri) {
  if ( ray.prev_facets && std::find(ray.prev_facets->begin(), ray.prev_facets->end(), tri) != ray.prev_facets->end() ) {
    return true;
  }
  else {
    return false;
  }
}

void backface_cull(MBRay &ray, void*) {

  if( in_facets(ray, ray.primID) ) {
    ray.geomID = -1;
    ray.primID = -1;
  }
      
  if(dot_prod(ray) < 0.0) {
    ray.geomID = -1;
    ray.primID = -1;
  }

  return;
}

void frontface_cull(MBRay &ray, void*) {

  if( in_facets(ray, ray.primID) ) {
    ray.geomID = -1;
    ray.primID = -1;
  }

  if(dot_prod(ray) > 0.0) {
    ray.geomID = -1;
    ray.primID = -1;
  }

  return;
}

void count_hits(MBRayAccumulate &ray, void*) {

  if (dot_prod(ray) > 0.0) {
    ray.sum -= 1; // leaving
  }
  else {
    ray.sum += 1; // entering
  }

  ray.geomID = -1;
  ray.primID = -1;

  ray.num_hit++;
  return;
}
