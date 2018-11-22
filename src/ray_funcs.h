
#ifndef DD_RAY_FUNCS_H
#define DD_RAY_FUNCS_H

#include "ray.h"

double dot_prod(RTCDRay ray);

bool in_facets(MBRay ray, moab::EntityHandle tri);

void backface_cull(MBRay &ray, void*);

void frontface_cull(MBRay &ray, void*);

void count_hits(MBRayAccumulate &ray, void*);

#endif
