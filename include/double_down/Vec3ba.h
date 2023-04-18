#ifndef VEC3BA_H
#define VEC3BA_H

#include <assert.h>
#include <iostream>
#include <math.h>
#include "constants.h"
#include "sys.h"

namespace double_down {

struct Vec3ba {
  enum { n = 3 };
  bool x,y,z;
  int a;

  __forceinline Vec3ba () {}

  __forceinline Vec3ba            ( const Vec3ba& other ) { x = other.x; y = other.y; z = other.z; a = other.a; }
  __forceinline Vec3ba& operator =( const Vec3ba& other ) { x = other.x; y = other.y; z = other.z; a = other.a; return *this;}

  __forceinline Vec3ba( const bool pa ) { x = pa; y = pa; z = pa; a = pa;}
  __forceinline Vec3ba( const bool pa[3]) { x = pa[0]; y = pa[1]; z = pa[2]; }
  __forceinline Vec3ba( const bool px, const bool py, const bool pz) { x = px; y = py; z=pz; }

  __forceinline Vec3ba( const bool px, const bool py, const bool pz, const bool pa) { x = px; y = py; z=pz; a = pa; }
  __forceinline Vec3ba( const bool px, const bool py, const bool pz, const int pa) { x = px; y = py; z=pz; a = (bool)pa; }

  __forceinline Vec3ba( FalseTy ) : x(False),y(False),z(False),a(False) {}

  __forceinline const bool& operator[](const size_t index) const { assert(index < 3); return (&x)[index]; }
  __forceinline       bool& operator[](const size_t index)       { assert(index < 3); return (&x)[index]; }
};

__forceinline bool all(Vec3ba v) { return v[0] && (v[0] == v[1]) && (v[0] == v[2]); }

__forceinline std::ostream& operator <<(std::ostream &os, Vec3ba  const& v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v.a << ']';
}

} // end namespace double_down

#endif
