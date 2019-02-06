#ifndef VEC3BA_H
#define VEC3BA_H

#include <assert.h>
#include <iostream>
#include <math.h>
#include "constants.h"
#include "sys.h"

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

/* __forceinline Vec3ba operator +( const Vec3ba& b, const Vec3ba& c ) { return Vec3ba(b.x+c.x, b.y+c.y, b.z+c.z, b.a+c.a); } */
/* __forceinline Vec3ba operator -( const Vec3ba& b, const Vec3ba& c ) { return Vec3ba(b.x-c.x, b.y-c.y, b.z-c.z, b.a-c.a); } */
/* __forceinline Vec3ba operator *( const Vec3ba& b, const Vec3ba& c ) { return Vec3ba(b.x*c.x, b.y*c.y, b.z*c.z, b.a*c.a); } */
/* __forceinline Vec3ba operator *( const float& pa, const Vec3ba& c ) { return Vec3ba(pa) * c; } */
/* __forceinline Vec3ba operator *( const Vec3ba& c, const float& pa ) { return Vec3ba(pa) * c; } */
/* __forceinline Vec3ba operator /( const Vec3ba& b, const Vec3ba& c ) { return Vec3ba(b.x/c.x, b.y/c.y, b.z/c.z, b.a/c.a); } */
/* __forceinline Vec3ba operator /( const float& pa, const Vec3ba& c ) { return Vec3ba(pa) / c; } */
/* __forceinline Vec3ba operator /( const Vec3ba& c, const float& pa ) { return Vec3ba(pa) / c; } */

/* __forceinline const Vec3ba min( const Vec3ba& b, const Vec3ba& c ) { return Vec3ba(std::min(b.x,c.x),std::min(b.y,c.y), */
/* 									   std::min(b.z,c.z),std::min(b.a,c.a)); } */
/* __forceinline const Vec3ba max( const Vec3ba& b, const Vec3ba& c ) { return Vec3ba(std::max(b.x,c.x),std::max(b.y,c.y), */
/* 									   std::max(b.z,c.z),std::max(b.a,c.a)); } */

/* __forceinline const Vec3ba ge_mask( const Vec3ba& b, const Vec3ba& c ) { return Vec3<bool>(b.x <= c.x,b.y <= c.y,b.z <= c.z,b.a <= c.a); } */







__forceinline std::ostream& operator <<(std::ostream &os, Vec3ba  const& v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v.a << ']';
}

#endif
