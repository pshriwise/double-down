
#ifndef VEC3FA_H
#define VEC3FA_H

#include <assert.h>
#include <iostream>
#include <math.h>
#include "constants.h"
#include "Vec3ba.h"
#include "Vec3da.h"
#include "sys.h"
#include <immintrin.h>

struct __aligned(16) Vec3fa {
  typedef float Scalar;
  enum { n = 3 };
  union{ __m128 v; struct {float x,y,z; int a;}; };

  __forceinline Vec3fa () {}

  __forceinline Vec3fa            ( const Vec3fa& other ) { x = other.x; y = other.y; z = other.z; a = other.a; }

  __forceinline Vec3fa            ( const Vec3da& other ) { x = (float)other.x; y = (float)other.y; z = (float)other.z; a = other.a; }

  __forceinline Vec3fa& operator =( const Vec3fa& other ) { x = other.x; y = other.y; z = other.z; a = other.a; return *this;}

  __forceinline Vec3fa( const float pa ) { x = pa; y = pa; z = pa; a = pa;}
  __forceinline Vec3fa( const float pa[3]) { x = pa[0]; y = pa[1]; z = pa[2]; }
  __forceinline Vec3fa( const float px, const float py, const float pz) { x = px; y = py; z = pz; a = pz;}

  __forceinline Vec3fa( const float px, const float py, const float pz, const int pa) { x = px; y = py; z = pz; a = pa; }

  __forceinline Vec3fa( ZeroTy ) { x = 0.0f; y = 0.0f; z = 0.0f; a = 0;}
  __forceinline Vec3fa( PosInfTy ) { x = inf; y = inf; z = inf; a = inf; };
  __forceinline Vec3fa( NegInfTy ) { x = neg_inf; y = neg_inf; z = neg_inf; a = neg_inf; };

  __forceinline const float& operator[](const size_t index) const { assert(index < 3); return (&x)[index]; }
  __forceinline       float& operator[](const size_t index)       { assert(index < 3); return (&x)[index]; }

  __forceinline float length () const { return sqrtf(x*x + y*y + z*z); }

  __forceinline Vec3fa normalize() {
    float len = length();
    len = len < min_rcp_input ? min_rcp_input : len;
    x /= len; y /= len; z/= len;
    return *this;
  }

};


__forceinline Vec3fa operator +( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x+c.x, b.y+c.y, b.z+c.z, b.a+c.a); }
__forceinline Vec3fa operator -( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x-c.x, b.y-c.y, b.z-c.z, b.a-c.a); }
__forceinline Vec3fa operator *( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x*c.x, b.y*c.y, b.z*c.z, b.a*c.a); }
__forceinline Vec3fa operator *( const float& pa, const Vec3fa& c ) { return Vec3fa(pa) * c; }
__forceinline Vec3fa operator *( const Vec3fa& c, const float& pa ) { return Vec3fa(pa) * c; }
__forceinline Vec3fa operator /( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x/c.x, b.y/c.y, b.z/c.z, b.a/c.a); }
__forceinline Vec3fa operator /( const float& pa, const Vec3fa& c ) { return Vec3fa(pa) / c; }
__forceinline Vec3fa operator /( const Vec3fa& c, const float& pa ) { return c / Vec3fa(pa); }


__forceinline bool operator ==( const Vec3fa& b, const Vec3fa& c) { return b.x == c.x &&
							            b.y == c.y &&
							            b.z == c.z;
                                                           }

__forceinline const Vec3fa min( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(std::min(b.x,c.x),std::min(b.y,c.y),
									   std::min(b.z,c.z),std::min(b.a,c.a)); }
__forceinline const Vec3fa max( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(std::max(b.x,c.x),std::max(b.y,c.y),
									   std::max(b.z,c.z),std::max(b.a,c.a)); }

__forceinline const Vec3ba ge_mask( const Vec3fa& b, const Vec3fa& c ) { return Vec3ba(b.x >= c.x,b.y >= c.y,b.z >= c.z,b.a >= c.a); }
__forceinline const Vec3ba le_mask( const Vec3fa& b, const Vec3fa& c ) { return Vec3ba(b.x <= c.x,b.y <= c.y,b.z <= c.z,b.a <= c.a); }

__forceinline float reduce_add( const Vec3fa &v ) { return v.x + v.y + v.z; }


__forceinline float reduce_mul( const Vec3fa& v ) { return v.x * v.y * v.z; }

__forceinline float reduce_min( const Vec3fa& v ) { return std::min(std::min(v.x, v.y), v.z); }

__forceinline float reduce_max( const Vec3fa& v ) { return std::max(std::max(v.x, v.y), v.z); }

__forceinline float halfArea(Vec3fa v) { return v.x*(v.y+v.z)+(v.y*v.z); }


__forceinline Vec3fa inf_fix( const Vec3fa &a ) {
  return Vec3fa( fabs(a.x) == float(inf) ? 1/float(min_rcp_input) : a.x,
		 fabs(a.y) == float(inf) ? 1/float(min_rcp_input) : a.y,
		 fabs(a.z) == float(inf) ? 1/float(min_rcp_input) : a.z);
}

__forceinline Vec3fa zero_fix( const Vec3fa& a )
  {
    return Vec3fa(fabs(a.x) < min_rcp_input ? float(min_rcp_input) : a.x,
                   fabs(a.y) < min_rcp_input ?  float(min_rcp_input) : a.y,
                   fabs(a.z) < min_rcp_input ? float(min_rcp_input) : a.z);
  }

__forceinline const Vec3fa rcp(const Vec3fa& v ) { return Vec3fa(1.0f/v.x,
							     1.0f/v.y,
							     1.0f/v.z); }

__forceinline const Vec3fa rcp_safe(const Vec3fa& a) { return rcp(zero_fix(a)); }

__forceinline Vec3fa operator +( const Vec3fa &a ) { return Vec3fa(+a.x, +a.y, +a.z); }

__forceinline Vec3fa operator -( const Vec3fa &a ) { return Vec3fa(-a.x, -a.y, -a.z); }

__forceinline float dot( const Vec3fa& a, const Vec3fa& b ) { return reduce_add(a*b); }



__forceinline std::ostream& operator <<(std::ostream &os, Vec3fa  const& v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v.a << ']';
}

#endif
