
#ifndef DD_VEC3DA_H
#define DD_VEC3DA_H

#include <assert.h>
#include <iostream>
#include <math.h>
#include <immintrin.h>
#include <xmmintrin.h>
#include <limits>

#ifndef NDEBUG
#define __forceinline inline
#else
#define __forceinline inline __attribute__((always_inline))
#endif

#include "constants.h"

struct Vec3da {
  typedef double Scalar;
  enum { n = 3 };
  union{
#ifdef __AVX2__
    __m256 v;
#endif
    struct{ double x,y,z;  size_t a;};
  };

  __forceinline Vec3da () {}
#ifdef __AVX2__
  __forceinline Vec3da ( const __m256 v_ ) : v(v_) { }
#endif
  __forceinline Vec3da            ( const Vec3da& other ) { x = other.x; y = other.y; z = other.z; a = other.a; }
  __forceinline Vec3da& operator =( const Vec3da& other ) { x = other.x; y = other.y; z = other.z; a = other.a; return *this;}

  __forceinline Vec3da( const double pa ) { x = pa; y = pa; z = pa; a = pa;}
  __forceinline Vec3da( const double pa[3]) { x = pa[0]; y = pa[1]; z = pa[2]; }
  __forceinline Vec3da( const double px, const double py, const double pz) { x = px; y = py; z = pz; a = pz;}

  __forceinline Vec3da( const double px, const double py, const double pz, const int pa) { x = px; y = py; z = pz; a = pa; }

  /* __forceinline Vec3da( ZeroTy ) { x = 0.0f; y = 0.0f; z = 0.0f; a = 0;} */
  /* __forceinline Vec3da( PosInfTy ) { x = inf; y = inf; z = inf; a = inf; }; */
  /* __forceinline Vec3da( NegInfTy ) { x = neg_inf; y = neg_inf; z = neg_inf; a = neg_inf; }; */

  __forceinline const double& operator[](const size_t index) const { assert(index < 3); return (&x)[index]; }
  __forceinline       double& operator[](const size_t index)       { assert(index < 3); return (&x)[index]; }

  __forceinline double length () const { return sqrt(length_sqr()); }

  __forceinline double length_sqr () const { return x*x + y*y + z*z; }

  __forceinline Vec3da normalize() {
    double len = length();
    len = len < min_rcp_input ? min_rcp_input : len;
    x /= len; y /= len; z/= len;
    return *this;
  }

    __forceinline Vec3da operator +=(const Vec3da& v) { x = x + v.x; y = y +v.y; z = z + v.z; return *this; }

    __forceinline Vec3da operator -=(const Vec3da& v) { x = x - v.x; y = y -v.y; z = z - v.z; return *this; }

    __forceinline Vec3da operator *=(const Vec3da& v) { x = x * v.x; y = y *v.y; z = z * v.z; return *this; }

    __forceinline Vec3da operator /=(const Vec3da& v) { x = x / v.x; y = y /v.y; z = z / v.z; return *this; }

};


__forceinline Vec3da operator +( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x+c.x, b.y+c.y, b.z+c.z, b.a+c.a); }
__forceinline Vec3da operator -( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x-c.x, b.y-c.y, b.z-c.z, b.a-c.a); }
__forceinline Vec3da operator *( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x*c.x, b.y*c.y, b.z*c.z, b.a*c.a); }
__forceinline Vec3da operator *( const double& pa, const Vec3da& c ) { return Vec3da(pa) * c; }
__forceinline Vec3da operator *( const Vec3da& c, const double& pa ) { return Vec3da(pa) * c; }
__forceinline Vec3da operator /( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x/c.x, b.y/c.y, b.z/c.z, b.a/c.a); }
__forceinline Vec3da operator /( const double& pa, const Vec3da& c ) { return Vec3da(pa) / c; }
__forceinline Vec3da operator /( const Vec3da& c, const double& pa ) { return c / Vec3da(pa); }


__forceinline bool operator ==( const Vec3da& b, const Vec3da& c) { return b.x == c.x &&
							            b.y == c.y &&
							            b.z == c.z;
                                                           }
#if defined(__AVX2__)
__forceinline const Vec3da min( const Vec3da& a, const Vec3da& b ) { return _mm256_min_ps(a.v, b.v); }
__forceinline const Vec3da max( const Vec3da& a, const Vec3da& b ) { return _mm256_max_ps(a.v, b.v); }
#else
__forceinline const Vec3da min( const Vec3da& b, const Vec3da& c ) { return Vec3da(std::min(b.x,c.x),std::min(b.y,c.y),
									   std::min(b.z,c.z),std::min(b.a,c.a)); }
__forceinline const Vec3da max( const Vec3da& b, const Vec3da& c ) { return Vec3da(std::max(b.x,c.x),std::max(b.y,c.y),
									   std::max(b.z,c.z),std::max(b.a,c.a)); }
#endif

/* __forceinline const Vec3ba ge_mask( const Vec3da& b, const Vec3da& c ) { return Vec3ba(b.x >= c.x,b.y >= c.y,b.z >= c.z,b.a >= c.a); } */
/* __forceinline const Vec3ba le_mask( const Vec3da& b, const Vec3da& c ) { return Vec3ba(b.x <= c.x,b.y <= c.y,b.z <= c.z,b.a <= c.a); } */

__forceinline double reduce_add( const Vec3da &v ) { return v.x + v.y + v.z; }


__forceinline double reduce_mul( const Vec3da& v ) { return v.x * v.y * v.z; }

__forceinline double reduce_min( const Vec3da& v ) { return std::min(std::min(v.x, v.y), v.z); }

__forceinline double reduce_max( const Vec3da& v ) { return std::max(std::max(v.x, v.y), v.z); }

__forceinline double halfArea(Vec3da v) { return v.x*(v.y+v.z)+(v.y*v.z); }

__forceinline Vec3da zero_fix( const Vec3da& a )
  {
    return Vec3da(fabs(a.x) < min_rcp_input ? double(min_rcp_input) : a.x,
                   fabs(a.y) < min_rcp_input ?  double(min_rcp_input) : a.y,
                   fabs(a.z) < min_rcp_input ? double(min_rcp_input) : a.z);
  }

__forceinline const Vec3da rcp(const Vec3da& v ) { return Vec3da(1.0f/v.x,
							     1.0f/v.y,
							     1.0f/v.z); }

__forceinline const Vec3da rcp_safe(const Vec3da& a) { return rcp(zero_fix(a)); }

__forceinline Vec3da operator +( const Vec3da &a ) { return Vec3da(+a.x, +a.y, +a.z); }

__forceinline Vec3da operator -( const Vec3da &a ) { return Vec3da(-a.x, -a.y, -a.z); }

__forceinline double dot( const Vec3da& a, const Vec3da& b ) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

__forceinline Vec3da cross( const Vec3da& a, const Vec3da& b ) { return Vec3da( a[1] * b[2] - a[2] * b[1],
										a[2] * b[0] - a[0] * b[2],
										a[0] * b[1] - a[1] * b[0] ); }


__forceinline std::ostream& operator <<(std::ostream &os, Vec3da  const& v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v.a << ']';
}

#endif
