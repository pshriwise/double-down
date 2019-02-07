
#ifndef AABB_H
#define AABB_H

#include <cstring>
#include <algorithm>


#include "Vec3.h"
// #include "Ray.h"
#include "constants.h"
#include "Vec3fa.h"

// Axis-Aligned Bounding Box Class
struct AABB {

  // member veriables
  Vec3fa lower, upper;

  // constructors
  // if not provided arguments, intentionally make an invalid box
  // so that it can be updated properly
  __forceinline AABB() : lower(inf), upper(neg_inf) { }

  __forceinline AABB(const EmptyTy& e) : lower(inf), upper(neg_inf) { }

  __forceinline AABB(float val) { lower[0] = val;
                           lower[1] = val;
			   lower[2] = val;
			   upper[0] = val;
			   upper[1] = val;
			   upper[2] = val; }


  __forceinline AABB (float x_min, float y_min, float z_min,
	       float x_max, float y_max, float z_max) {
    lower[0] = x_min; lower[1] = y_min; lower[2] = z_min;
    upper[0] = x_max; upper[1] = y_max; upper[2] = z_max;
  }

  __forceinline AABB( const float &low, const float &high) {
    lower = low;
    upper = high;
  }


  __forceinline AABB ( const float ext[6] ) {
    lower[0] = ext[0];
    lower[1] = ext[1];
    lower[2] = ext[2];
    upper[0] = ext[3];
    upper[1] = ext[4];
    upper[2] = ext[5];
  }

  __forceinline AABB ( const float low[3], const float high[3] ) {
    lower = Vec3fa(low);
    upper = Vec3fa(high);
  }

  __forceinline AABB ( const Vec3fa &l, const Vec3fa &u ) {
    lower = l;
    upper = u;
  }

  __forceinline AABB ( const Vec3f &l, const Vec3f &u ) {
    lower = Vec3fa(l.x,l.y,l.z);
    upper = Vec3fa(u.x,u.y,u.z);
  }

  __forceinline AABB& operator=( const AABB& other) {
    lower = other.lower;
    upper = other.upper;
    return *this;
  }

  // copy constructor
  __forceinline AABB ( const AABB& other) {
    lower = other.lower;
    upper = other.upper;
  }

  // extends the box to ensure containment of the provided box
  __forceinline void update (const AABB& other ) {
    this->update(other.lower.x, other.lower.y, other.lower.z);
    this->update(other.upper.x, other.upper.y, other.upper.z);
  }

  // extends the box to ensure containment of the x, y, and z values
  __forceinline void update ( const float& x_val, const float& y_val, const float& z_val ) {
    lower[0] = lower[0] < x_val ? lower[0] : x_val;
    lower[1] = lower[1] < y_val ? lower[1] : y_val;
    lower[2] = lower[2] < z_val ? lower[2] : z_val;
    upper[0] = upper[0] > x_val ? upper[0] : x_val;
    upper[1] = upper[1] > y_val ? upper[1] : y_val;
    upper[2] = upper[2] > z_val ? upper[2] : z_val;
  }

  // extends the box to ensure containment of the x, y, and z values
  __forceinline void update ( const float xyz[3] ) {
    lower[0] = std::min(lower[0], xyz[0]);
    lower[1] = std::min(lower[1], xyz[1]);
    lower[2] = std::min(lower[2], xyz[2]);
    upper[0] = std::max(upper[0], xyz[0]);
    upper[1] = std::max(upper[1], xyz[1]);
    upper[2] = std::max(upper[2], xyz[2]);
  }

  // clears the box - any update command will set the lower and upper extents of the box
  // note: this intentionally leaves the box in an invalid state
  __forceinline void clear() { lower = inf; upper = neg_inf; }

  __forceinline const AABB& extend(const AABB& other) { lower = min(lower, other.lower); upper = max(upper, other.upper); return *this; }

  // returns the dimensions of the box
  __forceinline Vec3fa size() const { return upper - lower; };

  // returns the center point of the box
  __forceinline Vec3fa center() const { return 0.5f*(lower + upper); }

  // returns the center point of the box plus a factor of 2
  __forceinline Vec3fa center2() const { return lower+upper; }

  // tests if the box is inverted
  __forceinline bool isValid() {
    return all(le_mask(lower, upper));
  }

  // determines if a point is inside the box
  __forceinline bool inside ( const Vec3fa& p ) { return all( ge_mask(p,lower)) && all(le_mask(p,upper) ); }


  __forceinline float nearest(const float pnt[3]) {
    if(inside(pnt)) { return 0.0f; }

    Vec3fa closest;
    closest[0] = pnt[0] < lower[0] ? lower[0] : pnt[0] > upper[0] ? upper[0] : pnt[0];
    closest[1] = pnt[1] < lower[1] ? lower[1] : pnt[1] > upper[1] ? upper[1] : pnt[1];
    closest[2] = pnt[2] < lower[2] ? lower[2] : pnt[2] > upper[2] ? upper[2] : pnt[2];
    return (closest - Vec3fa(pnt)).length();
  }

};

// determines if a point is inside the box
__forceinline bool inside ( const AABB &b, const Vec3fa& p ) { return all( ge_mask(p,b.lower)) && all(le_mask(p,b.upper) );
}

// returns a box that contains both boxes a and b
__forceinline AABB merge ( const AABB &a, const AABB &b ) { return AABB( min(a.lower,b.lower), max(a.upper,b.upper) );
}

// returns the volume of the box
__forceinline float volume ( const AABB &box ) { return reduce_mul(box.size()); }

// returns the halfArea of the box, efficient for situations in which
// many box areas are being computed, factor of two can be applied
// externally
__forceinline float halfArea ( const AABB &box ) { return halfArea(box.size()); }

// returns the area of the box
__forceinline float area ( const AABB &box ) { return 2.0f*halfArea(box); }

// equal operator
__forceinline bool operator ==(const AABB &a, const AABB& b) { return a.lower == b.lower && a.upper == b.upper; }

// print operator
__forceinline std::ostream& operator <<( std::ostream& os, const AABB &b ) {
  return os << "Lower Corner " << b.lower << std::endl
	    << "Upper Corner " << b.upper << std::endl;

}

#endif
