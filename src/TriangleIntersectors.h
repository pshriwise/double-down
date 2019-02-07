
#ifndef DD_TRI_INTERSECT_H
#define DD_TRI_INTERSECT_H

#include "Vec3da.h"

#define EXIT_EARLY if(type) *type = NONE; return false;

enum intersection_type {NONE=0, INTERIOR, NODE0, NODE1, NODE2, EDGE0, EDGE1, EDGE2};
/* intersection type is determined by which of the intermediate values are == 0.  There
   are three such values that can therefore be encoded in a 3 bit integer.
   0 = none are == 0 -> interior type
   1 = pip0 == 0 -> EDGE0
   2 = pip1 == 1 -> EDGE1
   4 = pip2 == 2 -> EDGE2
   5 = pip2 = pip0 == 0 -> NOEE0
   3 = pip0 = pip1 == 0 -> NODE1
   6 = pip1 = pip2 == 0 -> NODE2 */
const intersection_type type_list[] = {INTERIOR, EDGE0, EDGE1, NODE1, EDGE2, NODE0, NODE2};

/* Function to return the vertex with the lowest coordinates. To force the same
   ray-edge computation, the Plücker test needs to use consistent edge
   representation. This would be more simple with MOAB handles instead of
   coordinates... */
inline bool first( const Vec3da& a, const Vec3da& b) {
  if(a[0] < b[0]) {
    return true;
  } else if(a[0] == b[0]) {
    if(a[1] < b[1]) {
      return true;
    } else if(a[1] == b[1]) {
      if(a[2] < b[2]) {
	return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
}

inline double plucker_edge_test(const Vec3da& vertexa, const Vec3da& vertexb,
                         const Vec3da& ray, const Vec3da& ray_normal) {
  double pip;
  const double near_zero = 10*std::numeric_limits<double>::epsilon();

  if(first(vertexa,vertexb)) {
    const Vec3da edge = vertexb-vertexa;
    const Vec3da edge_normal = cross(edge,vertexa);
    pip = dot(ray,edge_normal) + dot(ray_normal,edge);
  } else {
    const Vec3da edge = vertexa-vertexb;
    const Vec3da edge_normal = cross(edge,vertexb);
    pip = dot(ray,edge_normal) + dot(ray_normal,edge);
    pip = -pip;
  }

  if (near_zero > fabs(pip)) pip = 0.0;

  return pip;
}

/* This test uses the same edge-ray computation for adjacent triangles so that
   rays passing close to edges/nodes are handled consistently.

   Reports intersection type for post processing of special cases. Optionally
   screen by orientation and negative/nonnegative distance limits.

   If screening by orientation, substantial pruning can occur. Indicate
   desired orientation by passing 1 (forward), -1 (reverse), or 0 (no preference).
   Note that triangle orientation is not always the same as surface
   orientation due to non-manifold surfaces.

   N. Platis and T. Theoharis, "Fast Ray-Tetrahedron Intersection using Plücker
   Coordinates", Journal of Graphics Tools, Vol. 8, Part 4, Pages 37-48 (2003). */
inline bool plucker_ray_tri_intersect( const Vec3da vertices[3],
                                const Vec3da& origin,
                                const Vec3da& direction,
                                double& dist_out,
                                const double* nonneg_ray_len,
                                const double* neg_ray_len = NULL,
                                const int*    orientation = NULL,
                                intersection_type* type = NULL) {

  const Vec3da raya = direction;
  const Vec3da rayb = cross(direction, origin);

  // Determine the value of the first Plucker coordinate from edge 0
  double plucker_coord0 = plucker_edge_test(vertices[0], vertices[1], raya, rayb);

  // If orientation is set, confirm that sign of plucker_coordinate indicate
  // correct orientation of intersection
  if(orientation && (*orientation)*plucker_coord0 > 0) {
    EXIT_EARLY
  }

  // Determine the value of the second Plucker coordinate from edge 1
  double plucker_coord1 = plucker_edge_test(vertices[1], vertices[2], raya, rayb);

  // If orientation is set, confirm that sign of plucker_coordinate indicate
  // correct orientation of intersection
  if(orientation) {
    if( (*orientation)*plucker_coord1 > 0) {
      EXIT_EARLY
    }
  // If the orientation is not specified, all plucker_coords must be the same sign or zero.
  } else if( (0.0<plucker_coord0 && 0.0>plucker_coord1) || (0.0>plucker_coord0 && 0.0<plucker_coord1) ) {
    EXIT_EARLY
  }

  // Determine the value of the second Plucker coordinate from edge 2
  double plucker_coord2 = plucker_edge_test(vertices[2], vertices[0], raya, rayb);

  // If orientation is set, confirm that sign of plucker_coordinate indicate
  // correct orientation of intersection
  if(orientation) {
    if( (*orientation)*plucker_coord2 > 0) {
      EXIT_EARLY
    }
  // If the orientation is not specified, all plucker_coords must be the same sign or zero.
  } else if( (0.0<plucker_coord1 && 0.0>plucker_coord2) || (0.0>plucker_coord1 && 0.0<plucker_coord2) ||
             (0.0<plucker_coord0 && 0.0>plucker_coord2) || (0.0>plucker_coord0 && 0.0<plucker_coord2) ) {
    EXIT_EARLY
  }

  // check for coplanar case to avoid dividing by zero
  if(0.0==plucker_coord0 && 0.0==plucker_coord1 && 0.0==plucker_coord2) {
    EXIT_EARLY
  }

  // get the distance to intersection
  const double inverse_sum = 1.0/(plucker_coord0+plucker_coord1+plucker_coord2);
  assert(0.0 != inverse_sum);
  const Vec3da intersection(plucker_coord0*inverse_sum*vertices[2]+
			    plucker_coord1*inverse_sum*vertices[0]+
			    plucker_coord2*inverse_sum*vertices[1]);

  // To minimize numerical error, get index of largest magnitude direction.
  int idx = 0;
  double max_abs_dir = 0;
  for(unsigned int i=0; i<3; ++i) {
    if( fabs(direction[i]) > max_abs_dir ) {
      idx = i;
      max_abs_dir = fabs(direction[i]);
    }
  }
  const double dist = (intersection[idx]-origin[idx])/direction[idx];

  // is the intersection within distance limits?
  if((nonneg_ray_len && *nonneg_ray_len<dist) || // intersection is beyond positive limit
     (neg_ray_len && *neg_ray_len>=dist) ||      // intersection is behind negative limit
     (!neg_ray_len && 0>dist) ) {                  // Unless a neg_ray_len is used, don't return negative distances
    EXIT_EARLY
  }

  dist_out = dist;

  if (type) *type = type_list[  ( (0.0==plucker_coord2)<<2 ) +
                                ( (0.0==plucker_coord1)<<1 ) +
                                  ( 0.0==plucker_coord0 )       ];

  return true;
}

inline void closest_location_on_tri( const Vec3da& location,
                              const Vec3da* vertices,
                              Vec3da& closest_out )
{                                                     // ops      comparisons
  const Vec3da sv( vertices[1] - vertices[0] );   // +3 = 3
  const Vec3da tv( vertices[2] - vertices[0] );   // +3 = 6
  const Vec3da pv( vertices[0] - location );      // +3 = 9
  const double ss = dot(sv, sv);                          // +5 = 14
  const double st = dot(sv, tv);                          // +5 = 19
  const double tt = dot(tv, tv);                          // +5 = 24
  const double sp = dot(sv, pv);                          // +5 = 29
  const double tp = dot(tv, pv);                          // +5 = 34
  const double det = ss*tt - st*st;                   // +3 = 37
  double s = st*tp - tt*sp;                           // +3 = 40
  double t = st*sp - ss*tp;                           // +3 = 43
  if (s+t < det) {                                    // +1 = 44, +1 = 1
    if (s < 0) {                                      //          +1 = 2
      if (t < 0) {                                    //          +1 = 3
        // region 4
        if (sp < 0) {                                 //          +1 = 4
          if (-sp > ss)                               //          +1 = 5
            closest_out = vertices[1];                //      44       5
          else
            closest_out = vertices[0] - (sp/ss) * sv; // +7 = 51,      5
        }
        else if (tp < 0) {                            //          +1 = 5
          if (-tp > tt)                               //          +1 = 6
            closest_out = vertices[2];                //      44,      6
          else
            closest_out = vertices[0] - (tp/tt) * tv; // +7 = 51,      6
        }
        else {
          closest_out = vertices[0];                  //      44,      5
        }
      }
      else {
        // region 3
        if (tp >= 0)                                  //          +1 = 4
          closest_out = vertices[0];                  //      44,      4
        else if (-tp >= tt)                           //          +1 = 5
          closest_out = vertices[2];                  //      44,      5
        else
          closest_out = vertices[0] - (tp/tt) * tv;   // +7 = 51,      5
      }
    }
    else if (t < 0) {                                 //          +1 = 3
      // region 5;
      if (sp >= 0.0)                                  //          +1 = 4
        closest_out = vertices[0];                    //      44,      4
      else if (-sp >= ss)                             //          +1 = 5
        closest_out = vertices[1];                    //      44       5
      else
        closest_out = vertices[0] - (sp/ss) * sv;     // +7 = 51,      5
    }
    else {
      // region 0
      const double inv_det = 1.0 / det;               // +1 = 45
      s *= inv_det;                                   // +1 = 46
      t *= inv_det;                                   // +1 = 47
      closest_out = vertices[0] + s*sv + t*tv;        //+12 = 59,      3
    }
  }
  else {
    if (s < 0) {                                      //          +1 = 2
      // region 2
      s = st + sp;                                    // +1 = 45
      t = tt + tp;                                    // +1 = 46
      if (t > s) {                                    //          +1 = 3
        const double num = t - s;                     // +1 = 47
        const double den = ss - 2*st + tt;            // +3 = 50
        if (num > den)                                //          +1 = 4
          closest_out = vertices[1];                  //      50,      4
        else {
          s = num/den;                                // +1 = 51
          t = 1 - s;                                  // +1 = 52
          closest_out = s*vertices[1] + t*vertices[2];// +9 = 61,      4
        }
      }
      else if (t <= 0)                                //          +1 = 4
        closest_out = vertices[2];                    //      46,      4
      else if (tp >= 0)                               //          +1 = 5
        closest_out = vertices[0];                    //      46,      5
      else
        closest_out = vertices[0] - (tp/tt) * tv;     // +7 = 53,      5
    }
    else if (t < 0) {                                 //          +1 = 3
      // region 6
      t = st + tp;                                    // +1 = 45
      s = ss + sp;                                    // +1 = 46
      if (s > t) {                                    //          +1 = 4
        const double num = t - s;                     // +1 = 47
        const double den = tt - 2*st + ss;            // +3 = 50
        if (num > den)                                //          +1 = 5
          closest_out = vertices[2];                  //      50,      5
        else {
          t = num/den;                                // +1 = 51
          s = 1 - t;                                  // +1 = 52
          closest_out = s*vertices[1] + t*vertices[2];// +9 = 61,      5
        }
      }
      else if (s <= 0)                                //          +1 = 5
        closest_out = vertices[1];                    //      46,      5
      else if (sp >= 0)                               //          +1 = 6
        closest_out = vertices[0];                    //      46,      6
      else
        closest_out = vertices[0] - (sp/ss) * sv;     // +7 = 53,      6
    }
    else {
      // region 1
      const double num = tt + tp - st - sp;           // +3 = 47
      if (num <= 0) {                                 //          +1 = 4
        closest_out = vertices[2];                    //      47,      4
      }
      else {
        const double den = ss - 2*st + tt;            // +3 = 50
        if (num >= den)                               //          +1 = 5
          closest_out = vertices[1];                  //      50,      5
        else {
          s = num/den;                                // +1 = 51
          t = 1 - s;                                  // +1 = 52
          closest_out = s*vertices[1] + t*vertices[2];// +9 = 61,      5
        }
      }
    }
  }
}

#endif
