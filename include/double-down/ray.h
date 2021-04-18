
#ifndef DD_RAY_H
#define DD_RAY_H

// Embree
#include "embree3/rtcore_ray.h"

// Double-down
#include "Vec3da.h"

enum RayFireType { RF, PIV, ACCUM };

// TO-DO: there should be a few more double elements here (barycentric coords)

/*! Stucture that is an extension of Embree's RTCRay with
    double precision versions of the origin, direction
    and intersection distance.
 */
struct RTCDRay: RTCRay {
  //! \brief Set both the single and double precision versions of the ray origin
  void set_org(double o[3]) {
    org_x = o[0]; org_y = o[1]; org_z = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  //! \brief Set both the single and double precision versions of the ray origin
  void set_org(const double o[3]) {
    org_x = o[0]; org_y = o[1]; org_z = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  //! \brief Set both the single and double precision versions of the ray origin
  void set_org(const Vec3da& o) {
    org_x = o[0]; org_y = o[1]; org_z = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  //! \brief Set both the single and double precision versions of the ray direction
  void set_dir(double o[3]) {
    dir_x = o[0]; dir_y = o[1]; dir_z = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  //! \brief Set both the single and double precision versions of the ray direction
  void set_dir(const double o[3]) {
    dir_x = o[0]; dir_y = o[1]; dir_z = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  //! \brief Set both the single and double precision versions of the ray direction
  void set_dir(const Vec3da& o) {
    dir_x = o[0]; dir_y = o[1]; dir_z = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  //! \brief Set both the single and double precision versions of the ray length
  void set_len(double len) {
    tfar = len;
    dtfar = len;
  }

  // Member variables
  RayFireType rf_type; //!< Enum indicating the type of query this ray is used for
  Vec3da dorg, ddir; //!< double precision versions of the origin and ray direction
  double dtfar; //!< double precision version of the ray length
};

/*! Structure extending Embree's RayHit to include a double precision version of the primitive normal */
struct RTCDHit : RTCHit {
  // data members
  Vec3da dNg; //!< Double precision version of the primitive normal
};

/*! Stucture combining the ray and ray-hit structures to be passed to Embree queries */
struct RTCDRayHit {
  struct RTCDRay ray; //<! Extended version of the Embree RTCRay struct with double precision values
  struct RTCDHit hit; //<! Extended version of the Embree RTDRayHit struct with double precision values

  //! \brief Compute the dot product of the ray direction and current hit normal
  double dot_prod() {
    return dot(ray.ddir, hit.dNg);
  }

};

/*! Structure extending Embree's RTCPointQuery to include double precision values */
struct RTCDPointQuery : RTCPointQuery {
  //! \brief Set both the single and double precision versions of the query radius
  void set_radius(double rad) {
    radius = rad;
    dradius = rad;
  }

  //! \brief Set both the single and double precision versions of the query location
  void set_point(const double xyz[3]) {
    x = xyz[0]; y = xyz[1]; z = xyz[2];
    dx = xyz[0]; dy = xyz[1]; dz = xyz[2];
  }

  unsigned int primID = RTC_INVALID_GEOMETRY_ID; //<! ID of the nearest primitive
  unsigned int geomID = RTC_INVALID_GEOMETRY_ID; //<! ID of the nearest geometry
  double dx, dy, dz; //<! Double precision version of the query location
  double dradius; //!< Double precision version of the query distance
};

#endif
