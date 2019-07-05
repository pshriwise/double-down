
#ifndef DD_RAY_H
#define DD_RAY_H

#include "embree3/rtcore_ray.h"
#include "Vec3da.h"

// This might go away with the alorithm update
enum RayFireType { RF, PIV, ACCUM };

struct RTCRay2 : RTCRay { RayFireType rf_type; };

// TO-DO: there should be a few more double elements here (barycentric coords)
struct RTCDRay: RTCRay2 {
  void set_org(double o[3]) {
    org_x = o[0]; org_y = o[1]; org_z = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  void set_org(const double o[3]) {
    org_x = o[0]; org_y = o[1]; org_z = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  void set_org(const Vec3da& o) {
    org_x = o[0]; org_y = o[1]; org_z = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  void set_dir(double o[3]) {
    dir_x = o[0]; dir_y = o[1]; dir_z = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  void set_dir(const double o[3]) {
    dir_x = o[0]; dir_y = o[1]; dir_z = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  void set_dir(const Vec3da& o) {
    dir_x = o[0]; dir_y = o[1]; dir_z = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  void set_len(double len) {
    tfar = len;
    dtfar = len;
  }

  Vec3da dorg, ddir;
  double dtfar;
};

struct RTCDHit : RTCHit {
  Vec3da dNg;
};

struct RTCDRayHit {
  struct RTCDRay ray;
  struct RTCDHit hit;

  double dot_prod() {
    return dot(ray.ddir, hit.dNg);
  }

};

struct RTCDPointQuery : RTCPointQuery {

  void set_radius(double rad) {
    radius = rad;
    dradius = rad;
  }

  void set_point(const double xyz[3]) {
    x = xyz[0]; y = xyz[1]; z = xyz[2];
    dx = xyz[0]; dy = xyz[1]; dz = xyz[2];
  }

  unsigned int primID = RTC_INVALID_GEOMETRY_ID;
  unsigned int geomID = RTC_INVALID_GEOMETRY_ID;
  double dx, dy, dz;
  double dradius;
};

#endif
