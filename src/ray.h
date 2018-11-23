
#ifndef DD_RAY_H
#define DD_RAY_H

#include "embree2/rtcore_ray.h"
#include "Vec3da.h"

// This might go away with the alorithm update
enum RayFireType { RF, PIV };

struct RTCRay2 : RTCRay { RayFireType rf_type; };

// TO-DO: there should be a few more double elements here (barycentric coords)
struct RTCDRay: RTCRay2 {
  void set_org(double o[3]) {
    org[0] = o[0]; org[1] = o[1]; org[2] = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  void set_org(const double o[3]) {
    org[0] = o[0]; org[1] = o[1]; org[2] = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }

  void set_org(const Vec3da& o) {
    org[0] = o[0]; org[1] = o[1]; org[2] = o[2];
    dorg[0] = o[0]; dorg[1] = o[1]; dorg[2] = o[2];
  }
  
  void set_dir(double o[3]) {
    dir[0] = o[0]; dir[1] = o[1]; dir[2] = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  void set_dir(const double o[3]) {
    dir[0] = o[0]; dir[1] = o[1]; dir[2] = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }

  void set_dir(const Vec3da& o) {
    dir[0] = o[0]; dir[1] = o[1]; dir[2] = o[2];
    ddir[0] = o[0]; ddir[1] = o[1]; ddir[2] = o[2];
  }
  
  void set_len(double len) {
    tfar = len;
    dtfar = len;
  }

  double dot_prod() {
    return dot(ddir, dNg);
  }

  Vec3da dorg, ddir, dNg;
  double dtfar;
};

#endif
