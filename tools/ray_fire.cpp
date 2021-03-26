#include <string>
#include <iostream>
#include <cassert>
#include <ctime>

#include "moab/ProgOptions.hpp"

#include "double-down/RTI.hpp"

static const double PI = acos(-1.0);
static const double denom = 1.0 / ((double) RAND_MAX);
static const double denomPI = PI * denom;

inline void RNDVEC(moab::CartVect& uvw) {
  // denom normalizes rand values (see global defines)
  double theta = 2 * denomPI * rand(); // randomly samples from 0 to az. (Default az is 2PI)
  double u = 2 * denom * rand() - 1; // randomly samples from -1 to 1.
  uvw[0] = sqrt(1 - u * u) * cos(theta);
  uvw[1] = sqrt(1 - u * u) * sin(theta);
  uvw[2] = u;
}

int main(int argc, char** argv) {

  ProgOptions po("Sample program for using the double-down interface to Embree.");

  std::string filename;
  po.addRequiredArg<std::string>("filename", "MOAB surface mesh to fire rays on.", &filename);

  po.parseCommandLine(argc, argv);

#ifdef __AVX2__
  std::cout << "AVX2 Enabled" << std::endl;
#endif

  RayTracingInterface* RTI = new RayTracingInterface();

  moab::ErrorCode rval;
  rval = RTI->init(filename);
  MB_CHK_SET_ERR(rval, "Failed to initialize the RayTracingInterface.");

  moab::Range vols;
  rval = RTI->get_vols(vols);
  MB_CHK_SET_ERR(rval, "Failed to get volumes from the RTI.");

  int num_rays = 1000000;
  double total= 0.0;
  std::clock_t mark;
  std::cout << "Firing " << num_rays
            << " randomly oriented rays from the origin..." << std::endl;
  for (int i = 0; i < num_rays; i++) {
    // setup ray
    RTCDRayHit rayhit;

    RTCDRay& ray = rayhit.ray;
    double org[3] = {0.0, 0.0, 0.0};
    ray.set_org(org);
    moab::CartVect dir;
    RNDVEC(dir);
    dir.normalize(); // unit len distance always
    ray.set_dir(dir.array());
    double len = 1000;
    ray.set_len(len);
    ray.mask = -1;
    ray.rf_type = RayFireType::RF;

    RTCDHit& hit = rayhit.hit;
    hit.geomID = RTC_INVALID_GEOMETRY_ID;
    hit.primID = RTC_INVALID_GEOMETRY_ID;

    // fire ray
    mark = std::clock();
    RTI->fire(vols.front(), rayhit);
    total += std::clock() - mark;

   // make sure we hit something
    if (rayhit.hit.geomID == -1) {
      std::cout << "Miss!" << std::endl;
      exit(1);
    }

  }

  double total_sec = total / (double)CLOCKS_PER_SEC;
  double per_ray = total_sec / (double)num_rays;

  std::cout << "Total time in Ray Fire: " << total_sec << " sec" << std::endl;
  std::cout << "Total time per Ray Fire: " << per_ray << " sec" << std::endl;

  delete RTI;
  return 0;
}
