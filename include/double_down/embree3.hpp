
#include "embree3/rtcore.h"

#ifndef DD_EMBREE_WRAPPERS
#define DD_EMBREE_WRAPPERS

// provide a function signature that matches the one in embree > v4.
// the context is created internally
inline void rtcIntersect1(RTCScene scene, RTCRayHit* rayhit) {
  {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene, &context, rayhit);
  }
}

#endif // include guard