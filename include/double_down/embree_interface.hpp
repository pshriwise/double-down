
#ifdef EMBREE4

#include "double_down/embree4.hpp"

#elif defined(EMBREE3)

#include "double_down/embree3.hpp"

#else

#error "No embree version provided to compiler"

#endif