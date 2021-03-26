
#ifndef SYS_H
#define SYS_H

#include <xmmintrin.h>

#ifndef NDEBUG
#define __forceinline
#else
#define __forceinline inline __attribute__((always_inline))
#endif

#define __aligned(...) __attribute__((aligned(__VA_ARGS__)))

inline void prefetchL1 (const void* ptr) { _mm_prefetch((const char*)ptr, _MM_HINT_T0); }
inline void prefetchL2 (const void* ptr) { _mm_prefetch((const char*)ptr, _MM_HINT_T1); }

#endif
