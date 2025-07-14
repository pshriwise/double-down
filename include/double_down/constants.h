
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <limits>

namespace double_down {
// for abs(x) >= min_rcp_input the newton raphson rcp calculation does not fail
static const float min_rcp_input = std::numeric_limits<float>::min() /* FIX ME */ *1E5 /* SHOULDNT NEED TO MULTIPLY BY THIS VALUE */;
static const int BVH_MAX_DEPTH = 64;

struct NegInfTy
{
  constexpr operator          double   ( ) const { return -std::numeric_limits<double>::infinity(); }
  constexpr operator          float    ( ) const { return -std::numeric_limits<float>::infinity(); }
  constexpr operator          long long( ) const { return std::numeric_limits<long long>::min(); }
  constexpr operator unsigned long long( ) const { return std::numeric_limits<unsigned long long>::min(); }
  constexpr operator          long     ( ) const { return std::numeric_limits<long>::min(); }
  constexpr operator unsigned long     ( ) const { return std::numeric_limits<unsigned long>::min(); }
  constexpr operator          int      ( ) const { return std::numeric_limits<int>::min(); }
  constexpr operator unsigned int      ( ) const { return std::numeric_limits<unsigned int>::min(); }
  constexpr operator          short    ( ) const { return std::numeric_limits<short>::min(); }
  constexpr operator unsigned short    ( ) const { return std::numeric_limits<unsigned short>::min(); }
  constexpr operator          char     ( ) const { return std::numeric_limits<char>::min(); }
  constexpr operator unsigned char     ( ) const { return std::numeric_limits<unsigned char>::min(); }

};
constexpr NegInfTy neg_inf;

struct EmptyTy {
};
constexpr EmptyTy empty;

struct PosInfTy
{
  constexpr operator          double   ( ) const { return std::numeric_limits<double>::infinity(); }
  constexpr operator          float    ( ) const { return std::numeric_limits<float>::infinity(); }
  constexpr operator          long long( ) const { return std::numeric_limits<long long>::max(); }
  constexpr operator unsigned long long( ) const { return std::numeric_limits<unsigned long long>::max(); }
  constexpr operator          long     ( ) const { return std::numeric_limits<long>::max(); }
  constexpr operator unsigned long     ( ) const { return std::numeric_limits<unsigned long>::max(); }
  constexpr operator          int      ( ) const { return std::numeric_limits<int>::max(); }
  constexpr operator unsigned int      ( ) const { return std::numeric_limits<unsigned int>::max(); }
  constexpr operator          short    ( ) const { return std::numeric_limits<short>::max(); }
  constexpr operator unsigned short    ( ) const { return std::numeric_limits<unsigned short>::max(); }
  constexpr operator          char     ( ) const { return std::numeric_limits<char>::max(); }
  constexpr operator unsigned char     ( ) const { return std::numeric_limits<unsigned char>::max(); }
};
constexpr PosInfTy inf;


struct ZeroTy
{
  constexpr operator          double   ( ) const { return 0; }
  constexpr operator          float    ( ) const { return 0; }
  constexpr operator          long long( ) const { return 0; }
  constexpr operator unsigned long long( ) const { return 0; }
  constexpr operator          long     ( ) const { return 0; }
  constexpr operator unsigned long     ( ) const { return 0; }
  constexpr operator          int      ( ) const { return 0; }
  constexpr operator unsigned int      ( ) const { return 0; }
  constexpr operator          short    ( ) const { return 0; }
  constexpr operator unsigned short    ( ) const { return 0; }
  constexpr operator          char     ( ) const { return 0; }
  constexpr operator unsigned char     ( ) const { return 0; }
};
constexpr ZeroTy zero;

} // end namespace double_down

#endif
