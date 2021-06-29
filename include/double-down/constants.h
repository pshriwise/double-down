
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <limits>

// for abs(x) >= min_rcp_input the newton raphson rcp calculation does not fail
static const float min_rcp_input = std::numeric_limits<float>::min() /* FIX ME */ *1E5 /* SHOULDNT NEED TO MULTIPLY BY THIS VALUE */;
static const int BVH_MAX_DEPTH = 64;

/* we consider floating point numbers in that range as valid input numbers */
static float FLT_LARGE = 1.844E18f;

static struct UlpTy
{
  inline operator double( ) const { return std::numeric_limits<double>::epsilon(); }
  inline operator float ( ) const { return std::numeric_limits<float>::epsilon(); }
} ulp;

static struct NegInfTy
{
  inline operator          double   ( ) const { return -std::numeric_limits<double>::infinity(); }
  inline operator          float    ( ) const { return -std::numeric_limits<float>::infinity(); }
  inline operator          long long( ) const { return std::numeric_limits<long long>::min(); }
  inline operator unsigned long long( ) const { return std::numeric_limits<unsigned long long>::min(); }
  inline operator          long     ( ) const { return std::numeric_limits<long>::min(); }
  inline operator unsigned long     ( ) const { return std::numeric_limits<unsigned long>::min(); }
  inline operator          int      ( ) const { return std::numeric_limits<int>::min(); }
  inline operator unsigned int      ( ) const { return std::numeric_limits<unsigned int>::min(); }
  inline operator          short    ( ) const { return std::numeric_limits<short>::min(); }
  inline operator unsigned short    ( ) const { return std::numeric_limits<unsigned short>::min(); }
  inline operator          char     ( ) const { return std::numeric_limits<char>::min(); }
  inline operator unsigned char     ( ) const { return std::numeric_limits<unsigned char>::min(); }

} neg_inf;

static struct EmptyTy {
} empty;

static struct PosInfTy
{
  inline operator          double   ( ) const { return std::numeric_limits<double>::infinity(); }
  inline operator          float    ( ) const { return std::numeric_limits<float>::infinity(); }
  inline operator          long long( ) const { return std::numeric_limits<long long>::max(); }
  inline operator unsigned long long( ) const { return std::numeric_limits<unsigned long long>::max(); }
  inline operator          long     ( ) const { return std::numeric_limits<long>::max(); }
  inline operator unsigned long     ( ) const { return std::numeric_limits<unsigned long>::max(); }
  inline operator          int      ( ) const { return std::numeric_limits<int>::max(); }
  inline operator unsigned int      ( ) const { return std::numeric_limits<unsigned int>::max(); }
  inline operator          short    ( ) const { return std::numeric_limits<short>::max(); }
  inline operator unsigned short    ( ) const { return std::numeric_limits<unsigned short>::max(); }
  inline operator          char     ( ) const { return std::numeric_limits<char>::max(); }
  inline operator unsigned char     ( ) const { return std::numeric_limits<unsigned char>::max(); }
} inf;


static struct ZeroTy
{
  inline operator          double   ( ) const { return 0; }
  inline operator          float    ( ) const { return 0; }
  inline operator          long long( ) const { return 0; }
  inline operator unsigned long long( ) const { return 0; }
  inline operator          long     ( ) const { return 0; }
  inline operator unsigned long     ( ) const { return 0; }
  inline operator          int      ( ) const { return 0; }
  inline operator unsigned int      ( ) const { return 0; }
  inline operator          short    ( ) const { return 0; }
  inline operator unsigned short    ( ) const { return 0; }
  inline operator          char     ( ) const { return 0; }
  inline operator unsigned char     ( ) const { return 0; }
} zero;

static struct TrueTy {
  inline operator bool( ) const { return true; }
} True;

static struct FalseTy {
  inline operator bool( ) const { return false; }
} False;

#endif
