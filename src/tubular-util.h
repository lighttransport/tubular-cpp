#ifndef TUBULAR_UTIL_H_
#define TUBULAR_UTIL_H_

#include <algorithm>

#include "tubular-math.h"
#include "type.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#endif

#include "trackball.h"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

namespace tubular {
template <typename T>
inline T Clamp(const T x, const T a, const T b) {
  return std::max(a, std::min(b, x));
}

template <typename T>
inline T Saturate(const T x) {
  return Clamp(x, T(0), T(1));
}

inline bool IsFinite(const float3& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline bool IsZero(const float3& v) {
  return fabsf(v.x()) < kEps && fabsf(v.y()) < kEps && fabsf(v.z()) < kEps;
}

inline float3 ArbitraryAxisRotation(const float3& axis, const float rad,
                                    const float3& v) {
  float quat[4];
  float axis_[3];
  axis_[0] = axis[0];
  axis_[1] = axis[1];
  axis_[2] = axis[2];

  axis_to_quat(axis_, rad, quat);  // TODO check

  float m[4][4];
  build_rotmatrix(m, quat);
  float3 ret;

  // TODO check
  ret[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
  ret[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
  ret[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];

  return ret;
}

}  // namespace tubular

#endif  // TUBULAR_UTIL_H_
