#ifndef TUBULAR_TYPE_H_
#define TUBULAR_TYPE_H_

#define NANORT_USE_CPP11_FEATURE
#include "nanort.h"

namespace tubular {

template <typename T = float>
class real4 {
public:
  real4() {}
  real4(T x) {
    v[0] = x;
    v[1] = x;
    v[2] = x;
    v[3] = x;
  }
  real4(T xx, T yy, T zz, T ww) {
    v[0] = xx;
    v[1] = yy;
    v[2] = zz;
    v[3] = ww;
  }
  explicit real4(const T *p) {
    v[0] = p[0];
    v[1] = p[1];
    v[2] = p[2];
    v[3] = p[3];
  }

  inline T x() const { return v[0]; }
  inline T y() const { return v[1]; }
  inline T z() const { return v[2]; }
  inline T w() const { return v[3]; }

  real4 operator*(T f) const {
    return real4(x() * f, y() * f, z() * f, w() * f);
  }
  real4 operator-(const real4 &f2) const {
    return real4(x() - f2.x(), y() - f2.y(), z() - f2.z(), w() - f2.w());
  }
  real4 operator*(const real4 &f2) const {
    return real4(x() * f2.x(), y() * f2.y(), z() * f2.z(), w() * f2.w());
  }
  real4 operator+(const real4 &f2) const {
    return real4(x() + f2.x(), y() + f2.y(), z() + f2.z(), w() + f2.w());
  }
  real4 &operator+=(const real4 &f2) {
    v[0] += f2.x();
    v[1] += f2.y();
    v[2] += f2.z();
    v[3] += f2.w();
    return (*this);
  }
  real4 operator/(const real4 &f2) const {
    return real4(x() / f2.x(), y() / f2.y(), z() / f2.z(), w() / f2.w());
  }
  real4 operator-() const { return real4(-x(), -y(), -z(), -w()); }
  T operator[](int i) const { return v[i]; }
  T &operator[](int i) { return v[i]; }

  T v[4];
  // T pad;  // for alignment (when T = float)
};

template <typename T>
inline real4<T> operator*(T f, const real4<T> &v) {
  return real4<T>(v.x() * f, v.y() * f, v.z() * f, v.w() * f);
}

template <typename T>
inline real4<T> vneg(const real4<T> &rhs) {
  return real4<T>(-rhs.x(), -rhs.y(), -rhs.z(), -rhs.w());
}

template <typename T>
inline T vlength(const real4<T> &rhs) {
  return std::sqrt(rhs.x() * rhs.x() + rhs.y() * rhs.y() + rhs.z() * rhs.z() +
                   rhs.w() * rhs.w());
}

template <typename T>
inline real4<T> vnormalize(const real4<T> &rhs) {
  real4<T> v = rhs;
  T len      = vlength(rhs);
  if (std::fabs(len) > std::numeric_limits<T>::epsilon()) {
    T inv_len = static_cast<T>(1.0) / len;
    v.v[0] *= inv_len;
    v.v[1] *= inv_len;
    v.v[2] *= inv_len;
    v.v[3] *= inv_len;
  }
  return v;
}

template <typename T = float>
class real2 {
public:
  real2() {}
  real2(T x) {
    v[0] = x;
    v[1] = x;
  }
  real2(T xx, T yy) {
    v[0] = xx;
    v[1] = yy;
  }
  explicit real2(const T *p) {
    v[0] = p[0];
    v[1] = p[1];
  }

  inline T x() const { return v[0]; }
  inline T y() const { return v[1]; }

  real2 operator*(T f) const { return real2(x() * f, y() * f); }
  real2 operator-(const real2 &f2) const {
    return real2(x() - f2.x(), y() - f2.y());
  }
  real2 operator*(const real2 &f2) const {
    return real2(x() * f2.x(), y() * f2.y());
  }
  real2 operator+(const real2 &f2) const {
    return real2(x() + f2.x(), y() + f2.y());
  }
  real2 &operator+=(const real2 &f2) {
    v[0] += f2.x();
    v[1] += f2.y();
    return (*this);
  }
  real2 operator/(const real2 &f2) const {
    return real2(x() / f2.x(), y() / f2.y());
  }
  real2 operator-() const { return real2(-x(), -y()); }
  T operator[](int i) const { return v[i]; }
  T &operator[](int i) { return v[i]; }

  T v[2];
  // T pad;  // for alignment (when T = float)
};

template <typename T>
inline real2<T> operator*(T f, const real2<T> &v) {
  return real2<T>(v.x() * f, v.y() * f);
}

template <typename T>
inline real2<T> vneg(const real2<T> &rhs) {
  return real2<T>(-rhs.x(), -rhs.y());
}

template <typename T>
inline T vlength(const real2<T> &rhs) {
  return std::sqrt(rhs.x() * rhs.x() + rhs.y() * rhs.y());
}

template <typename T>
inline real2<T> vnormalize(const real2<T> &rhs) {
  real2<T> v = rhs;
  T len      = vlength(rhs);
  if (std::fabs(len) > std::numeric_limits<T>::epsilon()) {
    T inv_len = static_cast<T>(1.0) / len;
    v.v[0] *= inv_len;
    v.v[1] *= inv_len;
  }
  return v;
}

using float2 = real2<float>;
using float3 = nanort::real3<float>;
using float4 = real4<float>;

}  // namespace tubular
#endif  // TUBULAR_TYPE_H_
