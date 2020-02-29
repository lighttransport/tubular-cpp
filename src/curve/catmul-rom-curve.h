#ifndef TUBULAR_CATMUL_ROM_CURVE_H_
#define TUBULAR_CATMUL_ROM_CURVE_H_

#include "curve/curve.h"

namespace tubular {

class CubicPoly3D {
public:
  /*
   * Compute coefficients for a cubic polynomial
   *   p(s) = c0 + c1*s + c2*s^2 + c3*s^3
   * such that
   *   p(0) = x0, p(1) = x1
   *  and
   *   p'(0) = t0, p'(1) = t1.
   */
  CubicPoly3D(const float3& v0, const float3& v1, const float3& v2,
              const float3& v3, const float tension = 0.5f) {
    const float3 t0 = tension * (v2 - v0);
    const float3 t1 = tension * (v3 - v1);

    c0_ = v1;
    c1_ = t0;
    c2_ = -3.f * v1 + 3.f * v2 - 2.f * t0 - t1;
    c3_ = 2.f * v1 - 2.f * v2 + t0 + t1;
  }

  float3 Calculate(const float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    return c0_ + c1_ * t + c2_ * t2 + c3_ * t3;
  }

private:
  float3 c0_, c1_, c2_, c3_;
};

class CatmullRomCurve final : public Curve {
public:
  CatmullRomCurve(const std::vector<float3>& points,
                  const std::vector<float>& radiuses, const bool closed = false)
      : Curve(points, radiuses, closed) {}
  ~CatmullRomCurve() override;

protected:
  float3 GetPoint(const float t) const override {
    const std::vector<float3>& points = points_;
    const int l                       = int(points.size());

    const float& point = (l - (closed_ ? 0.f : 1.f)) * t;
    int intPoint       = int(point);
    float weight       = point - intPoint;

    if (closed_) {
      intPoint += intPoint > 0 ? 0
                               : (int(abs(intPoint) / int(points.size())) + 1) *
                                     int(points.size());
    } else if (fabsf(weight) < std::numeric_limits<float>::epsilon() &&
               intPoint == l - 1) {
      intPoint = l - 2;
      weight   = 1;
    }

    float3 tmp, p0, p1, p2, p3;  // 4 points
    if (closed_ || intPoint > 0) {
      p0 = points[size_t((intPoint - 1) % l)];
    } else {
      // extrapolate first point
      tmp = (points[0] - points[1]) + points[0];
      p0  = tmp;
    }

    p1 = points[size_t(intPoint % l)];
    p2 = points[size_t((intPoint + 1) % l)];

    if (closed_ || intPoint + 2 < l) {
      p3 = points[size_t((intPoint + 2) % l)];
    } else {
      // extrapolate last point
      tmp = (points[size_t(l - 1)] - points[size_t(l - 2)]) +
            points[size_t(l - 1)];
      p3 = tmp;
    }

    CubicPoly3D poly(p0, p1, p2, p3);
    return poly.Calculate(weight);
  }

  float GetRadius(const float t) const override {
    assert(points_.size() == radiuses_.size());
    if (radiuses_.size() < 2) {
      // error
      return 0.f;
    }

    const size_t num_segment = radiuses_.size() - 1;

    const float d = 1.f / num_segment;

    const size_t l = size_t(floorf(t / d));
    const size_t r = size_t(ceilf(t / d));

    const float s = (t - d * l) / d;

    return radiuses_.at(l) * (1.f - s) + radiuses_.at(r) * s;
  }
};
}  // namespace tubular

#endif  // TUBULAR_CATMUL_ROM_CURVE_H_
