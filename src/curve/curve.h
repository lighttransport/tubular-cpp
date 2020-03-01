#ifndef TUBULAR_CURVE_H_
#define TUBULAR_CURVE_H_
#include <vector>

#include "curve/frenet-frame.h"
#include "tubular-math.h"
#include "tubular-util.h"
#include "type.h"

namespace tubular {
class Curve {
public:
  Curve(const std::vector<float3>& points, const std::vector<float>& radiuses,
        const bool closed = false);

  virtual ~Curve();

  float3 GetPointAt(const float u) const;

  float GetRadiusAt(const float u) const;

  float3 GetTangentAt(const float u) const;

  // TODO return value. use pointer?
  std::vector<float> GetLengths(int divisions = -1) const;

  void ComputeFrenetFrames(const int segments, const bool closed,
                           std::vector<FrenetFrame>* frames) const;
  void ComputeFrenetFramesFixNormal(const int segments, const bool closed,
                                    const float3& fix_normal,
                                    std::vector<FrenetFrame>* frames) const;

protected:
  virtual float3 GetPoint(const float t) const = 0;
  virtual float GetRadius(const float t) const = 0;

  virtual float3 GetTangent(const float t) const;

  // Given u ( 0 .. 1 ), get a t to find p. This gives you points which are
  // equidistant
  float GetUtoTmapping(const float u) const;

  std::vector<float3> points_;
  std::vector<float> radiuses_;
  bool closed_ = false;

  mutable std::vector<float> cacheArcLengths_;  // TODO rename, mutable
  mutable bool needsUpdate_ = false;            // TODO rename, mutable
};

}  // namespace tubular

#endif  // TUBULAR_CURVE_H_
