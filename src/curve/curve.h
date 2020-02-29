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
        const bool closed = false) {
    points_   = points;
    radiuses_ = radiuses;
    closed_   = closed;
    assert(points_.size() == radiuses_.size());
  }
  virtual ~Curve();

  float3 GetPointAt(const float u) const {
    const float t = GetUtoTmapping(u);
    return GetPoint(t);
  }

  float GetRadiusAt(const float u) const {
    const float t = GetUtoTmapping(u);
    return GetRadius(t);
  }

  float3 GetTangentAt(const float u) const {
    const float t = GetUtoTmapping(u);
    return GetTangent(t);
  }

  // TODO return value. use pointer?
  std::vector<float> GetLengths(int divisions = -1) const {
    if (divisions < 0) {
      divisions = 200;
    }

    if (!cacheArcLengths_.empty() /* TODO is this ok?*/ &&
        (int(cacheArcLengths_.size()) == divisions + 1) && !needsUpdate_) {
      return cacheArcLengths_;
    }

    needsUpdate_ = false;

    std::vector<float> cache(size_t(divisions + 1));

    float3 current, last = GetPoint(0.f);

    cache[0] = 0.f;

    float sum = 0.f;
    for (int p = 1; p <= divisions; p++) {
      current = GetPoint(1.f * p / divisions);
      sum += vlength(current - last);
      cache[size_t(p)] = sum;
      last             = current;
    }

    cacheArcLengths_ = cache;
    return cache;
  }

  std::vector<FrenetFrame> ComputeFrenetFrames(const int segments,
                                               const bool closed) const {
    float3 normal(0.f);

    std::vector<float3> tangents(size_t(segments + 1));
    std::vector<float3> normals(size_t(segments + 1));
    std::vector<float3> binormals(size_t(segments + 1));

    float u, theta;

    // compute the tangent vectors for each segment on the curve
    for (int i = 0; i <= segments; i++) {
      u                   = (1.f * i) / segments;
      tangents[size_t(i)] = vnormalized(GetTangentAt(u));
    }

    // select an initial normal vector perpendicular to the first tangent
    // vector, and in the direction of the minimum tangent xyz component

    normals[0]   = float3(0.f);
    binormals[0] = float3(0.f);

    float min      = std::numeric_limits<float>::max();
    const float tx = fabsf(tangents[0].x());
    const float ty = fabsf(tangents[0].y());
    const float tz = fabsf(tangents[0].z());
    if (tx <= min) {
      min    = tx;
      normal = float3(1, 0, 0);
    }
    if (ty <= min) {
      min    = ty;
      normal = float3(0, 1, 0);
    }
    if (tz <= min) {
      normal = float3(0, 0, 1);
    }

    const float3 vec = vnormalized(vcross(tangents[0], normal));
    normals[0]       = vcross(tangents[0], vec);
    binormals[0]     = vcross(tangents[0], normals[0]);

    // compute the slowly-varying normal and binormal vectors for each segment
    // on the curve

    for (int i = 1; i <= segments; i++) {
      // copy previous
      normals[size_t(i)]   = normals[size_t(i - 1)];
      binormals[size_t(i)] = binormals[size_t(i - 1)];

      // Rotation axis
      float3 axis = vcross(tangents[size_t(i - 1)], tangents[size_t(i)]);
      if (vlength(axis) > std::numeric_limits<float>::epsilon()) {
        axis = vnormalized(axis);

        const float dot = vdot(tangents[size_t(i - 1)], tangents[size_t(i)]);

        // clamp for floating pt errors
        theta = acosf(Clamp(dot, -1.f, 1.f));  // rad

        // TODO check
        normals[size_t(i)] =
            ArbitraryAxisRotation(axis, theta, normals[size_t(i)]);
      }

      binormals[size_t(i)] =
          vnormalized(vcross(tangents[size_t(i)], normals[size_t(i)]));
    }

    // if the curve is closed, postprocess the vectors so the first and last
    // normal vectors are the same

    if (closed) {
      theta = acosf(Clamp(vdot(normals[size_t(0)], normals[size_t(segments)]),
                          -1.f, 1.f));  // rad
      theta /= segments;

      if (vdot(tangents[size_t(0)],
               vcross(normals[size_t(0)], normals[size_t(segments)])) > 0.f) {
        theta = -theta;
      }

      for (int i = 1; i <= segments; i++) {
        normals[size_t(i)] = ArbitraryAxisRotation(
            tangents[size_t(i)], theta * i, normals[size_t(i)]);
        // normals[i] =
        //     (Quaternion.AngleAxis(Mathf.Deg2Rad * theta * i, tangents[i])
        //     *
        //      normals[i]);
        //  TODO Deg2Rad?
        binormals[size_t(i)] = vcross(tangents[size_t(i)], normals[size_t(i)]);
      }
    }

    std::vector<FrenetFrame> frames;
    int n = int(tangents.size());
    for (int i = 0; i < n; i++) {
      FrenetFrame frame(tangents[size_t(i)], normals[size_t(i)],
                        binormals[size_t(i)]);
      frames.emplace_back(frame);
    }
    return frames;
  }

protected:
  virtual float3 GetPoint(const float t) const = 0;
  virtual float GetRadius(const float t) const = 0;

  virtual float3 GetTangent(const float t) const {
    const float delta = 0.001f;

    float t1 = t - delta;
    float t2 = t + delta;

    // Capping in case of danger
    if (t1 < 0.f) t1 = 0.f;
    if (t2 > 1.f) t2 = 1.f;

    const float3 pt1 = GetPoint(t1);
    const float3 pt2 = GetPoint(t2);
    return vnormalized(pt2 - pt1);
  }

  // Given u ( 0 .. 1 ), get a t to find p. This gives you points which are
  // equidistant
  float GetUtoTmapping(const float u) const {
    const std::vector<float>& arcLengths = GetLengths();
    int i = 0, il = int(arcLengths.size());

    // The targeted u distance value to get
    const float targetArcLength = u * arcLengths[size_t(il - 1)];

    // binary search for the index with largest value smaller than target u
    // distance
    int low = 0, high = il - 1;
    float comparison;

    while (low <= high) {
      i          = int(floorf(low + (high - low) / 2.f));  // TODO is this ok ?
      comparison = arcLengths[size_t(i)] - targetArcLength;

      if (comparison < 0.f) {
        low = i + 1;
      } else if (comparison > 0.f) {
        high = i - 1;
      } else {
        high = i;
        break;
      }
    }

    i = high;

    // if (Mathf.Approximately(arcLengths[i], targetArcLength))
    if (fabsf(arcLengths[size_t(i)] - targetArcLength) < kEps /*TODO*/) {
      return 1.f * i / (il - 1);
    }

    // we could get finer grain at lengths, or use simple interpolation between
    // two points

    const float lengthBefore = arcLengths[size_t(i)];
    const float lengthAfter  = arcLengths[size_t(i + 1)];

    const float segmentLength = lengthAfter - lengthBefore;

    // determine where we are between the 'before' and 'after' points

    const float segmentFraction =
        (targetArcLength - lengthBefore) / segmentLength;

    // add that fractional amount to t
    const float t = 1.f * (i + segmentFraction) / (il - 1);

    return t;
  }

  std::vector<float3> points_;
  std::vector<float> radiuses_;
  bool closed_ = false;

  mutable std::vector<float> cacheArcLengths_;  // TODO rename, mutable
  mutable bool needsUpdate_ = false;            // TODO rename, mutable
};

}  // namespace tubular

#endif  // TUBULAR_CURVE_H_
