#ifndef TUBULAR_TUBULAR_H_
#define TUBULAR_TUBULAR_H_
#include <string>
#include <vector>

#include "curve/curve.h"
#include "curve/frenet-frame.h"
#include "mesh//triangle-mesh.h"
#include "tubular-config.h"
#include "tubular-math.h"

namespace tubular {

TriangleMesh BuildTriangleMesh(const Curve* curve, const int tubularSegments,
                               const int radialSegments, const bool closed);

void GenerateSegment(const Curve* curve, const std::vector<FrenetFrame>& frames,
                     const int tubularSegments, const int radialSegments,
                     const int i, std::vector<float3>* vertices,
                     std::vector<float3>* normals,
                     std::vector<float4>* tangents);

void Tubular(const TubularConfig& config);

}  // namespace tubular
#endif  // TUBULAR_TUBULAR_H_
