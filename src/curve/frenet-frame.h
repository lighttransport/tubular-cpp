#ifndef TUBULAR_FRENET_FRAME_H_
#define TUBULAR_FRENET_FRAME_H_

#include "type.h"

namespace tubular {

class FrenetFrame {
public:
  FrenetFrame(const float3& tangent, const float3& normal,
              const float3& binormal)
      : tangent_(tangent), normal_(normal), binormal_(binormal) {}

  float3 Tangent() const { return tangent_; }
  float3 Normal() const { return normal_; }
  float3 Binormal() const { return binormal_; }

private:
  float3 tangent_;
  float3 normal_;
  float3 binormal_;
};

}  // namespace tubular

#endif  //  TUBULAR_FRENET_FRAME_H_
