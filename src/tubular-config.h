#ifndef TUBULAR_TUBULAR_CONFIG_H_
#define TUBULAR_TUBULAR_CONFIG_H_
#include <stdint.h>

#include <string>

namespace tubular {

struct TubularConfig {
  std::string xpd_filepath    = "";
  std::string cyhair_filepath = "";
  std::string obj_filepath    = "";

  int max_segments    = 10;
  int radial_segments = 4;
  float radius        = 0.01f;

  uint32_t max_strands = 5000;  // 0 -> use all strands

  float tile_ratio = 4.0;
};

}  // namespace tubular
#endif  // TUBULAR_TUBULAR_CONFIG_H_
