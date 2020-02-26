#ifndef TUBULAR_CONFIG_LOADER_H_
#define TUBULAR_CONFIG_LOADER_H_
#include <string>

#include "tubular-config.h"

namespace tubular {
bool LoadConfigFromJson(const std::string& filepath, TubularConfig* config);
}  // namespace tubular

#endif  // TUBULAR_CONFIG_LOADER_H_
