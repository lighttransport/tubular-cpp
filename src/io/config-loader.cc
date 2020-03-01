#include "config-loader.h"

#include <iostream>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#endif

#include "staticjson/staticjson.hpp"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

namespace staticjson {
template <>
void init(tubular::TubularConfig* config, ObjectHandler* h) {
  h->add_property("xpd_filepath", &(config->xpd_filepath),
                  staticjson::Flags::Optional);
  h->add_property("cyhair_filepath", &(config->cyhair_filepath),
                  staticjson::Flags::Optional);
  h->add_property("obj_filepath", &(config->obj_filepath));
  h->add_property("max_segments",
                  &(config->max_segments));  // TODO(LTE): make this optional?
  h->add_property(
      "radial_segments",
      &(config->radial_segments));  // TODO(LTE): make this optional?

  h->add_property("radius_scale", &(config->radius_scale),
                  staticjson::Flags::Optional);
  h->add_property("user_radius", &(config->user_radius),
                  staticjson::Flags::Optional);

  h->add_property("max_strands", &(config->max_strands),
                  staticjson::Flags::Optional);
  h->add_property("tile_ratio", &(config->tile_ratio),
                  staticjson::Flags::Optional);

  h->add_property("fix_normal", &(config->fix_normal),
                  staticjson::Flags::Optional);

  h->add_property("one_side_plane", &(config->one_side_plane),
                  staticjson::Flags::Optional);
}
}  // namespace staticjson

namespace tubular {
bool LoadConfigFromJson(const std::string& filepath, TubularConfig* config) {
  staticjson::ParseStatus res;
  if (!staticjson::from_json_file(filepath, config, &res)) {
    std::cerr << "Failed to parse JSON.\n";
    std::cerr << res.description() << "\n";
    return false;
  }
  return true;
}
}  // namespace tubular
