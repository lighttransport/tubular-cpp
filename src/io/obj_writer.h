#ifndef OBJ_WRITER_H_
#define OBJ_WRITER_H_

#include <string>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#endif

#include "io/tiny_obj_loader.h"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

namespace tubular {

bool WriteObj(const std::string& filename, const tinyobj::attrib_t& attributes,
              const std::vector<tinyobj::shape_t>& shapes,
              const std::vector<tinyobj::material_t>& materials,
              bool coordTransform = false);

}

#endif  // OBJ_WRITER_H_
