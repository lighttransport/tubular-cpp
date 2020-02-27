#include "load_xpd.h"

#include <cassert>
#include <cstring>
#include <set>

#include "logger/logger.h"
#include "tiny_xpd.h"

namespace tubular {

static void GetPrimData(const tiny_xpd::XPDHeader &xpd,
                        const std::vector<uint8_t> &xpd_data, size_t face_idx,
                        size_t block_idx, std::vector<float> *prims) {
  prims->clear();

  uint32_t num_prims = xpd.numPrims[face_idx];
  if (num_prims == 0) {
    return;
  }

  // prim_count = num_prims * sum(xpd.primSize[])
  size_t prim_count = 0;
  for (size_t p = 0; p < xpd.primSize.size(); p++) {
    prim_count += xpd.primSize[p];
  }

  prim_count *= num_prims;

  // Pritive value is always float.
  const size_t num_bytes = sizeof(float) * prim_count;

  const size_t src_offset =
      xpd.blockPosition[face_idx * xpd.numBlocks + block_idx];
  std::vector<float> buffer;
  buffer.resize(prim_count);
  memcpy(buffer.data(), xpd_data.data() + src_offset, num_bytes);

  prims->insert(prims->end(), buffer.begin(), buffer.end());
}

bool LoadXPD(
    const std::string &xpd_filename, const std::vector<uint32_t> &face_ids,
    std::vector<std::vector<std::vector<float>>> *catmull_rom_curve_vertices,
    std::vector<std::vector<std::vector<float>>>
        *catmull_rom_curve_thicknesses) {
  std::string err;
  tiny_xpd::XPDHeader xpd_header;
  std::vector<uint8_t> xpd_data;

  if (!tiny_xpd::ParseXPDFromFile(xpd_filename, &xpd_header, &xpd_data, &err)) {
    if (!err.empty()) {
      RTLOG_ERROR("XPD: Parse error message: {}", err);
    }

    RTLOG_ERROR("XPD: Failed to parse XPD file {}", xpd_filename);
    return false;
  }

  if (xpd_header.primType != tiny_xpd::Xpd::PrimType::Spline) {
    RTLOG_ERROR("XPD primType is not 1(Spline). Got {}", xpd_header.primType);
    return false;
  }

  const int max_face_id =
      *std::max_element(xpd_header.faceid.begin(), xpd_header.faceid.end());

  catmull_rom_curve_vertices->clear();
  catmull_rom_curve_thicknesses->clear();

  catmull_rom_curve_vertices->resize(size_t(max_face_id + 1));
  catmull_rom_curve_thicknesses->resize(size_t(max_face_id + 1));

  std::set<uint32_t> face_indices;
  for (const auto v : face_ids) {
    face_indices.insert(v);
  }

  // foreach face.
  for (size_t f = 0; f < xpd_header.numFaces; f++) {
    if (xpd_header.numPrims[f] < 1) continue;
    // foreach block
    for (size_t b = 0; b < xpd_header.numBlocks; b++) {
      std::vector<float> prims;
      GetPrimData(xpd_header, xpd_data, f, b, &prims);

      assert(xpd_header.primSize[b] == 3 + 3 * xpd_header.numCVs + 7);

      if (prims.size() == 0) continue;

      for (size_t p_id = 0; p_id < xpd_header.numPrims[f]; p_id++) {
        // 3: 0, 1, 2 => face id, u, v
        // 3 * CV: xyz * CV
        // 7: length, width, taper, taperStart, width vector(xyz)
        int offset = int(p_id * (3 + 3 * xpd_header.numCVs + 7));

        const int face_id = int(prims[size_t(offset)]);
        const float u     = int(prims[size_t(offset + 1)]);
        const float v     = int(prims[size_t(offset + 2)]);

        const size_t cv_offset   = size_t(offset + 3);
        const size_t attr_offset = cv_offset + 3 * xpd_header.numCVs;

        std::vector<float> cvs;  // xyz * numCVs

        for (size_t i = 0; i < (attr_offset - cv_offset); i++) {
          cvs.push_back(prims[i + cv_offset]);
        }
        assert(cvs.size() % 3 == 0);

        const float length         = prims[attr_offset + 0];
        const float width          = prims[attr_offset + 1];
        const float taper          = prims[attr_offset + 2];
        const float taper_start    = prims[attr_offset + 3];
        const float width_vector_x = prims[attr_offset + 4];
        const float width_vector_y = prims[attr_offset + 5];
        const float width_vector_z = prims[attr_offset + 6];

        // Convert to buffer for ray tracing.

        std::vector<float> cv_radiuss;
        for (size_t i = 0; i < cvs.size() / 3; i++) {
          cv_radiuss.push_back(width);
        }

        if (!face_indices.empty() &&
            face_indices.find(uint32_t(xpd_header.faceid[f])) ==
                face_indices.end()) {
          continue;
        }

        (*catmull_rom_curve_vertices)[size_t(xpd_header.faceid[f])]
            .emplace_back(cvs);
        (*catmull_rom_curve_thicknesses)[size_t(xpd_header.faceid[f])]
            .emplace_back(cv_radiuss);

        // TODO(LTE):
        (void)face_id;
        (void)u;
        (void)v;

        (void)length;
        (void)taper;
        (void)taper_start;

        (void)width_vector_x;
        (void)width_vector_y;
        (void)width_vector_z;
      }
    }
  }

  return true;
}

}  // namespace tubular
