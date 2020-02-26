#ifndef EXAMPLE_LOAD_XPD_H_
#define EXAMPLE_LOAD_XPD_H_

#include <string>
#include <vector>
namespace tubular {

bool LoadXPD(
    const std::string &xpd_filename, const std::vector<uint32_t> &face_ids,
    std::vector<std::vector<std::vector<float>>> *catmull_rom_curve_vertices,
    std::vector<std::vector<std::vector<float>>>
        *catmull_rom_curve_thicknesses);
}

#endif  // EXAMPLE_LOAD_XPD_H_
