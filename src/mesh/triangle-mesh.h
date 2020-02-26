#ifndef TUBULAR_TRIANGLE_MESH_H_
#define TUBULAR_TRIANGLE_MESH_H_

#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

namespace tubular {

struct TriangleMesh {
  TriangleMesh();

  std::vector<float> vertices;  // (3(xyz) + 1(w)) * num vertices
  std::vector<float> normals;   // (3(xyz) + 1(w)) * num normals
  std::vector<float> tangents;  // 4(xyzw) * num tangent
  std::vector<float> uvs;       // 2(uv) * num uv // TODO rename texcoord

  std::vector<uint32_t> indices;  // 3(v1v2v3) * num_faces_

  std::string name;
};

}  // namespace tubular

#endif  // PBRLAB_TRIANGLE_MESH_H_
