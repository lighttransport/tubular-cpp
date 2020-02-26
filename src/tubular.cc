#include "tubular.h"

#include <math.h>

#include <random>

#include "curve/catmul-rom-curve.h"
#include "io/load_xpd.h"
#include "io/obj_writer.h"
#include "logger/logger.h"

namespace tubular {

const float PI2 = kPi * 2.f;

static void CopyFloat2ToFloatArray(const std::vector<float2>& src,
                                   std::vector<float>* out) {
  out->clear();
  out->reserve(src.size() * 2);
  for (auto& v : src) {
    out->emplace_back(v.x());
    out->emplace_back(v.y());
  }
}
static void CopyFloat3ToFloatArray(const std::vector<float3>& src,
                                   std::vector<float>* out) {
  out->clear();
  out->reserve(src.size() * 4);
  for (auto& v : src) {
    out->emplace_back(v.x());
    out->emplace_back(v.y());
    out->emplace_back(v.z());
    out->emplace_back(1.f);
  }
}
static void CopyFloat4ToFloatArray(const std::vector<float4>& src,
                                   std::vector<float>* out) {
  out->clear();
  out->reserve(src.size() * 4);
  for (auto& v : src) {
    out->emplace_back(v.x());
    out->emplace_back(v.y());
    out->emplace_back(v.z());
    out->emplace_back(v.w());
  }
}

TriangleMesh BuildTriangleMesh(const Curve* curve, const int tubularSegments,
                               const float radius, const int radialSegments,
                               const bool closed) {
  std::vector<float3> vertices;
  std::vector<float3> normals;
  std::vector<float4> tangents;
  std::vector<float2> uvs;
  std::vector<int> indices;

  const std::vector<FrenetFrame>& frames =
      curve->ComputeFrenetFrames(tubularSegments, closed);

  for (int i = 0; i < tubularSegments; i++) {
    GenerateSegment(curve, frames, tubularSegments, radius, radialSegments, i,
                    &vertices, &normals, &tangents);
  }
  GenerateSegment(curve, frames, tubularSegments, radius, radialSegments,
                  (!closed) ? tubularSegments : 0, &vertices, &normals,
                  &tangents);

  for (int i = 0; i <= tubularSegments; i++) {
    for (int j = 0; j <= radialSegments; j++) {
      const float u = 1.f * j / radialSegments;
      const float v = 1.f * i / tubularSegments;
      uvs.emplace_back(u, v);
    }
  }

  for (int j = 1; j <= tubularSegments; j++) {
    for (int i = 1; i <= radialSegments; i++) {
      const int a = (radialSegments + 1) * (j - 1) + (i - 1);
      const int b = (radialSegments + 1) * j + (i - 1);
      const int c = (radialSegments + 1) * j + i;
      const int d = (radialSegments + 1) * (j - 1) + i;

      // faces
      indices.emplace_back(a);
      indices.emplace_back(d);
      indices.emplace_back(b);
      indices.emplace_back(b);
      indices.emplace_back(d);
      indices.emplace_back(c);
    }
  }

  TriangleMesh mesh;
  CopyFloat3ToFloatArray(vertices, &(mesh.vertices));
  CopyFloat3ToFloatArray(normals, &(mesh.normals));
  CopyFloat4ToFloatArray(tangents, &(mesh.tangents));
  CopyFloat2ToFloatArray(uvs, &(mesh.uvs));
  mesh.indices.clear();
  mesh.indices.reserve(indices.size());
  for (auto& v : indices) {
    mesh.indices.emplace_back(v);
  }

  return mesh;
}

void GenerateSegment(const Curve* curve, const std::vector<FrenetFrame>& frames,
                     const int tubularSegments, const float radius,
                     const int radialSegments, const int i,
                     std::vector<float3>* vertices,
                     std::vector<float3>* normals,
                     std::vector<float4>* tangents) {
  const float u         = 1.f * i / tubularSegments;
  const float3 p        = curve->GetPointAt(u);
  const FrenetFrame& fr = frames[size_t(i)];

  const float3 N = fr.Normal();
  const float3 B = fr.Binormal();

  for (int j = 0; j <= radialSegments; j++) {
    const float v    = 1.f * j / radialSegments * PI2;
    const float sin_ = sin(v);
    const float cos_ = cos(v);

    const float3 normal = vnormalize(cos_ * N + sin_ * B);
    vertices->emplace_back(p + radius * normal);
    normals->emplace_back(normal);

    const float3 tangent = fr.Tangent();
    tangents->emplace_back(tangent.x(), tangent.y(), tangent.z(), 0.f);
  }
}

tinyobj::shape_t CombineShapes(const std::vector<tinyobj::shape_t>& shapes) {
  tinyobj::shape_t shape_out;
  shape_out.name = "cage_polygon";

  for (const auto& shape : shapes) {
    std::copy(shape.mesh.indices.begin(), shape.mesh.indices.end(),
              std::back_inserter(shape_out.mesh.indices));

    std::copy(shape.mesh.num_face_vertices.begin(),
              shape.mesh.num_face_vertices.end(),
              std::back_inserter(shape_out.mesh.num_face_vertices));

    std::copy(shape.mesh.material_ids.begin(), shape.mesh.material_ids.end(),
              std::back_inserter(shape_out.mesh.material_ids));

    std::copy(shape.mesh.smoothing_group_ids.begin(),
              shape.mesh.smoothing_group_ids.end(),
              std::back_inserter(shape_out.mesh.smoothing_group_ids));

    std::copy(shape.mesh.tags.begin(), shape.mesh.tags.end(),
              std::back_inserter(shape_out.mesh.tags));
  }

  return shape_out;
}

static std::pair<double, double> SpreadTileSize(const uint32_t n,
                                                const double r) {
  const auto LowerBound = [](double s, double t,
                             const std::function<bool(const double)> C) {
    while (std::abs(t - s) >= 1e-9) {
      const double m = (t + s) * 0.5;
      (C(m) ? s : t) = m;
    }

    return s;
  };

  const double tmp = LowerBound(0.0, 1.0, [&n, &r](const double a) {
    return int64_t(1.0 / a) * int64_t(1.0 / (a * r)) >= int64_t(n);
  });

  return std::make_pair(tmp, tmp * r);
}

static void CombineUV(const tinyobj::attrib_t& attrib,
                      const std::vector<tinyobj::shape_t>& shapes,
                      const double tile_ratio, tinyobj::attrib_t* attrib_out,
                      std::vector<tinyobj::shape_t>* shapes_out) {
  // copy shapes //
  (*shapes_out) = shapes;
  /////////////////

  std::vector<uint32_t> new_vertex_indices(attrib.vertices.size(),
                                           uint32_t(-1));
  std::vector<uint32_t> new_normal_indices(attrib.normals.size(), uint32_t(-1));
  std::vector<uint32_t> new_texcoord_indices;

  std::vector<float> new_vertices;
  std::vector<float> new_normals;
  std::vector<float> new_texcoords;

  const size_t n = shapes.size();
  const std::pair<double, double> tile =
      SpreadTileSize(uint32_t(n), tile_ratio);
  const uint32_t w = uint32_t(1.0 / tile.first);
  const uint32_t h = uint32_t(1.0 / tile.second);

  if (w * h < n) {
    RTLOG_ERROR("w {}  h {}  w * h {}  n {}", w, h, w * h, n);
    throw std::runtime_error("error");
  }
  RTLOG_INFO("w {}  h {}  w * h {}  n {}", w, h, w * h, n);

  const double kShrinkPercent = 90.0;
  const double kShrink        = kShrinkPercent * 0.01;
  const double tile_scale_u   = (1.0 / w) * kShrink;
  const double tile_scale_v   = (1.0 / h) * kShrink;
  const double kMarginU       = ((1.0 / w) - tile_scale_u) * 0.5;
  const double kMarginV       = ((1.0 / h) - tile_scale_v) * 0.5;

  for (size_t i = 0; i < n; i++) {
    auto& shape_out = (*shapes_out)[i];
    const size_t x  = i % w;
    const size_t y  = i / w;

    const double u0 = double(x) / w;
    const double v0 = double(y) / h;

    for (auto& index : shape_out.mesh.indices) {
      if (new_vertex_indices[size_t(index.vertex_index)] == uint32_t(-1)) {
        new_vertex_indices[size_t(index.vertex_index)] =
            uint32_t(new_vertices.size() / 3);
        new_vertices.emplace_back(
            attrib.vertices[size_t(index.vertex_index * 3 + 0)]);
        new_vertices.emplace_back(
            attrib.vertices[size_t(index.vertex_index * 3 + 1)]);
        new_vertices.emplace_back(
            attrib.vertices[size_t(index.vertex_index * 3 + 2)]);
      }
      index.vertex_index = int(new_vertex_indices[size_t(index.vertex_index)]);

      if (new_normal_indices[size_t(index.normal_index)] == uint32_t(-1)) {
        new_normal_indices[size_t(index.normal_index)] =
            uint32_t(new_normals.size() / 3);
        new_normals.emplace_back(
            attrib.normals[size_t(index.normal_index * 3 + 0)]);
        new_normals.emplace_back(
            attrib.normals[size_t(index.normal_index * 3 + 1)]);
        new_normals.emplace_back(
            attrib.normals[size_t(index.normal_index * 3 + 2)]);
      }
      index.normal_index = int(new_normal_indices[size_t(index.normal_index)]);

      {
        const double u =
            double(attrib.texcoords[size_t(index.texcoord_index * 2 + 0)]);
        const double v =
            double(attrib.texcoords[size_t(index.texcoord_index * 2 + 1)]);

        const double u_out = u0 + kMarginU + u * tile_scale_u;
        const double v_out = v0 + kMarginV + v * tile_scale_v;

        new_texcoord_indices.emplace_back(uint32_t(new_texcoords.size() / 2));
        index.texcoord_index = int(new_texcoords.size() / 2);

        new_texcoords.emplace_back(double(u_out));
        new_texcoords.emplace_back(double(v_out));
      }
    }
  }

  {
    {
      attrib_out->vertices  = new_vertices;
      attrib_out->normals   = new_normals;
      attrib_out->texcoords = new_texcoords;
    }
  }
  const tinyobj::shape_t tmp_shape = CombineShapes(*shapes_out);
  shapes_out->clear();
  shapes_out->emplace_back(tmp_shape);

  //return std::make_tuple(attrib_out, shapes_out);
}

static void ToTinyObjMesh(const std::vector<TriangleMesh>& meshes,
                          const bool combine_shapes,
                          tinyobj::attrib_t* attributes,
                          std::vector<tinyobj::shape_t>* shapes) {
  std::vector<int> vertex_offset;
  std::vector<int> normal_offset;
  std::vector<int> texcoord_offset;

  for (const auto& mesh : meshes) {
    vertex_offset.emplace_back(attributes->vertices.size() / 3);
    normal_offset.emplace_back(attributes->normals.size() / 3);
    texcoord_offset.emplace_back(attributes->texcoords.size() / 2);

    const size_t num_vertices = mesh.vertices.size() / 4;
    for (size_t v_id = 0; v_id < num_vertices; ++v_id) {
      attributes->vertices.emplace_back(mesh.vertices[v_id * 4 + 0]);
      attributes->vertices.emplace_back(mesh.vertices[v_id * 4 + 1]);
      attributes->vertices.emplace_back(mesh.vertices[v_id * 4 + 2]);
      // RTLOG_INFO("{} {} {}", mesh.vertices[v_id * 4 + 0],
      //            mesh.vertices[v_id * 4 + 1], mesh.vertices[v_id * 4 + 2]);
    }

    const size_t num_normals = mesh.normals.size() / 4;
    for (size_t n_id = 0; n_id < num_normals; ++n_id) {
      attributes->normals.emplace_back(mesh.normals[n_id * 4 + 0]);
      attributes->normals.emplace_back(mesh.normals[n_id * 4 + 1]);
      attributes->normals.emplace_back(mesh.normals[n_id * 4 + 2]);
    }
    // TODO tangent

    const size_t num_texcorrds = mesh.uvs.size() / 2;
    for (size_t tc_id = 0; tc_id < num_texcorrds; ++tc_id) {
      attributes->texcoords.emplace_back(mesh.uvs[tc_id * 2 + 0]);
      attributes->texcoords.emplace_back(mesh.uvs[tc_id * 2 + 1]);
    }
  }

  if (combine_shapes) {
    tinyobj::shape_t shape;
    int face_cnt = 0;
    for (size_t mesh_id = 0; mesh_id < meshes.size(); ++mesh_id) {
      const auto& mesh = meshes[mesh_id];

      const size_t num_faces = mesh.indices.size() / 3;
      for (size_t f_id = 0; f_id < num_faces; ++f_id) {
        face_cnt++;
        shape.mesh.num_face_vertices.emplace_back(3);
        for (size_t i = 0; i < 3; ++i) {
          tinyobj::index_t id;
          id.vertex_index =
              int(mesh.indices[f_id * 3l + i]) + vertex_offset[mesh_id];
          id.normal_index =
              int(mesh.indices[f_id * 3l + i]) + normal_offset[mesh_id];
          id.texcoord_index =
              int(mesh.indices[f_id * 3l + i]) + texcoord_offset[mesh_id];
          shape.mesh.indices.emplace_back(id);
        }
      }
    }
    shape.mesh.material_ids.clear();
    shape.mesh.material_ids.resize(size_t(face_cnt), 0);
    shapes->emplace_back(shape);
  } else {
    for (size_t mesh_id = 0; mesh_id < meshes.size(); ++mesh_id) {
      tinyobj::shape_t shape;
      const auto& mesh       = meshes[mesh_id];
      const size_t num_faces = mesh.indices.size() / 3;

      for (size_t f_id = 0; f_id < num_faces; ++f_id) {
        shape.mesh.num_face_vertices.emplace_back(3);
        for (size_t i = 0; i < 3; ++i) {
          tinyobj::index_t id;
          id.vertex_index =
              int(mesh.indices[f_id * 3l + i]) + vertex_offset[mesh_id];
          id.normal_index =
              int(mesh.indices[f_id * 3l + i]) + normal_offset[mesh_id];
          id.texcoord_index =
              int(mesh.indices[f_id * 3l + i]) + texcoord_offset[mesh_id];
          shape.mesh.indices.emplace_back(id);
        }
      }
      shape.mesh.material_ids.clear();
      shape.mesh.material_ids.resize(num_faces, 0);
      shapes->emplace_back(shape);
    }
  }
}

static std::vector<std::vector<bool>> RandomSelection(
    const std::vector<std::vector<std::vector<float>>>& vertices,
    const uint32_t max_strands) {
  std::vector<bool> bl;
  for (const auto& v : vertices) {
    for (const auto& vv : v) {
      (void)vv;
      bl.push_back(true);
    }
  }
  if (max_strands > 0 && bl.size() > max_strands) {
    std::fill(bl.begin(), bl.end(), false);
    std::fill(bl.begin(), bl.begin() + max_strands, true);

    RTLOG_INFO("random shuffle");
    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());

    std::shuffle(bl.begin(), bl.end(), engine);
  }
  std::vector<std::vector<bool>> ret;
  size_t cnt = 0;
  for (const auto& v : vertices) {
    ret.emplace_back();
    for (const auto& vv : v) {
      (void)vv;
      ret.back().push_back(bl[cnt]);
      cnt++;
    }
  }
  return ret;
}

void XpdToObj(const TubularConfig& config) {
  const std::string& xpd_filepath = config.xpd_filepath;

  std::vector<uint32_t> face_ids;
  std::vector<std::vector<std::vector<float>>> curve_vertices;
  std::vector<std::vector<std::vector<float>>> curve_thicknesses;

  LoadXPD(xpd_filepath, face_ids, &curve_vertices, &curve_thicknesses);

  assert(curve_vertices.size() == curve_thicknesses.size());

  std::vector<std::vector<bool>> use_or_not =
      RandomSelection(curve_vertices, config.max_strands);

  std::vector<TriangleMesh> meshes;
  for (size_t bundle_id = 0; bundle_id < curve_vertices.size(); ++bundle_id) {
    const auto& bundle = curve_vertices[bundle_id];
    for (size_t strand_id = 0; strand_id < bundle.size(); ++strand_id) {
      const std::vector<float>& strand = bundle[strand_id];

      if (strand.empty()) {
        RTLOG_WARN("empty strand");
        continue;
      }
      if (!use_or_not.at(bundle_id).at(strand_id)) {
        continue;
      }

      std::vector<float3> tmp;
      tmp.reserve(strand.size() / 3);
      assert(strand.size() % 3 == 0);
      for (size_t i = 0; i < strand.size() / 3; i++) {
        tmp.emplace_back(strand.data() + i * 3);
      }

      CatmullRomCurve catmul_rom_curve(tmp);
      const int tubular_segments =
          std::min(int((tmp.size() - 1) * 4), config.max_segments);
      const float radius       = config.radius;
      const int radialSegments = config.radial_segments;
      const bool closed        = false;

      TriangleMesh mesh = BuildTriangleMesh(&catmul_rom_curve, tubular_segments,
                                            radius, radialSegments, closed);
      mesh.name = std::to_string(bundle_id) + "-" + std::to_string(strand_id);
      meshes.emplace_back(mesh);
    }
  }

  tinyobj::attrib_t attributes;
  std::vector<tinyobj::shape_t> shapes;
  ToTinyObjMesh(meshes, false, &attributes, &shapes);
  {
    tinyobj::attrib_t tmp_attributes;
    std::vector<tinyobj::shape_t> tmp_shapes;
    CombineUV(attributes, shapes, config.tile_ratio, &tmp_attributes,
              &tmp_shapes);
    attributes = tmp_attributes;
    shapes     = tmp_shapes;
  }

  std::vector<tinyobj::material_t> output_materials;
  output_materials.emplace_back();
  output_materials.back().name = "bake";
  // output_materials.back().diffuse_texname = "output.exr";

  WriteObj(config.obj_filepath, attributes, shapes, output_materials);
}

}  // namespace tubular
