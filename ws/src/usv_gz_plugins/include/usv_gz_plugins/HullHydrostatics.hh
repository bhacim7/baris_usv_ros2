// Hydrostatic force on a closed hull mesh: the pressure of the water integrated over every
// triangle (or part of a triangle) below the surface.
//
// Kept free of Gazebo so it can be unit tested on its own; the HullBuoyancy system only
// feeds it world positions and water depths.
//
// Pressure at a point is rho * g * depth, depth = water surface height above the point.
// It is linear over a triangle whose corner depths are known, so the force on a triangle
// is exact: F = -mean(p) * area * n (n = outward unit normal), applied at the centre of
// pressure. A triangle cut by the surface is clipped at depth 0. The open waterline cut
// carries zero pressure, so for a closed mesh in still water the result equals
// rho * g * submerged volume, acting through the centre of buoyancy (divergence theorem).

#ifndef USV_GZ_PLUGINS__HULL_HYDROSTATICS_HH_
#define USV_GZ_PLUGINS__HULL_HYDROSTATICS_HH_

#include <array>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>

namespace usv_gz_plugins
{

struct Mesh
{
  std::vector<gz::math::Vector3d> vertices;
  /// Vertex indices, counter-clockwise seen from outside (outward normal).
  std::vector<std::array<std::size_t, 3>> triangles;
};

struct Wrench
{
  gz::math::Vector3d force{0, 0, 0};
  /// About the reference point given to Integrate().
  gz::math::Vector3d torque{0, 0, 0};
  double wettedArea{0};
};

/// Read the vertices and faces of a Wavefront OBJ file. Normals and texture coordinates
/// are ignored; polygons with more than three corners are split into a fan.
inline Mesh LoadObj(const std::string & _path)
{
  std::ifstream file(_path);
  if (!file) {
    throw std::runtime_error("cannot open mesh file " + _path);
  }
  Mesh mesh;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream in(line);
    std::string tag;
    in >> tag;
    if (tag == "v") {
      double x, y, z;
      in >> x >> y >> z;
      mesh.vertices.emplace_back(x, y, z);
    } else if (tag == "f") {
      std::vector<std::size_t> corners;
      std::string token;
      while (in >> token) {
        // "a", "a/b", "a//n" or "a/b/n": the vertex index is before the first slash
        long index = std::stol(token.substr(0, token.find('/')));
        if (index < 0) {
          index += static_cast<long>(mesh.vertices.size()) + 1;   // relative index
        }
        corners.push_back(static_cast<std::size_t>(index - 1));
      }
      for (std::size_t k = 1; k + 1 < corners.size(); ++k) {
        mesh.triangles.push_back({corners[0], corners[k], corners[k + 1]});
      }
    }
  }
  for (const auto & t : mesh.triangles) {
    for (std::size_t i : t) {
      if (i >= mesh.vertices.size()) {
        throw std::runtime_error("face refers to a missing vertex in " + _path);
      }
    }
  }
  if (mesh.triangles.empty()) {
    throw std::runtime_error("no faces in mesh file " + _path);
  }
  return mesh;
}

namespace detail
{

/// Force and torque of the pressure on one fully wetted triangle (all depths >= 0).
inline void AddTriangle(
  const gz::math::Vector3d & _a, const gz::math::Vector3d & _b,
  const gz::math::Vector3d & _c, double _da, double _db, double _dc,
  const gz::math::Vector3d & _ref, double _rhoG, Wrench & _out)
{
  // Area vector: |S| = area, direction = outward normal (from the winding)
  const gz::math::Vector3d s = 0.5 * (_b - _a).Cross(_c - _a);
  const double depthSum = _da + _db + _dc;
  if (depthSum <= 0.0) {
    return;
  }
  // Mean pressure over a triangle with linear pressure = mean of the corner pressures
  const gz::math::Vector3d force = -_rhoG * (depthSum / 3.0) * s;
  // Centre of pressure of a linear pressure field over a triangle:
  //   integral(p x dA) = A/12 * (sum(p) * sum(x) + sum(p_i x_i)),  integral(p dA) = A * sum(p) / 3
  const gz::math::Vector3d centre =
    (depthSum * (_a + _b + _c) + _da * _a + _db * _b + _dc * _c) / (4.0 * depthSum);
  _out.force += force;
  _out.torque += (centre - _ref).Cross(force);
  _out.wettedArea += s.Length();
}

}  // namespace detail

/// Pressure force and torque on a closed mesh.
/// \param[in] _points Vertex positions in the world frame (same order as _mesh.vertices).
/// \param[in] _depths Water depth at each vertex: surface height minus vertex height.
///   Positive under water.
/// \param[in] _ref Point the torque is taken about (world frame).
/// \param[in] _rho Fluid density (kg/m^3).
/// \param[in] _g Gravity magnitude (m/s^2).
inline Wrench Integrate(
  const Mesh & _mesh, const std::vector<gz::math::Vector3d> & _points,
  const std::vector<double> & _depths, const gz::math::Vector3d & _ref,
  double _rho, double _g)
{
  Wrench out;
  const double rhoG = _rho * _g;
  for (const auto & t : _mesh.triangles) {
    const double d[3] = {_depths[t[0]], _depths[t[1]], _depths[t[2]]};
    if (d[0] <= 0.0 && d[1] <= 0.0 && d[2] <= 0.0) {
      continue;                                   // dry
    }
    const gz::math::Vector3d p[3] = {_points[t[0]], _points[t[1]], _points[t[2]]};
    if (d[0] >= 0.0 && d[1] >= 0.0 && d[2] >= 0.0) {
      detail::AddTriangle(p[0], p[1], p[2], d[0], d[1], d[2], _ref, rhoG, out);
      continue;
    }
    // Cut by the surface: keep the wet part (depth >= 0), walking the corners in order so
    // the winding, and with it the outward normal, is preserved. Result: 3 or 4 corners.
    gz::math::Vector3d poly[4];
    double polyDepth[4];
    int n = 0;
    for (int i = 0; i < 3; ++i) {
      const int j = (i + 1) % 3;
      if (d[i] >= 0.0) {
        poly[n] = p[i];
        polyDepth[n] = d[i];
        ++n;
      }
      if ((d[i] >= 0.0) != (d[j] >= 0.0)) {
        const double f = d[i] / (d[i] - d[j]);  // where the depth crosses zero
        poly[n] = p[i] + f * (p[j] - p[i]);
        polyDepth[n] = 0.0;
        ++n;
      }
    }
    for (int k = 1; k + 1 < n; ++k) {
      detail::AddTriangle(
        poly[0], poly[k], poly[k + 1], polyDepth[0], polyDepth[k], polyDepth[k + 1],
        _ref, rhoG, out);
    }
  }
  return out;
}

}  // namespace usv_gz_plugins

#endif  // USV_GZ_PLUGINS__HULL_HYDROSTATICS_HH_
