// Checks of the pressure integration against shapes whose answer is known by hand.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "usv_gz_plugins/HullHydrostatics.hh"

using gz::math::Vector3d;
using usv_gz_plugins::Integrate;
using usv_gz_plugins::LoadObj;
using usv_gz_plugins::Mesh;
using usv_gz_plugins::Wrench;

namespace
{

constexpr double kRho = 1000.0;
constexpr double kG = 10.0;

/// Axis-aligned box centred on _c, outward winding.
Mesh Box(const Vector3d & _c, double _lx, double _ly, double _lz)
{
  Mesh m;
  for (int i = 0; i < 8; ++i) {
    m.vertices.emplace_back(
      _c.X() + ((i & 1) ? 0.5 : -0.5) * _lx,
      _c.Y() + ((i & 2) ? 0.5 : -0.5) * _ly,
      _c.Z() + ((i & 4) ? 0.5 : -0.5) * _lz);
  }
  // Each face as two triangles, counter-clockwise seen from outside
  const std::size_t quads[6][4] = {
    {0, 2, 3, 1},   // z-
    {4, 5, 7, 6},   // z+
    {0, 1, 5, 4},   // y-
    {2, 6, 7, 3},   // y+
    {0, 4, 6, 2},   // x-
    {1, 3, 7, 5}};  // x+
  for (const auto & q : quads) {
    m.triangles.push_back({q[0], q[1], q[2]});
    m.triangles.push_back({q[0], q[2], q[3]});
  }
  return m;
}

/// Volume by the divergence theorem: positive only if every normal points outwards.
double SignedVolume(const Mesh & _m)
{
  double v = 0;
  for (const auto & t : _m.triangles) {
    v += _m.vertices[t[0]].Dot(_m.vertices[t[1]].Cross(_m.vertices[t[2]])) / 6.0;
  }
  return v;
}

/// Still water at height _level; vertices given in the world frame already.
Wrench StillWater(const Mesh & _m, double _level, const Vector3d & _ref)
{
  std::vector<double> depths;
  for (const auto & v : _m.vertices) {
    depths.push_back(_level - v.Z());
  }
  return Integrate(_m, _m.vertices, depths, _ref, kRho, kG);
}

}  // namespace

TEST(HullHydrostatics, BoxWindingIsOutward)
{
  EXPECT_NEAR(SignedVolume(Box({0, 0, 0}, 2, 3, 4)), 24.0, 1e-12);
}

TEST(HullHydrostatics, DryBoxFeelsNothing)
{
  const Wrench w = StillWater(Box({0, 0, 5}, 1, 1, 1), 0.0, {0, 0, 0});
  EXPECT_EQ(w.force, Vector3d::Zero);
  EXPECT_EQ(w.wettedArea, 0.0);
}

TEST(HullHydrostatics, FullySubmergedBoxGetsArchimedes)
{
  // 2 x 1 x 0.5 m = 1 m^3, centre 3 m deep
  const Wrench w = StillWater(Box({0, 0, -3}, 2, 1, 0.5), 0.0, {0, 0, 0});
  EXPECT_NEAR(w.force.Z(), kRho * kG * 1.0, 1e-6);
  EXPECT_NEAR(w.force.X(), 0.0, 1e-6);
  EXPECT_NEAR(w.force.Y(), 0.0, 1e-6);
}

TEST(HullHydrostatics, HalfSubmergedBoxActsThroughCentreOfBuoyancy)
{
  // 1 x 1 x 1 box centred at (2, -1, 0): half under water, buoyancy centre (2, -1, -0.25)
  const Wrench w = StillWater(Box({2, -1, 0}, 1, 1, 1), 0.0, {0, 0, 0});
  const double expected = kRho * kG * 0.5;
  EXPECT_NEAR(w.force.Z(), expected, 1e-6);
  EXPECT_NEAR(w.force.X(), 0.0, 1e-6);
  EXPECT_NEAR(w.force.Y(), 0.0, 1e-6);
  // torque about the origin = r x F with F vertical: (y * Fz, -x * Fz, 0)
  EXPECT_NEAR(w.torque.X(), -1.0 * expected, 1e-6);
  EXPECT_NEAR(w.torque.Y(), -2.0 * expected, 1e-6);
  EXPECT_NEAR(w.torque.Z(), 0.0, 1e-6);
  // wetted: bottom 1 m^2 + four half sides 4 x 0.5 m^2
  EXPECT_NEAR(w.wettedArea, 3.0, 1e-9);
}

TEST(HullHydrostatics, TiltedBoxRightsItself)
{
  // Box 2 m wide (y), 1 m tall, half submerged, then rolled 10 degrees about x.
  // A wide, low box has its metacentre above its centre of gravity, so the torque about the
  // centre of the box must turn it back (negative x torque for a positive roll).
  Mesh m = Box({0, 0, 0}, 1, 2, 1);
  const double a = 10.0 * M_PI / 180.0;
  for (auto & v : m.vertices) {
    v = Vector3d(v.X(), v.Y() * std::cos(a) - v.Z() * std::sin(a),
        v.Y() * std::sin(a) + v.Z() * std::cos(a));
  }
  const Wrench w = StillWater(m, 0.0, {0, 0, 0});
  EXPECT_NEAR(w.force.Z(), kRho * kG * 1.0, 1e-6);   // same volume under water
  EXPECT_LT(w.torque.X(), 0.0);
}

TEST(HullHydrostatics, SlopedSurfacePushesDownTheSlope)
{
  // Water surface z = 0.1 y. Pressure rho g (0.1 y - z) has gradient rho g (0, 0.1, -1), so
  // the force on the wetted part is -rho g V (0, 0.1, -1): up, and sideways towards -y.
  // A box cut through its centre by any plane has half its volume below it.
  const Mesh m = Box({0, 0, 0}, 1, 1, 1);
  std::vector<double> depths;
  for (const auto & v : m.vertices) {
    depths.push_back(0.1 * v.Y() - v.Z());
  }
  const Wrench w = Integrate(m, m.vertices, depths, {0, 0, 0}, kRho, kG);
  EXPECT_NEAR(w.force.Z(), kRho * kG * 0.5, 1e-6);
  EXPECT_NEAR(w.force.Y(), -0.1 * kRho * kG * 0.5, 1e-6);
  EXPECT_NEAR(w.force.X(), 0.0, 1e-6);
}

TEST(HullHydrostatics, ObjLoaderReadsVerticesAndFaces)
{
  const std::string path = ::testing::TempDir() + "usv_hull_test.obj";
  {
    std::ofstream f(path);
    f << "# square as a quad, normals given\n"
      << "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
      << "vn 0 0 1\n"
      << "f 1//1 2//1 3//1 4//1\n";
  }
  const Mesh m = LoadObj(path);
  std::remove(path.c_str());
  ASSERT_EQ(m.vertices.size(), 4u);
  ASSERT_EQ(m.triangles.size(), 2u);   // quad split into a fan
  EXPECT_EQ(m.triangles[0][0], 0u);
  EXPECT_EQ(m.triangles[1][2], 3u);
}
