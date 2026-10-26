// HullBuoyancy: buoyancy of a boat from its real hull shape.
//
// Replaces the VRX Surface system, which treats each hull as a horizontal cylinder and
// stops adding buoyancy once half of it is under water. Here the water pressure is
// integrated over a closed hull mesh (see HullHydrostatics.hh), so draft, heave, roll and
// pitch stiffness and the reserve buoyancy all follow from the shape. Waves come from the
// same VRX wavefield the rest of the world uses.
//
// Only hydrostatics (and the wave surface it sees). Drag, damping and added mass stay in
// VRX SimpleHydrodynamics.
//
// SDF (inside a <model>):
//   <plugin filename="UsvHullBuoyancy" name="usv_gz_plugins::HullBuoyancy">
//     <link_name>base_link</link_name>
//     <mesh>file://usv_boat/meshes/hull.obj</mesh>  <!-- as for a visual; see ResolvePath -->
//     <hull_offset>0 0.26 0</hull_offset>     <!-- one per hull copy, in the link frame -->
//     <hull_offset>0 -0.26 0</hull_offset>
//     <fluid_density>1025</fluid_density>
//     <fluid_level>0</fluid_level>
//     <report_period>5</report_period>        <!-- s; 0 = silent -->
//     <wavefield><topic>/vrx/wavefield/parameters</topic></wavefield>
//   </plugin>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/param.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "Wavefield.hh"                       // VRX, from /opt/vrx_ws (see CMakeLists.txt)
#include "usv_gz_plugins/HullHydrostatics.hh"

namespace usv_gz_plugins
{

class HullBuoyancy
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    gz::sim::Model model(_entity);
    if (!model.Valid(_ecm)) {
      gzerr << "HullBuoyancy must be attached to a model.\n";
      return;
    }
    const std::string linkName = _sdf->Get<std::string>("link_name", "base_link").first;
    this->link = gz::sim::Link(model.LinkByName(_ecm, linkName));
    if (!this->link.Valid(_ecm)) {
      gzerr << "HullBuoyancy: no link named [" << linkName << "].\n";
      return;
    }
    this->link.EnableVelocityChecks(_ecm, true);   // also keeps the WorldPose component

    const std::string meshUri = _sdf->Get<std::string>("mesh", "").first;
    const std::string meshPath = this->ResolvePath(meshUri, _sdf->FilePath());
    if (meshPath.empty()) {
      gzerr << "HullBuoyancy: cannot find mesh [" << meshUri << "].\n";
      return;
    }
    try {
      this->mesh = LoadObj(meshPath);
    } catch (const std::exception & e) {
      gzerr << "HullBuoyancy: " << e.what() << "\n";
      return;
    }

    auto sdf = _sdf->Clone();
    for (auto e = sdf->FindElement("hull_offset"); e; e = e->GetNextElement("hull_offset")) {
      this->offsets.push_back(e->Get<gz::math::Vector3d>());
    }
    if (this->offsets.empty()) {
      this->offsets.emplace_back(0, 0, 0);
    }
    this->fluidDensity = _sdf->Get<double>("fluid_density", 1025.0).first;
    this->fluidLevel = _sdf->Get<double>("fluid_level", 0.0).first;
    this->reportPeriod = _sdf->Get<double>("report_period", 0.0).first;

    gz::sim::World world(gz::sim::worldEntity(_ecm));
    this->gravity = world.Gravity(_ecm).value_or(gz::math::Vector3d(0, 0, -9.8)).Length();

    this->wavefield.Load(_sdf);
    const std::string topic = this->wavefield.Topic();
    if (!topic.empty()) {
      this->node.Subscribe(topic, &HullBuoyancy::OnWavefield, this);
    }

    // Every hull copy reuses the same mesh: one block of points/depths per copy
    const std::size_t n = this->mesh.vertices.size() * this->offsets.size();
    this->points.resize(n);
    this->depths.resize(n);
    this->ready = true;
    gzmsg << "HullBuoyancy: " << meshPath << ", " << this->mesh.vertices.size()
          << " vertices, " << this->mesh.triangles.size() << " triangles x "
          << this->offsets.size() << " hull(s), fluid density " << this->fluidDensity
          << " kg/m^3, wavefield topic [" << topic << "].\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (!this->ready || _info.paused) {
      return;
    }
    const auto pose = this->link.WorldPose(_ecm);
    if (!pose) {
      return;
    }
    const double time = std::chrono::duration<double>(_info.simTime).count();

    std::size_t k = 0;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      const bool waves = this->wavefield.Active();
      for (const auto & offset : this->offsets) {
        for (const auto & v : this->mesh.vertices) {
          const gz::math::Vector3d p = pose->Pos() + pose->Rot().RotateVector(v + offset);
          double eta = 0.0;
          if (waves) {
            eta = this->wavefield.ComputeDepthSimply(gz::math::Vector3d(p.X(), p.Y(), 0), time);
            if (!std::isfinite(eta)) {
              eta = 0.0;
            }
          }
          this->points[k] = p;
          this->depths[k] = this->fluidLevel + eta - p.Z();
          ++k;
        }
      }
    }

    Wrench total;
    const std::size_t nv = this->mesh.vertices.size();
    for (std::size_t h = 0; h < this->offsets.size(); ++h) {
      // Integrate() indexes points by mesh vertex, so give it this copy's block
      const std::vector<gz::math::Vector3d> pts(
        this->points.begin() + h * nv, this->points.begin() + (h + 1) * nv);
      const std::vector<double> dep(
        this->depths.begin() + h * nv, this->depths.begin() + (h + 1) * nv);
      const Wrench w = Integrate(
        this->mesh, pts, dep, pose->Pos(), this->fluidDensity, this->gravity);
      total.force += w.force;
      total.torque += w.torque;
      total.wettedArea += w.wettedArea;
    }

    // ExternalWorldWrenchCmd acts at the link origin; the torque above is about it too
    this->link.AddWorldWrench(_ecm, total.force, total.torque);

    if (this->reportPeriod > 0 && time - this->lastReport >= this->reportPeriod) {
      this->lastReport = time;
      gzmsg << "HullBuoyancy: t " << time << " s, buoyancy " << total.force.Z() / this->gravity
            << " kg, wetted area " << total.wettedArea << " m^2, link origin z "
            << pose->Pos().Z() << " m\n";
    }
  }

private:
  void OnWavefield(const gz::msgs::Param & _msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->wavefield.Load(_msg);
  }

  /// Mesh path: absolute, relative to the model folder, or found on GZ_SIM_RESOURCE_PATH
  /// (a "model://" or "file://" prefix is dropped).
  static std::string ResolvePath(std::string _uri, const std::string & _sdfFile)
  {
    namespace fs = std::filesystem;
    for (const std::string prefix : {"model://", "file://"}) {
      if (_uri.rfind(prefix, 0) == 0) {
        _uri = _uri.substr(prefix.size());
      }
    }
    if (_uri.empty()) {
      return "";
    }
    std::vector<fs::path> candidates{fs::path(_uri)};
    if (!_sdfFile.empty()) {
      candidates.push_back(fs::path(_sdfFile).parent_path() / _uri);
    }
    if (const char * env = std::getenv("GZ_SIM_RESOURCE_PATH")) {
      std::string paths(env);
      std::size_t start = 0;
      while (start <= paths.size()) {
        const std::size_t end = paths.find(':', start);
        const std::string dir = paths.substr(start, end - start);
        if (!dir.empty()) {
          candidates.push_back(fs::path(dir) / _uri);
        }
        if (end == std::string::npos) {
          break;
        }
        start = end + 1;
      }
    }
    for (const auto & c : candidates) {
      std::error_code ec;
      if (fs::is_regular_file(c, ec)) {
        return c.string();
      }
    }
    return "";
  }

  gz::sim::Link link;
  Mesh mesh;
  std::vector<gz::math::Vector3d> offsets;
  double fluidDensity{1025.0};
  double fluidLevel{0.0};
  double gravity{9.8};
  double reportPeriod{0.0};
  double lastReport{-1e9};
  bool ready{false};

  vrx::Wavefield wavefield;
  gz::transport::Node node;
  std::mutex mutex;

  std::vector<gz::math::Vector3d> points;
  std::vector<double> depths;
};

}  // namespace usv_gz_plugins

GZ_ADD_PLUGIN(
  usv_gz_plugins::HullBuoyancy,
  gz::sim::System,
  usv_gz_plugins::HullBuoyancy::ISystemConfigure,
  usv_gz_plugins::HullBuoyancy::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(usv_gz_plugins::HullBuoyancy, "usv_gz_plugins::HullBuoyancy")
