/*
   Copyright (C) 2023 ardupilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "GravityPlugin.hh"

#include <string>

#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
class GravityPlugin::Impl
{
  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief Name of the world entity.
  public: std::string worldName;

  /// \brief Model providing the source of gravity.
  public: Model sourceModel{kNullEntity};

  /// \brief Name of the gravity source model entity.
  public: std::string sourceName;

  /// \brief Flag set to true for additional debug messages.
  public: bool debug{false};

  /// \brief Flag set to true if the plugin is correctly configured.
  public: bool validConfig{false};

  /// \brief Flag set to true if the plugin is initialised.
  public: bool initialised{false};

  /// \brief Universal gravitational constant (N m^2 / kg^2).
  public: static constexpr double G{6.67430E-11};

  /// \brief Mass of earth (kg).
  //public: static constexpr double massEarth{5.972E24};
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
GravityPlugin::~GravityPlugin() = default;

//////////////////////////////////////////////////
GravityPlugin::GravityPlugin() :
    impl(std::make_unique<GravityPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void GravityPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  // capture world entity
  this->impl->world = World(_entity);
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "GravityPlugin: should be attached to a world. "
             "Failed to initialize.\n";
    return;
  }
  if (this->impl->world.Name(_ecm).has_value())
  {
    this->impl->worldName = this->impl->world.Name(_ecm).value();
  }

  // parameters
  if (_sdf->HasElement("source"))
  {
    this->impl->sourceName = _sdf->Get<std::string>("source");
  }
  else
  {
    gzerr << "GravityPlugin: requires parameter 'source'. "
             "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("debug"))
  {
    this->impl->debug = _sdf->Get<bool>("debug");
  }

  this->impl->validConfig = true;
}

//////////////////////////////////////////////////
void GravityPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("GravityPlugin::PreUpdate");

  if (!this->impl->validConfig)
    return;

  if (!this->impl->initialised)
  {
    // resolve source model
    this->impl->sourceModel = Model(
        this->impl->world.ModelByName(_ecm, this->impl->sourceName));
    static bool notified{false};
    if (!this->impl->sourceModel.Valid(_ecm) && !notified)
    {
      gzerr << "GravityPlugin: source model ["
              << this->impl->sourceName
              << "] not found. "
              "Failed to initialize.\n";
      notified = true;
      return;
    }
    this->impl->initialised = true;
  }

  if (!this->impl->initialised || _info.paused)
    return;

  // canonical link of gravity source (C for central force)
  auto link_C = Link(this->impl->sourceModel.CanonicalLink(_ecm));

  // world pose of source
  auto X_WC = worldPose(link_C.Entity(), _ecm);

  // mass of source
  auto *inertialComp_C = _ecm.Component<components::Inertial>(link_C.Entity());
  {
    static bool notified{false};
    if (inertialComp_C == nullptr)
    {
      gzerr << "GravityPlugin: source model ["
              << this->impl->sourceName
              << "] does not have valid inertial. "
              "Failed to initialize.\n";
      notified = true;
      return;
    }
  }
  const auto &inertial_C = inertialComp_C->Data();
  double mass_C = inertial_C.MassMatrix().Mass();

  // loop over all links
  _ecm.Each<components::Link>(
      [&](const Entity &_entity,
          const components::Link *) -> bool
  {
    Link link_B(_entity);

    /// @todo(srmainwaring) add check that gravity is enabled for link.

    // mass of link
    auto *inertialComp_B =
        _ecm.Component<components::Inertial>(link_B.Entity());
    if (inertialComp_B == nullptr)
    {
      return true;
    }
    const auto &inertial_B = inertialComp_B->Data();
    double mass_B = inertial_B.MassMatrix().Mass();

    // world pose of link
    auto X_WB = worldPose(_entity, _ecm);

    // position vector from central mass to body
    auto p_CB_W = X_WB.Pos() - X_WC.Pos();

    // length of position vector
    double r = p_CB_W.Length();

    // avoid singularity as r -> 0
    if (std::abs(r) < 1.0E-3)
    {
      return true;
    }

    // inverse radius
    double one_over_r = 1.0 / r;
    double one_over_r2 = 1.0 / (r * r);

    // magnitude of force
    auto f = 1.0 * this->impl->G * mass_C * mass_B * one_over_r2;

    // direction of force
    auto direction = -1.0 * one_over_r * p_CB_W;

    // force
    auto f_B_W = f * direction;

    // apply force at CoM
    link_B.AddWorldForce(_ecm, f_B_W);

    // debug
    if (this->impl->debug)
    {
      // enable velocity and acceleration components
      link_B.EnableVelocityChecks(_ecm, true);
      link_B.EnableAccelerationChecks(_ecm, true);

      auto v_WB_W = link_B.WorldLinearVelocity(_ecm).value();
      double v = v_WB_W.Length();

      auto a_WB_W = link_B.WorldLinearAcceleration(_ecm).value();
      double a = a_WB_W.Length();

      gzdbg << "p_CB_W: " << p_CB_W << "\n";
      gzdbg << "v_WB_W: " << v_WB_W << "\n";
      gzdbg << "a_WB_W: " << a_WB_W << "\n";
      gzdbg << "mass_C: " << mass_C << "\n";
      gzdbg << "mass_B: " << mass_B << "\n";
      gzdbg << "r:      " << r << "\n";
      gzdbg << "v:      " << v << "\n";
      gzdbg << "a:      " << a << "\n";
      gzdbg << "f:      " << f << "\n";
      gzdbg << "f_B_W:  " << f_B_W << "\n";
    }

    return true;
  });
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::GravityPlugin,
    gz::sim::System,
    gz::sim::systems::GravityPlugin::ISystemConfigure,
    gz::sim::systems::GravityPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::GravityPlugin,
    "GravityPlugin")
