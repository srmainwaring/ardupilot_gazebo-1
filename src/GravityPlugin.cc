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

  /// \brief Model entity.
  public: Model model{kNullEntity};

  /// \brief Name of the model entity.
  public: std::string modelName;

  /// \brief Link entity.
  public: Link link{kNullEntity};

  /// \brief Name of the link entity.
  public: std::string linkName;

  /// \brief Flag set to true if the model is correctly initialised.
  public: bool validConfig{false};
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
  // capture model entity
  this->impl->model = Model(_entity);
  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "GravityPlugin should be attached to a model. "
             "Failed to initialize.\n";
    return;
  }
  this->impl->modelName = this->impl->model.Name(_ecm);

  // retrieve world entity
  this->impl->world = World(
      _ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "GravityPlugin - world not found. "
             "Failed to initialize.\n";
    return;
  }
  if (this->impl->world.Name(_ecm).has_value())
  {
    this->impl->worldName = this->impl->world.Name(_ecm).value();
  }

  // parameters
  if (_sdf->HasElement("link"))
  {
    this->impl->linkName = _sdf->Get<std::string>("link");
  }
  else
  {
    gzerr << "GravityPlugin requires parameter 'link'. "
             "Failed to initialize.\n";
    return;
  }

  // resolve links
  this->impl->link = Link(_ecm.EntityByComponents(
      components::Link(),
      components::ParentEntity(this->impl->model.Entity()),
      components::Name(this->impl->linkName)));
  if (!this->impl->link.Valid(_ecm))
  {
    gzerr << "GravityPlugin - link ["
             << this->impl->linkName
             << "] not found. "
             "Failed to initialize.\n";
    return;
  }
  this->impl->link.EnableVelocityChecks(_ecm, true);
  this->impl->link.EnableAccelerationChecks(_ecm, true);

  this->impl->validConfig = true;
}

//////////////////////////////////////////////////
void GravityPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("GravityPlugin::PreUpdate");

  if (_info.paused)
    return;

  if (!this->impl->validConfig)
    return;

  double G = 6.67430E-11;
  double mass_earth = 5.972E24;

  // hardcoded masses
  double mass_C = mass_earth * 1.0E-10;
  double mass_B = 1.0;

  // world pose of central mass
  auto X_WC = math::Pose3d(0, 0, 0, 0, 0, 0);

  // world pose of body base_link
  auto X_WB = worldPose(this->impl->link.Entity(), _ecm);

  // position vector from central mass to body
  auto p_CB_W = X_WB.Pos() - X_WC.Pos();

  // length of position vector
  double r = p_CB_W.Length();
  if (std::abs(r) < 1.0E-3)
  {
      return;
  }

  double one_over_r = 1.0 / r;
  double one_over_r2 = 1.0 / (r * r);
  // double one_over_r3 = 1.0 / (r * r * r);

  // magnitude of force
  auto f = 1.0 * G * mass_C * mass_B * one_over_r2;

  // direction of force
  auto direction = -1.0 * one_over_r * p_CB_W;

  // force
  auto f_B_W = f * direction;

  // apply force at CoM
  this->impl->link.AddWorldForce(_ecm, f_B_W);

  // debug
  if (this->impl->debug)
  {
    auto v_WB_W = this->impl->link.WorldLinearVelocity(_ecm).value();
    double v = v_WB_W.Length();

    auto a_WB_W = this->impl->link.WorldLinearAcceleration(_ecm).value();
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
