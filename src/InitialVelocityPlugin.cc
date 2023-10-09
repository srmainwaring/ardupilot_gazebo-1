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


// Adapted from VelocityController

#include "InitialVelocityPlugin.hh"

#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
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
class InitialVelocityPlugin::Impl
{
  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief Name of the world entity.
  public: std::string worldName;

  /// \brief Model interface
  public: Model model{kNullEntity};

 /// \brief Name of the model entity.
  public: std::string modelName;

  /// \brief Link entity.
  public: Link link{kNullEntity};

  /// \brief Name of the link entity.
  public: std::string linkName;

  /// \brief Flag set to true if the model is correctly initialised.
  public: bool validConfig{false};

  /// \brief Target initial linear world velocity
  public: math::Vector3d linearVelocity{0, 0, 0};

  /// \brief Velocity-X PID controller.
  public: math::PID velXPid;

  /// \brief Velocity-Y PID controller.
  public: math::PID velYPid;

  /// \brief Velocity-Z PID controller.
  public: math::PID velZPid;

  /// \brief Max number of iterations for initial impulse.
  public: int maxIter{100};

  /// \brief Number of iterations for initial impulse.
  public: int iter{0};

  /// \brief Flag set to true for additional debug messages.
  public: bool debug{false};
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
InitialVelocityPlugin::~InitialVelocityPlugin() = default;

//////////////////////////////////////////////////
InitialVelocityPlugin::InitialVelocityPlugin() :
    impl(std::make_unique<InitialVelocityPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void InitialVelocityPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  // capture model entity
  this->impl->model = Model(_entity);
  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "InitialVelocityPlugin: should be attached to a model. "
             "Failed to initialize.\n";
    return;
  }
  this->impl->modelName = this->impl->model.Name(_ecm);

  // retrieve world entity
  this->impl->world = World(
      _ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "InitialVelocityPlugin: world not found. "
             "Failed to initialize.\n";
    return;
  }
  if (this->impl->world.Name(_ecm).has_value())
  {
    this->impl->worldName = this->impl->world.Name(_ecm).value();
  }

  // parameters
  if (_sdf->HasElement("linear"))
  {
    this->impl->linearVelocity = _sdf->Get<math::Vector3d>("linear");
    gzmsg << "InitialVelocityPlugin: linear velocity set to ["
          << this->impl->linearVelocity << "]\n";
  }

  if (_sdf->HasElement("link"))
  {
    this->impl->linkName = _sdf->Get<std::string>("link");
  }
  else
  {
    gzerr << "InitialVelocityPlugin: requires parameter 'link'. "
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
    gzerr << "InitialVelocityPlugin: link ["
             << this->impl->linkName
             << "] not found. "
             "Failed to initialize.\n";
    return;
  }

  // PID parameters
  double p         = _sdf->Get<double>("p_gain",     1.0).first;
  double i         = _sdf->Get<double>("i_gain",     0.0).first;
  double d         = _sdf->Get<double>("d_gain",     0.0).first;
  double iMax      = _sdf->Get<double>("i_max",      1.0).first;
  double iMin      = _sdf->Get<double>("i_min",     -1.0).first;
  double cmdMax    = _sdf->Get<double>("cmd_max",    1000.0).first;
  double cmdMin    = _sdf->Get<double>("cmd_min",   -1000.0).first;
  double cmdOffset = _sdf->Get<double>("cmd_offset", 0.0).first;

  this->impl->velXPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->impl->velYPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->impl->velZPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

  if (_sdf->HasElement("debug"))
  {
    this->impl->debug = _sdf->Get<bool>("debug");
  }

  if (this->impl->debug)
  {
    gzdbg << "PID parameters:\n";
    gzdbg << "p_gain: ["     << p         << "]\n";
    gzdbg << "i_gain: ["     << i         << "]\n";
    gzdbg << "d_gain: ["     << d         << "]\n";
    gzdbg << "i_max: ["      << iMax      << "]\n";
    gzdbg << "i_min: ["      << iMin      << "]\n";
    gzdbg << "cmd_max: ["    << cmdMax    << "]\n";
    gzdbg << "cmd_min: ["    << cmdMin    << "]\n";
    gzdbg << "cmd_offset: [" << cmdOffset << "]\n";
  }

  this->impl->link.EnableVelocityChecks(_ecm, true);

  this->impl->validConfig = true;
}

//////////////////////////////////////////////////
void InitialVelocityPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("InitialVelocityPlugin::PreUpdate");

  if (!this->impl->validConfig || _info.paused)
    return;

  if (this->impl->iter > this->impl->maxIter)
    return;

  this->impl->iter++;

  // current world velocity
  auto v_WB_W = this->impl->link.WorldLinearVelocity(_ecm).value();

  double errorX = v_WB_W.X() - this->impl->linearVelocity.X();
  double errorY = v_WB_W.Y() - this->impl->linearVelocity.Y();
  double errorZ = v_WB_W.Z() - this->impl->linearVelocity.Z();

  double forceX = this->impl->velXPid.Update(errorX, _info.dt);
  double forceY = this->impl->velYPid.Update(errorY, _info.dt);
  double forceZ = this->impl->velZPid.Update(errorZ, _info.dt);

  auto f_B_W = math::Vector3d(forceX, forceY, forceZ);

  // apply force at CoM
  this->impl->link.AddWorldForce(_ecm, f_B_W);

  if (this->impl->debug)
  {
    gzdbg << "iter:   " << this->impl->iter << "\n";
    gzdbg << "errorX: " << errorX << "\n";
    gzdbg << "errorY: " << errorY << "\n";
    gzdbg << "errorZ: " << errorZ << "\n";
    gzdbg << "forceX: " << forceX << "\n";
    gzdbg << "forceY: " << forceY << "\n";
    gzdbg << "forceZ: " << forceZ << "\n";
  }
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::InitialVelocityPlugin,
    gz::sim::System,
    gz::sim::systems::InitialVelocityPlugin::ISystemConfigure,
    gz::sim::systems::InitialVelocityPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::InitialVelocityPlugin,
    "InitialVelocityPlugin")
