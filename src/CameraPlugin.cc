/*
   Copyright (C) 2023 ArduPilot.org

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

#include "CameraPlugin.hh"

#include <string>

#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
class CameraPlugin::Impl
{
  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief Model containing this plugin.
  public: Model model{kNullEntity};

  /// \brief Transport node for subscriptions.
  public: transport::Node node;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
CameraPlugin::~CameraPlugin() = default;

//////////////////////////////////////////////////
CameraPlugin::CameraPlugin() :
    impl(std::make_unique<CameraPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void CameraPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  // Capture model entity.
  this->impl->model = Model(_entity);
  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "CameraPlugin should be attached to a model. "
             "Failed to initialize.\n";
    return;
  }

  // Retrieve world entity.
  this->impl->world = World(
      _ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "CameraPlugin - world not found. "
             "Failed to initialize.\n";
    return;
  }
}

//////////////////////////////////////////////////
void CameraPlugin::PreUpdate(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("CameraPlugin::PreUpdate");
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::CameraPlugin,
    gz::sim::System,
    gz::sim::systems::CameraPlugin::ISystemConfigure,
    gz::sim::systems::CameraPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::CameraPlugin,
    "CameraPlugin")
