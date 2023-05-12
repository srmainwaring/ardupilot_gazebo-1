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

#include "CameraZoomPlugin.hh"

#include <atomic>
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
#include <gz/sim/Sensor.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
class CameraZoomPlugin::Impl
{
  /// \brief Handle a zoom command.
  public: void OnZoom(const msgs::Double &_msg);

  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief The parent model.
  public: Model parentModel{kNullEntity};

  /// \brief Sensor containing this plugin.
  public: Sensor sensor{kNullEntity};

  /// \brief Name of the topic to subscribe to zoom commands.
  public: std::string zoomTopic;

  /// \brief Value of the most recently received zoom command.
  public: std::atomic<double> zoomCommand{1.0};

  /// \brief Flag set to true if the plugin is correctly initialised.
  public: bool isValidConfig{false};

  /// \brief Transport node for subscriptions.
  public: transport::Node node;
};

//////////////////////////////////////////////////
void CameraZoomPlugin::Impl::OnZoom(const msgs::Double &_msg)
{
  this->zoomCommand = _msg.data();
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
CameraZoomPlugin::~CameraZoomPlugin() = default;

//////////////////////////////////////////////////
CameraZoomPlugin::CameraZoomPlugin() :
    impl(std::make_unique<CameraZoomPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void CameraZoomPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  // Capture sensor entity.
  this->impl->sensor = Sensor(_entity);
  if (!this->impl->sensor.Valid(_ecm))
  {
    gzerr << "CameraZoomPlugin should be attached to a sensor. "
             "Failed to initialize.\n";
    return;
  }

  // Display plugin load status..
  if (auto maybeName = this->impl->sensor.Name(_ecm))
  {
    gzdbg << "CameraZoomPlugin attached to sensor ["
          << maybeName.value() << "].\n";
  }
  else
  {
    gzerr << "CameraZoomPlugin has invalid name.\n";
    return;
  }

  // Retrieve parent model.
  if (auto maybeParentLink = this->impl->sensor.Parent(_ecm))
  {
    Link link(maybeParentLink.value());
    if (link.Valid(_ecm))
    {   
      if (auto maybeParentModel = link.ParentModel(_ecm))
      {
        this->impl->parentModel = maybeParentModel.value();
      }
    }
  }
  if (!this->impl->parentModel.Valid(_ecm))
  {
    gzerr << "CameraZoomPlugin - parent model not found. "
             "Failed to initialize.\n";
    return;
  }

  // Retrieve world entity.
  this->impl->world = World(
      _ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "CameraZoomPlugin - world not found. "
             "Failed to initialize.\n";
    return;
  }

  // Configure zoom command topic.
  {
    std::vector<std::string> topics;
    if (_sdf->HasElement("topic"))
    {
      topics.push_back(_sdf->Get<std::string>("topic"));
    }
    auto parentModelName = this->impl->parentModel.Name(_ecm);
    auto sensorName = this->impl->sensor.Name(_ecm).value();
    topics.push_back("/model/" + parentModelName +
        "/sensor/" + sensorName + "/zoom/cmd_zoom");
    this->impl->zoomTopic = validTopic(topics);
  }

  // Subscriptions.
  this->impl->node.Subscribe(
      this->impl->zoomTopic,
      &CameraZoomPlugin::Impl::OnZoom, this->impl.get());

  gzdbg << "CameraZoomPlugin subscribing to messages on "
         << "[" << this->impl->zoomTopic << "]\n";

  this->impl->isValidConfig = true;
}

//////////////////////////////////////////////////
void CameraZoomPlugin::PreUpdate(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("CameraZoomPlugin::PreUpdate");

  if (!this->impl->isValidConfig)
    return;
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::CameraZoomPlugin,
    gz::sim::System,
    gz::sim::systems::CameraZoomPlugin::ISystemConfigure,
    gz::sim::systems::CameraZoomPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::CameraZoomPlugin,
    "CameraZoomPlugin")
