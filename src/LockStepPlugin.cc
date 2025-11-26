/*
   Copyright (C) 2025 ArduPilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "LockStepPlugin.hh"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/common/SignalHandler.hh>

#include <gz/math/Helpers.hh>

#include <gz/msgs/Utility.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>

#include <gz/transport/Node.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
class LockStepPlugin::Impl
{
  /// \brief Handle a lock step message.
  public: void OnEnable(const msgs::Boolean &_msg);

  /// \brief Handle a step message.
  public: void OnStart(const msgs::Time &_msg);

  /// \brief The model containing this plugin.
  public: Model model{kNullEntity};

  /// \brief The model name.
  public: std::string modelName;

  /// \brief The world containing the model.
  public: World world{kNullEntity};

  /// \brief The world name.
  public: std::string worldName;

  /// \brief Mutex used when accessing lock step message.
  public: std::mutex enableLockStepMutex;

  /// \brief True if lock-step is enabled.
  public: bool enableLockStep{false};

  /// \brief Mutex used when accessing start step message.
  public: std::mutex startStepMutex;

  /// \brief Last start step message received.
  public: gz::msgs::Time startStepMsg;

  /// \brief Indicate a step can start.
  public: bool startStep{false};

  // \brief Keep track of update sim time
  public: std::chrono::steady_clock::duration lastUpdateSimTime{0};

  /// \brief Flag set to true if the plugin is correctly initialised.
  public: bool isValidConfig{false};

  /// \brief Publisher for sending a step complete message
  public: gz::transport::Node::Publisher stepCompletePub;

  /// \brief Transport node for subscriptions.
  public: transport::Node node;

  /// \brief A copy of the most recently received signal.
  public: int signal{0};

  /// \brief Signal handler.
  public: gz::common::SignalHandler sigHandler;

  /// \brief Signal handler callback.
  public: void OnSignal(int _sig);
};

//////////////////////////////////////////////////
void LockStepPlugin::Impl::OnEnable(const msgs::Boolean &_msg)
{
  std::lock_guard<std::mutex> lock(this->enableLockStepMutex);
  this->enableLockStep = _msg.data();
}

//////////////////////////////////////////////////
void LockStepPlugin::Impl::OnStart(const msgs::Time &_msg)
{
  std::lock_guard<std::mutex> lock(this->startStepMutex);
  this->startStepMsg = _msg;
  this->startStep = true;
}

//////////////////////////////////////////////////
void LockStepPlugin::Impl::OnSignal(int _sig)
{
    gzdbg << "LockStepPlugin received signal[" << _sig  << "]"
          << std::endl;
    this->signal = _sig;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
LockStepPlugin::~LockStepPlugin() = default;

//////////////////////////////////////////////////
LockStepPlugin::LockStepPlugin() :
    impl(std::make_unique<LockStepPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void LockStepPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  // Model entity
  this->impl->model = gz::sim::Model(_entity);
  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "LockStepPlugin must be attached to a model entity. "
          << "Failed to initialize."
          << std::endl;
    return;
  }
  this->impl->modelName = this->impl->model.Name(_ecm);

  // World entity
  this->impl->world = World(_ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "LockStepPlugin - world not found. "
          << "Failed to initialize."
          << std::endl;;
    return;
  }

  // Parameters
  std::string enableTopic = "/model/" + this->impl->modelName
      + "/lock_step/enable";
  std::string startTopic = "/model/" + this->impl->modelName
      + "/lock_step/start";
  std::string completeTopic = "/model/" + this->impl->modelName
      + "/lock_step/complete";

  // Subscriptions
  this->impl->node.Subscribe(
      enableTopic,
      &LockStepPlugin::Impl::OnEnable, this->impl.get());

  gzdbg << "LockStepPlugin subscribing to messages on "
         << "[" << enableTopic << "]"
         << std::endl;

  this->impl->node.Subscribe(
      startTopic,
      &LockStepPlugin::Impl::OnStart, this->impl.get());

  gzdbg << "LockStepPlugin subscribing to messages on "
         << "[" << startTopic << "]"
         << std::endl;

  // Publishers
  this->impl->stepCompletePub =
      this->impl->node.Advertise<msgs::Time>(completeTopic);

  gzdbg << "LockStepPlugin publishing messages on "
         << "[" << completeTopic << "]"
         << std::endl;

  // Signal handler
  this->impl->sigHandler.AddCallback(
      std::bind(
        &LockStepPlugin::Impl::OnSignal,
        this->impl.get(),
        std::placeholders::_1));

  // Ready
  this->impl->isValidConfig = true;
}

//////////////////////////////////////////////////
void LockStepPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  using namespace std::literals;

  GZ_PROFILE("LockStepPlugin::PreUpdate");

  if (!this->impl->isValidConfig)
  {
    return;
  }

  if (!_info.paused && _info.simTime > this->impl->lastUpdateSimTime
      && this->impl->enableLockStep)
  {
    while (!this->impl->startStep)
    {
      // SIGNINT should interrupt this loop.
      if (this->impl->signal != 0)
      {
        break;
      }
      std::this_thread::sleep_for(100us);
    }
    {
      std::lock_guard<std::mutex> lock(this->impl->startStepMutex);
      this->impl->startStep = false;
    }
  }
}

//////////////////////////////////////////////////
void LockStepPlugin::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("LockStepPlugin::PostUpdate");

  if (!_info.paused && _info.simTime > this->impl->lastUpdateSimTime
      && this->impl->enableLockStep)
  {
    this->impl->lastUpdateSimTime = _info.simTime;

    auto simTimeSecNsec = math::durationToSecNsec(_info.simTime);
    msgs::Time stepCompleteMsg;
    stepCompleteMsg.set_sec(simTimeSecNsec.first);
    stepCompleteMsg.set_nsec(simTimeSecNsec.second);
    this->impl->stepCompletePub.Publish(stepCompleteMsg);
  }
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::LockStepPlugin,
    gz::sim::System,
    gz::sim::systems::LockStepPlugin::ISystemConfigure,
    gz::sim::systems::LockStepPlugin::ISystemPreUpdate,
    gz::sim::systems::LockStepPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::LockStepPlugin,
    "LockStepPlugin")
