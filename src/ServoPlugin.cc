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

#include "ServoPlugin.hh"

#include <gz/msgs/double.pb.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/parameters.hh>


namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
//! \brief Utility to declare and update a parameter owned by another object
template <typename T>
class ParameterProxy
{
  //! \brief Constructor
  public: ParameterProxy(const std::string_view& _name) : name(_name)
  {
  }

  //! \brief Initialise
  public: template<typename Getter, typename Setter>
  void Init(
    gz::transport::parameters::ParametersRegistry * _registry,
    T * _obj,
    Getter&& _getter,
    Setter&& _setter,
    const std::string_view& _prefix)
  {
    this->registry = _registry;
    this->obj = _obj;
    this->getter = std::forward<Getter>(_getter);
    this->setter = std::forward<Setter>(_setter);
    this->prefix = _prefix;
    this->scopedName = std::string(this->prefix) + std::string(this->name);
  }

  //! \brief Declare the parameter to the registry
  public: void Declare()
  {
    if (this->registry == nullptr)
    {
      gzerr << "Uninitialised parameter " << this->scopedName << std::endl;
      return;
    }
    auto value = std::make_unique<gz::msgs::Double>();
    value->set_data(this->getter(*obj));
    auto result = this->registry->DeclareParameter(
        this->scopedName, std::move(value));
  }

  //! \brief Update the parameter from the registry
  public: void Update()
  {
    if (this->registry == nullptr)
    {
      return;
    }
    const double change_tolerance{1.0e-8};

    auto value = std::make_unique<gz::msgs::Double>();
    auto result = this->registry->Parameter(scopedName, *value);
    if (result.ResultType() ==
        gz::transport::parameters::ParameterResultType::Success)
    {
      const double a = this->getter(*obj);
      const double b = value->data();
      const bool changed = !math::equal(a, b, change_tolerance);
      if (changed)
      {
        this->setter(*obj, b);
        gzdbg << "Parameter " << this->scopedName << " updated from "
              << a << " to " << b << std::endl;
      }
    }
    else
    {
      gzerr << "Failed to get parameter [" << this->scopedName << "] :"
            << result << std::endl;
    }
  }

  private: gz::transport::parameters::ParametersRegistry * registry{nullptr};
  private: T * obj{nullptr};
  private: std::function<double(const T&)> getter;
  private: std::function<void(T&, double)> setter;
  private: std::string_view prefix;
  private: std::string_view name;
  private: std::string scopedName;
};

//////////////////////////////////////////////////
class ServoPlugin::Impl
{
  //! Helper to initialise and declare a PID parameter
  public: template<typename Getter, typename Setter>
  void DeclareParameter(
      ParameterProxy<math::PID>& param,
      Getter&& getter,
      Setter&& setter,
      const std::string& prefix)
  {
    param.Init(registry, &posPid,
      std::forward<Getter>(getter),
      std::forward<Setter>(setter),
      prefix);
    param.Declare();
  }

  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief Name of the world entity.
  public: std::string worldName;

  /// \brief Model entity.
  public: Model model{kNullEntity};

  /// \brief Name of the model entity.
  public: std::string modelName;

  /// \brief Joint Entity
  public: Joint joint{kNullEntity};

  /// \brief Name of the joint entity.
  public: std::string jointName;

  /// \brief Position PID controller.
  public: math::PID posPid;

  /// \brief Parameters registry
  public: transport::parameters::ParametersRegistry * registry;

  /// Dynamic parameters
  public: ParameterProxy<math::PID> pGain{"p_gain"};
  public: ParameterProxy<math::PID> iGain{"i_gain"};
  public: ParameterProxy<math::PID> dGain{"d_gain"};
  public: ParameterProxy<math::PID> iMax{"i_max"};
  public: ParameterProxy<math::PID> iMin{"i_min"};
  public: ParameterProxy<math::PID> cmdMax{"cmd_max"};
  public: ParameterProxy<math::PID> cmdMin{"cmd_min"};
  public: ParameterProxy<math::PID> cmdOffset{"cmd_offset"};

  /// \brief Transport node for subscriptions.
  public: transport::Node node;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
ServoPlugin::~ServoPlugin() = default;

//////////////////////////////////////////////////
ServoPlugin::ServoPlugin() :
    impl(std::make_unique<ServoPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void ServoPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  // Capture model entity
  this->impl->model = Model(_entity);
  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "ServoPlugin should be attached to a model. "
             "Failed to initialize." << std::endl;
    return;
  }
  this->impl->modelName = this->impl->model.Name(_ecm);

  // Retrieve world entity
  this->impl->world = World(
      _ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "ServoPlugin - world not found. "
             "Failed to initialize." << std::endl;
    return;
  }
  if (this->impl->world.Name(_ecm).has_value())
  {
    this->impl->worldName = this->impl->world.Name(_ecm).value();
  }

  // Parameter: joint_name
  if (_sdf->HasElement("joint_name"))
  {
    this->impl->jointName = _sdf->Get<std::string>("joint_name");
  }
  else
  {
    gzerr << "Failed to get any <joint_name>." << std::endl;
    return;
  }

  // Retrieve joint entity
  {
    const std::string &name = this->impl->jointName;

    // Resolve by scoped name.
    Entity jointEntity = kNullEntity;
    auto entities = entitiesFromScopedName(
        name, _ecm, this->impl->model.Entity());

    if (!entities.empty())
    {
      if (entities.size() > 1)
      {
        gzwarn << "Multiple joint entities with name ["
              << name << "] found. "
              << "Using the first one.\n";
      }
      jointEntity = *entities.begin();

      // Validate
      if (!_ecm.EntityHasComponentType(jointEntity, components::Joint::typeId))
      {
        gzerr << "Entity with name[" << name
              << "] is not a joint\n";
        jointEntity = kNullEntity;
      }
      else
      {
        gzdbg << "Identified joint [" << name
              << "] as Entity [" << jointEntity << "]\n";
      }
    }

    if (jointEntity != kNullEntity)
    {
      this->impl->joint = Joint(jointEntity);
    }
    else
    {
      gzwarn << "Failed to find joint [" << name << "]\n";
    }
  }
}

//////////////////////////////////////////////////
void ServoPlugin::ConfigureParameters(
    gz::transport::parameters::ParametersRegistry &_registry,
    gz::sim::EntityComponentManager &_ecm)
{
  this->impl->registry = &_registry;

  std::string scopedName = gz::sim::scopedName(
    this->impl->joint.Entity(), _ecm, ".", false);
  std::string prefix = std::string("ServoPlugin.") + scopedName
    + std::string(".");

  //! @note not using gz::msgs::PID because the message does not support all
  //!       fields available in gz::math::PID (cmd_max, cmd_min, cmd_offset)

  // Declare parameter proxies
  this->impl->DeclareParameter(this->impl->pGain,
      &math::PID::PGain, &math::PID::SetPGain, prefix);
  this->impl->DeclareParameter(this->impl->iGain,
      &math::PID::IGain, &math::PID::SetIGain, prefix);
  this->impl->DeclareParameter(this->impl->dGain,
      &math::PID::DGain, &math::PID::SetDGain, prefix);
  this->impl->DeclareParameter(this->impl->iMax,
      &math::PID::IMax, &math::PID::SetIMax, prefix);
  this->impl->DeclareParameter(this->impl->iMin,
      &math::PID::IMin, &math::PID::SetIMin, prefix);
  this->impl->DeclareParameter(this->impl->cmdMax,
      &math::PID::CmdMax, &math::PID::SetCmdMax, prefix);
  this->impl->DeclareParameter(this->impl->cmdMin,
      &math::PID::CmdMin, &math::PID::SetCmdMin, prefix);
  this->impl->DeclareParameter(this->impl->cmdOffset,
      &math::PID::CmdOffset, &math::PID::SetCmdOffset, prefix);
}

//////////////////////////////////////////////////
void ServoPlugin::PreUpdate(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ServoPlugin::PreUpdate");

  // Update parameters
  this->impl->pGain.Update();
  this->impl->iGain.Update();
  this->impl->dGain.Update();
  this->impl->iMax.Update();
  this->impl->iMin.Update();
  this->impl->cmdMax.Update();
  this->impl->cmdMin.Update();
  this->impl->cmdOffset.Update();
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::ServoPlugin,
    gz::sim::System,
    gz::sim::systems::ServoPlugin::ISystemConfigureParameters,
    gz::sim::systems::ServoPlugin::ISystemConfigure,
    gz::sim::systems::ServoPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::ServoPlugin,
    "ServoPlugin")
