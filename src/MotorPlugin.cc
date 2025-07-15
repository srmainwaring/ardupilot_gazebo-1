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

#include "MotorPlugin.hh"

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/parameters.hh>

#include "Util.hh"

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
  public: ParameterProxy(const std::string_view &_name) : name(_name)
  {
  }

  //! \brief Initialise
  public: template<typename Getter, typename Setter>
  void Init(
    gz::transport::parameters::ParametersRegistry *_registry,
    T *_obj,
    Getter &&_getter,
    Setter &&_setter,
    const std::string_view &_prefix)
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
    const double changeTolerance{1.0e-8};

    auto value = std::make_unique<gz::msgs::Double>();
    auto result = this->registry->Parameter(scopedName, *value);
    if (result.ResultType() ==
        gz::transport::parameters::ParameterResultType::Success)
    {
      const double a = this->getter(*obj);
      const double b = value->data();
      const bool changed = !math::equal(a, b, changeTolerance);
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

  private: gz::transport::parameters::ParametersRegistry *registry{nullptr};
  private: T *obj{nullptr};
  private: std::function<double(const T&)> getter;
  private: std::function<void(T&, double)> setter;
  private: std::string_view prefix;
  private: std::string_view name;
  private: std::string scopedName;
};

//////////////////////////////////////////////////
class MotorPlugin::Impl
{ 
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Double &_msg);

  /// \brief Callback for actuator velocity subscription
  /// \param[in] _msg Actuators.velocity message
  public: void OnActuatorVel(const msgs::Actuators &_msg);

  /// \brief Helper to initialise and declare a PID parameter
  public: template<typename Getter, typename Setter>
  void DeclareParameter(
      ParameterProxy<math::PID> &_param,
      Getter &&_getter,
      Setter &&_setter,
      const std::string &_prefix)
  {
    _param.Init(registry, &velPid,
      std::forward<Getter>(_getter),
      std::forward<Setter>(_setter),
      _prefix);
    _param.Declare();
  }

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Joint Entity
  public: std::vector<Entity> jointEntities;

  /// \brief Joint name
  public: std::vector<std::string> jointNames;

  /// \brief Commanded joint velocity
  public: double jointVelCmd{0.0};

  /// \brief Index of velocity actuator.
  public: int actuatorNumber = 0;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;

  /// \brief True if using Actuator msg to control joint velocity.
  public: bool useActuatorMsg{false};

  /// \brief True if braking is disabled.
  public: bool disableBraking{false};

  /// \brief The battery voltage (terminal voltage) (V).
  public: double voltageBat;

  /// \brief The motor speed constant K_V (rad/s/V).
  public: double speedConstant;

  /// \brief The motor torque constant K_Q (A/N.m).
  public: double torqueConstant;

  /// \brief R the motor internal resistance (Ohms).
  public: double resistance;

  /// \brief i_0 the current draw when the motor is run at its
  /// specificed voltage and operational RPM with no load (A).
  public: double noLoadCurrent;

  /// \brief Velocity PID controller.
  public: math::PID velPid;

  /// \brief Parameters registry
  public: transport::parameters::ParametersRegistry *registry;

  /// Dynamic parameters
  public: ParameterProxy<math::PID> pGain{"p_gain"};
  public: ParameterProxy<math::PID> iGain{"i_gain"};
  public: ParameterProxy<math::PID> dGain{"d_gain"};
  public: ParameterProxy<math::PID> iMax{"i_max"};
  public: ParameterProxy<math::PID> iMin{"i_min"};
  public: ParameterProxy<math::PID> cmdMax{"cmd_max"};
  public: ParameterProxy<math::PID> cmdMin{"cmd_min"};
  public: ParameterProxy<math::PID> cmdOffset{"cmd_offset"};
};

//////////////////////////////////////////////////
void MotorPlugin::Impl::OnCmdVel(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  this->jointVelCmd = _msg.data();
}

/////////////////////////////////////////////////
void MotorPlugin::Impl::OnActuatorVel(const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  if (this->actuatorNumber > _msg.velocity_size() - 1)
  {
    gzerr << "You tried to access index " << this->actuatorNumber
      << " of the Actuator velocity array which is of size "
      << _msg.velocity_size() << std::endl;
    return;
  }

  this->jointVelCmd = static_cast<double>(_msg.velocity(this->actuatorNumber));
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
MotorPlugin::~MotorPlugin() = default;

//////////////////////////////////////////////////
MotorPlugin::MotorPlugin() : impl(std::make_unique<MotorPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void MotorPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->impl->model = Model(_entity);

  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "MotorPlugin: should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto sdfElem = _sdf->FindElement("joint_name");
  while (sdfElem)
  {
    if (!sdfElem->Get<std::string>().empty())
    {
      this->impl->jointNames.push_back(sdfElem->Get<std::string>());
    }
    else
    {
      gzerr << "<joint_name> provided but is empty." << std::endl;
    }
    sdfElem = sdfElem->GetNextElement("joint_name");
  }
  if (this->impl->jointNames.empty())
  {
    gzerr << "Failed to get any <joint_name>." << std::endl;
    return;
  }

  {
    // PID parameters
    double p         = _sdf->Get<double>("p_gain",     1.0).first;
    double i         = _sdf->Get<double>("i_gain",     0.0).first;
    double d         = _sdf->Get<double>("d_gain",     0.0).first;
    double iMax      = _sdf->Get<double>("i_max",      1.0).first;
    double iMin      = _sdf->Get<double>("i_min",     -1.0).first;
    double cmdMax    = _sdf->Get<double>("cmd_max",    1000.0).first;
    double cmdMin    = _sdf->Get<double>("cmd_min",   -1000.0).first;
    double cmdOffset = _sdf->Get<double>("cmd_offset", 0.0).first;

    this->impl->velPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

    if (_sdf->HasElement("disable_braking"))
    {
      this->impl->disableBraking = _sdf->Get<bool>("disable_braking");
    }

    gzdbg << "MotorPlugin: system parameters:" << std::endl;
    gzdbg << "p_gain: ["     << p         << "]" << std::endl;
    gzdbg << "i_gain: ["     << i         << "]" << std::endl;
    gzdbg << "d_gain: ["     << d         << "]" << std::endl;
    gzdbg << "i_max: ["      << iMax      << "]" << std::endl;
    gzdbg << "i_min: ["      << iMin      << "]" << std::endl;
    gzdbg << "cmd_max: ["    << cmdMax    << "]" << std::endl;
    gzdbg << "cmd_min: ["    << cmdMin    << "]" << std::endl;
    gzdbg << "cmd_offset: [" << cmdOffset << "]" << std::endl;
    gzdbg << "disable_braking: [" << this->impl->disableBraking
          << "]" << std::endl;
  }

  if (_sdf->HasElement("use_actuator_msg") &&
    _sdf->Get<bool>("use_actuator_msg"))
  {
    if (_sdf->HasElement("actuator_number"))
    {
      this->impl->actuatorNumber =
        _sdf->Get<int>("actuator_number");
      this->impl->useActuatorMsg = true;
    }
    else
    {
      gzerr << "Please specify an actuator_number" <<
        "to use Actuator velocity message control." << std::endl;
    }
  }

  // Subscribe to commands
  std::string topic;
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic"))
    && (!this->impl->useActuatorMsg))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
        this->impl->model.Name(_ecm) + "/joint/" +
        this->impl->jointNames[0] + "/cmd_vel");
    if (topic.empty())
    {
      gzerr << "Failed to create topic for joint ["
            << this->impl->jointNames[0]
            << "]" << std::endl;
      return;
    }
  }
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic"))
    && (this->impl->useActuatorMsg))
  {
    topic = transport::TopicUtils::AsValidTopic("/actuators");
    if (topic.empty())
    {
      gzerr << "Failed to create Actuator topic for joint ["
            << this->impl->jointNames[0]
            << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("sub_topic"))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->impl->model.Name(_ecm) + "/" +
        _sdf->Get<std::string>("sub_topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic from sub_topic [/model/"
             << this->impl->model.Name(_ecm) << "/"
             << _sdf->Get<std::string>("sub_topic")
             << "]" << " for joint [" << this->impl->jointNames[0]
             << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
             << "]" << " for joint [" << this->impl->jointNames[0]
             << "]" << std::endl;
      return;
    }
  }

  if (this->impl->useActuatorMsg)
  {
    this->impl->node.Subscribe(topic,
      &MotorPlugin::Impl::OnActuatorVel,
      this->impl.get());

    gzmsg << "MotorPlugin: subscribing to Actuator messages on [" << topic
         << "]" << std::endl;
  }
  else
  {
    this->impl->node.Subscribe(topic,
      &MotorPlugin::Impl::OnCmdVel,
      this->impl.get());

    gzmsg << "MotorPlugin: subscribing to Double messages on [" << topic
         << "]" << std::endl;
  }

  // Electro-mechanical parameters
  if (_sdf->HasElement("voltage_bat"))
  {
    this->impl->voltageBat = _sdf->Get<double>("voltage_bat");
  }
  else
  {
    gzerr << "MotorPlugin: must set `<voltage_bat>`." << std::endl;  
    return;
  }

  if (_sdf->HasElement("speed_constant"))
  {
    this->impl->speedConstant = _sdf->Get<double>("speed_constant");
  }
  else
  {
    gzerr << "MotorPlugin: must set `<speed_constant>`." << std::endl;  
    return;
  }

  if (_sdf->HasElement("torque_constant"))
  {
    this->impl->torqueConstant = _sdf->Get<double>("torque_constant");
  }
  else
  {
    gzerr << "MotorPlugin: must set `<torque_constant>`." << std::endl;  
    return;
  }

  if (_sdf->HasElement("resistance"))
  {
    this->impl->resistance = _sdf->Get<double>("resistance");
  }
  else
  {
    gzerr << "MotorPlugin: must set `<resistance>`." << std::endl;  
    return;
  }

  if (_sdf->HasElement("no_load_current"))
  {
    this->impl->noLoadCurrent = _sdf->Get<double>("no_load_current");
  }
  else
  {
    gzerr << "MotorPlugin: must set `<no_load_current>`." << std::endl;  
    return;
  }

  {
    gzdbg << "MotorPlugin: system parameters:" << std::endl;
    gzdbg << "voltage_bat: [" << this->impl->voltageBat << "]" << std::endl;
    gzdbg << "speed_constant: [" << this->impl->speedConstant << "]" << std::endl;
    gzdbg << "torque_constant: [" << this->impl->torqueConstant << "]" << std::endl;
    gzdbg << "resistance: [" << this->impl->resistance << "]" << std::endl;
    gzdbg << "no_load_current: [" << this->impl->noLoadCurrent << "]" << std::endl;
  }

  // this->impl->validConfig = true;
}

//////////////////////////////////////////////////
void MotorPlugin::ConfigureParameters(
    gz::transport::parameters::ParametersRegistry &_registry,
    gz::sim::EntityComponentManager &_ecm)
{
  this->impl->registry = &_registry;

  // If the joints haven't been identified yet, look for them
  if (this->impl->jointEntities.empty())
  {
    bool warned{false};
    for (const std::string &name : this->impl->jointNames)
    {
      // First try to resolve by scoped name.
      Entity joint = kNullEntity;
      auto entities = entitiesFromScopedName(
          name, _ecm, this->impl->model.Entity());

      if (!entities.empty())
      {
        if (entities.size() > 1)
        {
          gzwarn << "Multiple joint entities with name ["
                << name << "] found. "
                << "Using the first one." << std::endl;
        }
        joint = *entities.begin();

        // Validate
        if (!_ecm.EntityHasComponentType(joint, components::Joint::typeId))
        {
          gzerr << "Entity with name[" << name
                << "] is not a joint" << std::endl;
          joint = kNullEntity;
        }
        else
        {
          gzdbg << "Identified joint [" << name
                << "] as Entity [" << joint << "]" << std::endl;
        }
      }

      if (joint != kNullEntity)
      {
        this->impl->jointEntities.push_back(joint);
      }
      else if (!warned)
      {
        gzwarn << "Failed to find joint [" << name << "]" << std::endl;
        warned = true;
      }
    }
  }

  if (this->impl->jointEntities.empty())
    return;
  
  // Use the first joint to create the scoped parameter names
  std::string scopedName = gz::sim::scopedName(
    this->impl->jointEntities[0], _ecm, ".", false);
  std::string prefix = std::string("MotorPlugin") + scopedName
    + std::string(".");

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
void MotorPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("MotorPlugin::PreUpdate");

  //! @todo(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (this->impl->jointEntities.empty())
    return;

    // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Update parameters
  {
    this->impl->pGain.Update();
    this->impl->iGain.Update();
    this->impl->dGain.Update();
    this->impl->iMax.Update();
    this->impl->iMin.Update();
    this->impl->cmdMax.Update();
    this->impl->cmdMin.Update();
    this->impl->cmdOffset.Update();
  }

  // Create joint velocity component if one doesn't exist
  auto jointVelComp = _ecm.Component<components::JointVelocity>(
      this->impl->jointEntities[0]);
  if (!jointVelComp)
  {
    _ecm.CreateComponent(this->impl->jointEntities[0],
        components::JointVelocity());
  }

  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (jointVelComp == nullptr || jointVelComp->Data().empty())
    return;

  double targetVel;
  {
    std::lock_guard<std::mutex> lock(this->impl->jointVelCmdMutex);
    targetVel = this->impl->jointVelCmd;
  }
  double actualVel = jointVelComp->Data().at(0);
  double error = actualVel - targetVel;

  // If braking is disabled only apply force if the actual velocity is below
  // the target - i.e. allow joint to freewheel.
  bool applyForce = true;
  if (this->impl->disableBraking)
  {
      double sgn = math::signum(targetVel);
      applyForce = !math::equal(sgn, 0.0, 1e-16) && sgn * error <= 0.0;
  }

  // Apply forces.
  if ((!jointVelComp->Data().empty()) &&
      (applyForce))
  {
    for (Entity joint : this->impl->jointEntities)
    {
      // Update force command.
      double force = this->impl->velPid.Update(error, _info.dt);

      auto forceComp =
          _ecm.Component<components::JointForceCmd>(joint);
      if (forceComp == nullptr)
      {
        _ecm.CreateComponent(joint,
                            components::JointForceCmd({force}));
      }
      else
      {
        *forceComp = components::JointForceCmd({force});
      }

      // Electro-mechanical calculations (per-joint)
      // First-Order DC Electric Motor Model
      // Mark Drela, MIT Aero & Astro
      // February 2007
      double Omega = actualVel;
      double Q_m = force;
      double K_V = this->impl->speedConstant;
      double K_Q = this->impl->torqueConstant;
      double i_0 = this->impl->noLoadCurrent;
      double R = this->impl->resistance;
      double v_bat = this->impl->voltageBat;
      double v_m = Omega / K_V;
      double i = Q_m * K_Q + i_0;
      double v = v_m + i * R;
      
      {
        gzdbg << "--------------------------------" << std::endl;
        gzdbg << "MotorPlugin: electic motor calcs" << std::endl;
        gzdbg << "actualVel: " << actualVel << std::endl;
        gzdbg << "targetVel: " << targetVel << std::endl;
        gzdbg << "error: " << error << std::endl;
        gzdbg << "Omega: " << Omega << std::endl;
        gzdbg << "K_V:   " << K_V << std::endl;
        gzdbg << "K_Q:   " << K_Q << std::endl;
        gzdbg << "i_0:   " << i_0 << std::endl;
        gzdbg << "R:     " << R << std::endl;
        gzdbg << "v_bat: " << v_bat << std::endl;
        gzdbg << "v_m:   " << v_m << std::endl;
        gzdbg << "Q_m:   " << Q_m << std::endl;
        gzdbg << "i:     " << i << std::endl;
        gzdbg << "v:     " << v << std::endl;
        gzdbg << "--------------------------------" << std::endl;
      }
    }
  }

#if 0
    for (size_t i = 0; i < this->impl->controls.size(); ++i)
    {
      auto &control = this->impl->controls[i];
      double pwm = 0.0;
      
      auto joint_vel_comp = _ecm.Component<gz::sim::components::JointVelocity>(control.joint);
      if (!joint_vel_comp)
      {
        gzerr << "JointVelocity component missing for joint [" << control.jointName << "]\n";
        return;
      }

      const auto &velocities = joint_vel_comp->Data();
      if (velocities.empty())
      {
        gzerr << "Empty velocities for joint [" << control.jointName << "]\n";
        continue;
      }

      // current joint speed (rad/s)
      double currOmega = velocities[0];
      
      std::string topic = "/" + this->impl->topics[i];
      {
        std::lock_guard<std::mutex> lock(this->impl->pwmMutex);
        auto it = this->impl->pwmValues.find(topic);
        if (it != this->impl->pwmValues.end())
        {
          try
          {
            pwm = std::stod(it->second);
            // gzdbg << "Pwm received for channel [" << control.channel << "] is: [" << pwm << "]\n";
          }
          catch (const std::exception &e)
          {
            gzwarn << "Failed to convert PWM value for topic [" << topic << "]: " << e.what() << "\n";
            pwm = 0.0;
          }
        }
      }

      double targetOmega = pwm;


      if (std::abs(pwm) < 1.0) 
      {
        // gzdbg << "Zero rad/s PWM from topic, setting torque to 0\n";
        auto jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(control.joint);
        if (jfcComp)
        {
          auto &forceCmd = jfcComp->Data();
          if (!forceCmd.empty())
          {
            forceCmd[0] = 0.0;
          }
        }
        continue;
      }

      double dt =std::chrono::duration_cast<std::chrono::duration<double> >(
                  _info.simTime - this->impl->
          lastMotorModelUpdateTime).count();
      gzdbg << "Dt:- " << dt << "\n";
      double velError = targetOmega - currOmega;
      double desOmega = this->impl->controls[i].pid.Update(
          velError, std::chrono::duration<double>(dt)); 
      

      // Apply Drela's motor model equations
      
      // Equation (4): Current as function of speed and terminal voltage
      // i(Ω, v) = (v - Ω/KV) / R
      // Note: velocityConstant should be KV in rad/s/Volt

      double reqV = (desOmega / control.velocityConstant) +   
                            (control.noLoadCurrent * control.resistance);
      
      // Clamp voltage to available range
      double terminalV = std::max(-control.maxVolts, std::min(control.maxVolts, reqV));
      
      double backEmfV = currOmega / control.velocityConstant;  // Ω/KV
      double current = (terminalV - backEmfV) / control.resistance;

      // Equation (5): Qm = (i - io) / KQ
      double torque = (current - control.noLoadCurrent) * control.velocityConstant;
      
      // Debug output
      gzdbg << "Joint [" << control.jointName << "]:\n"
            << "  PWM: " << pwm << "\n"
            << "  Terminal Voltage: " << terminalV << " V\n"
            << "  Current Speed: " << currOmega << " rad/s (" << (currOmega * 60.0) / (2.0 * M_PI) << " RPM)\n"
            << "  Back-EMF: " << backEmfV << " V\n"
            << "  Current: " << current << " A\n"
            << "  Torque: " << torque << " Nm\n";

      // Apply torque to joint
      auto jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(control.joint);
      if (jfcComp)
      {
        auto &forceCmd = jfcComp->Data();
        forceCmd[0] = torque;
      }
      else
      {
        gzerr << "JointForceCmd component missing for joint [" << control.jointName << "]\n";
      }
    }
#endif
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::MotorPlugin,
    gz::sim::System,
    gz::sim::systems::MotorPlugin::ISystemPreUpdate,
    gz::sim::systems::MotorPlugin::ISystemConfigure,
    gz::sim::systems::MotorPlugin::ISystemConfigureParameters)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::MotorPlugin,
    "MotorPlugin")
