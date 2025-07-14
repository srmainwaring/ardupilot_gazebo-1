/*
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef MOTORPLUGIN_HH_
#define MOTORPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Motor plugin which can be attached to a model and is used to
/// control the velocity of a joint.
///
/// ## System Parameters:
///
/// - `<joint_name>` The name of the joint to be controlled.
///
/// - `<topic>` Topic to receive commands in. Defaults to
///     `/model/<model_name>/joint/<joint_name>/cmd_vel`.
///
/// ### Electro-mechanical properties
///
/// - `<voltage_max>` The maximum terminal voltage.
///
/// - `<speed_constant>` K_V the motor speed constant, relates the back-emf to
/// the motor angular velocity omega by the relation V_m = omega / K_V.
///
/// - `<coil_resistance>` R the motor internal resistance (Ohms)
///
/// - `<no_load_current>` i_0 the current draw when the motor is run at its
/// specificed voltage and operational RPM with no load.
///
/// ### Force control
///
/// The controller accepts the next optional parameters:
///
/// - `<p_gain>` The proportional gain of the PID.
/// The default value is 1.
///
/// - `<i_gain>` The integral gain of the PID.
/// The default value is 0.
///
/// - `<d_gain>` The derivative gain of the PID.
/// The default value is 0.
///
/// - `<i_max>` The integral upper limit of the PID.
/// The default value is 1.
///
/// - `<i_min>` The integral lower limit of the PID.
/// The default value is -1.
///
/// - `<cmd_max>` Output max value of the PID.
/// The default value is 1000.
///
/// - `<cmd_min>` Output min value of the PID.
/// The default value is -1000.
///
/// - `<cmd_offset>` Command offset (feed-forward) of the PID.
/// The default value is 0.
///
/// - `<disable_braking>` Disable braking. Allows the joint to freewheel
/// if the commanded velocity is below the actual velocity.
/// The default value is 0.
///
class MotorPlugin :
    public System,
    public ISystemPreUpdate,
    public ISystemConfigure,
    public ISystemConfigureParameters
{
  /// \brief Destructor
  public: virtual ~MotorPlugin();

  /// \brief Constructor
  public: MotorPlugin();

  // Documentation inherited
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         EntityComponentManager &_ecm) final;

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) final;

  // Documentation inherited
  public: void ConfigureParameters(
      gz::transport::parameters::ParametersRegistry &_registry,
      gz::sim::EntityComponentManager &_ecm) override;

  /// \internal
  /// \brief Private implementation
  private: class Impl;
  private: std::unique_ptr<Impl> impl;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // MOTORPLUGIN_HH_
