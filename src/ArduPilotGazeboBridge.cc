/*
   Copyright (C) 2025 ardupilot.org

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

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Util.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/Utility.hh>

#include <gz/transport/Node.hh>

#include "SocketUDP.hh"
#include "Util.hh"

struct servo_packet_16 {
    uint16_t magic;         // 18458 expected magic value
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[16];
};

struct servo_packet_32 {
    uint16_t magic;         // 29569 expected magic value
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[32];
};

// Taken from ArduPilotPlugin
namespace
{
/// \brief Get a servo packet. Templated for 16 or 32 channel packets.
template<typename TServoPacket>
ssize_t getServoPacket(
  SocketUDP &_sock,
  const char *&_fcu_address,
  uint16_t &_fcu_port,
  uint32_t _wait_ms,
  const std::string &_model_name,
  TServoPacket &_pkt
)
{
    ssize_t recv_size = _sock.recv(&_pkt, sizeof(TServoPacket), _wait_ms);

    _sock.get_client_address(_fcu_address, _fcu_port);

    // drain the socket in the case we're backed up
    int counter = 0;
    while (true)
    {
        TServoPacket last_pkt;
        auto recv_size_last = _sock.recv(&last_pkt, sizeof(TServoPacket), 0ul);
        if (recv_size_last == -1)
        {
            break;
        }
        counter++;
        _pkt = last_pkt;
        recv_size = recv_size_last;
    }
    if (counter > 0)
    {
        std::cout << "[" << _model_name << "] "
                  << "Drained n packets: " << counter << "\n";
    }
    return recv_size;
}
}  // namespace


struct Control
{
  uint16_t channel{0};
  std::string type{"COMMAND"};
  bool use_force{true};
  std::string joint_name;
  std::string cmd_topic;
  double multiplier{1.0};
  double offset{0.0};
  double servo_max{2000.0};
  double servo_min{1000.0};
};


struct ArduPilotPlugin
{
  std::string fdm_address{"127.0.0.1"};
  uint16_t fdm_port{9002};
  uint16_t connection_timeout_max_count{5};
  bool lock_step{true};
  bool have_32_channels{false};
  gz::math::Pose3d pose_flu_to_frd;
  gz::math::Pose3d pose_enu_to_ned;
  std::string imu_name;
};


class ArduPilotGazeboBridge
{
public:
  ArduPilotGazeboBridge()
  {
    this->world_name = "runway";
    this->model_name = "r1_rover";
  }

  void configure()
  {
    // Load model
    this->_load_model();
    
    // Setup Gazebo connections
    this->_init_pubsub();

    // Setup SITL connections
    this->_init_sockets();
  }

  void run()
  {
    using namespace std::chrono_literals;

    uint64_t count = 0;
    while (true)
    {

      // Pre-update
      if (this->update_state == UpdateState::PREUPDATE)
      {
        // Publish actuator commands
        this->_recv_servo_packet();
        this->_send_commands();

        this->update_state = UpdateState::POSTUPDATE;
      }

      // Post-update
      if (this->update_state == UpdateState::POSTUPDATE)
      {
        // Create and send state to SITL
        this->_create_state_json();
        this->_send_state();

        this->update_state = UpdateState::PREUPDATE;
      }

      // Throttle update
      std::cout << "Step: " << count << std::endl;
      count++;
      std::this_thread::sleep_for(1ms);
    }
  }


private:
  void _load_model()
  {
    //! @todo replace hardcoded quadcopter example

    // Constants
    gz::math::Angle one_hundred_and_eighty_deg;
    gz::math::Angle ninety_deg;
    one_hundred_and_eighty_deg.SetDegree(180.0);
    ninety_deg.SetDegree(90.0);
    
    // Common settings
    this->plugin.connection_timeout_max_count = 5;
    this->plugin.lock_step = false;
    this->plugin.have_32_channels = false;
    this->plugin.pose_flu_to_frd = gz::math::Pose3d(
        0.0,
        0.0,
        0.0,
        one_hundred_and_eighty_deg.Radian(),
        0.0,
        0.0
    );
    this->plugin.pose_enu_to_ned = gz::math::Pose3d(
        0.0,
        0.0,
        0.0,
        one_hundred_and_eighty_deg.Radian(),
        0.0,
        ninety_deg.Radian()
    );

    if (this->model_name == "r1_rover")
    {
      this->plugin.imu_name = "base_link::imu_sensor";

      {
        auto &control = this->controls.emplace_back();
        control.channel = 0;
        control.type = "COMMAND";
        control.use_force = true;
        control.joint_name = "motor_0";
        control.cmd_topic = "/model/r1_rover/joint/motor_0/cmd_vel";
        control.multiplier = 46.3;
        control.offset = -0.5;
        control.servo_max = 2000.0;
        control.servo_min = 1000.0;
      }

      {
        auto &control = this->controls.emplace_back();
        control.channel = 0;
        control.type = "COMMAND";
        control.use_force = true;
        control.joint_name = "motor_1";
        control.cmd_topic = "/model/r1_rover/joint/motor_1/cmd_vel";
        control.multiplier = 46.3;
        control.offset = -0.5;
        control.servo_max = 2000.0;
        control.servo_min = 1000.0;
      }
      {
        auto &control = this->controls.emplace_back();
        control.channel = 2;
        control.type = "COMMAND";
        control.use_force = true;
        control.joint_name = "motor_0";
        control.cmd_topic = "/model/r1_rover/joint/motor_2/cmd_vel";
        control.multiplier = -46.3;
        control.offset = -0.5;
        control.servo_max = 2000.0;
        control.servo_min = 1000.0;
      }
      {
        auto &control = this->controls.emplace_back();
        control.channel = 2;
        control.type = "COMMAND";
        control.use_force = true;
        control.joint_name = "motor_1";
        control.cmd_topic = "/model/r1_rover/joint/motor_3/cmd_vel";
        control.multiplier = -46.3;
        control.offset = -0.5;
        control.servo_max = 2000.0;
        control.servo_min = 1000.0;
      }
    }
  }

  void _init_pubsub()
  {
    // gz.msgs.Clock
    // /world/iris_runway/clock
    this->clock_topic = std::string("/world/").append(this->world_name).append("/clock");
    this->node.Subscribe(this->clock_topic, &ArduPilotGazeboBridge::_clock_cb, this);
    std::cout << "Subscribing to Clock on: " << this->clock_topic << std::endl;

    // gz.msgs.Odometry
    // /model/iris_with_ardupilot_1/odometry
    this->odometry_topic = std::string("/model/").append(this->model_name).append("/odometry");
    this->node.Subscribe(this->odometry_topic, &ArduPilotGazeboBridge::_odometry_cb, this);
    std::cout << "Subscribing to Odometry on: " << this->odometry_topic << std::endl;

    // gz.msgs.Pose_V
    // /world/iris_runway/pose/info
    this->pose_info_topic = std::string("/world/").append(this->world_name).append("/pose/info");
    this->node.Subscribe(this->pose_info_topic, &ArduPilotGazeboBridge::_pose_info_cb, this);
    std::cout << "Subscribing to Pose_V on: " << this->pose_info_topic << std::endl;

    // gz.msgs.IMU
    // /world/runway/model/r1_rover/link/imu_link/sensor/imu_sensor/imu
    auto imu_split = gz::common::split(this->plugin.imu_name, "::");
    if (imu_split.size() == 2)
    {
      const auto &imu_link = imu_split[0];
      const auto &imu_sensor = imu_split[1];
      this->imu_topic = std::string("/world/").append(this->world_name)
          .append("/model/").append(this->model_name)
          .append("/link/").append(imu_link)
          .append("/sensor/").append(imu_sensor)
          .append("/imu");
    }
    if (imu_split.size() == 3)
    {
      const auto &imu_model = imu_split[0];
      const auto &imu_link = imu_split[1];
      const auto &imu_sensor = imu_split[2];
      this->imu_topic = std::string("/world/").append(this->world_name)
          .append("/model/").append(this->model_name)
          .append("/model/").append(imu_model)
          .append("/link/").append(imu_link)
          .append("/sensor/").append(imu_sensor)
          .append("/imu");
    }

    this->node.Subscribe(this->imu_topic, &ArduPilotGazeboBridge::_imu_cb, this);
    std::cout << "Subscribing to IMU on: " << this->imu_topic << std::endl;

    // Commands
    for (const auto &control : this->controls)
    {
      this->command_pubs.emplace_back(this->node.Advertise<gz::msgs::Double>(control.cmd_topic));
      std::cout << "Advertising command for channel " << control.channel
                << " on: " << control.cmd_topic << std::endl;      
    }
  }

  // Taken from ArduPilotPlugin
  void _init_sockets()
  {
    //! @todo move to _load_model
    this->fdm_address = this->plugin.fdm_address;
    this->fdm_port = this->plugin.fdm_port;
    this->is_lock_step = this->plugin.lock_step;
    this->connection_timeout_max_count = this->plugin.connection_timeout_max_count;
    this->have_32_channels = this->plugin.have_32_channels;

    if (!this->sock.bind(this->fdm_address.c_str(), this->fdm_port))
    {
      std::cout << "Model [" << this->model_name
                << "] failed to connect to ArduPilot SITL at "
                << this->fdm_address << ":" << this->fdm_port
                << std::endl;
    }
    else
    {
      std::cout << "Model [" << this->model_name
                << "] connecting to ArduPilot SITL at "
                << this->fdm_address << ":" << this->fdm_port
                << std::endl;
    }
  }

  bool _recv_servo_packet()
  {
    // Added detection for whether ArduPilot is online or not.
    // If ArduPilot is detected (receive of fdm packet from someone),
    // then socket receive wait time is increased from 1ms to 1 sec
    // to accomodate network jitter.
    // If ArduPilot is not detected, receive call blocks for 1ms
    // on each call.
    // Once ArduPilot presence is detected, it takes this many
    // missed receives before declaring the FCS offline.

    uint32_t wait_ms;
    if (this->ardupilot_online)
    {
        // Increase timeout for recv once we detect a packet from ArduPilot FCS.
        // If this value is too high then it will block the main Gazebo
        // update loop and adversely affect the RTF.
        wait_ms = 10;
    }
    else
    {
        // Otherwise skip quickly and do not set control force.
        wait_ms = 1;
    }

    // 16 / 32 channel compatibility
    uint16_t pkt_magic{0};
    uint16_t pkt_frame_rate{0};
    uint16_t pkt_frame_count{0};
    std::array<uint16_t, 32> pkt_pwm;
    ssize_t recv_size{-1};
    if (this->have_32_channels)
    {
      servo_packet_32 pkt;
      recv_size = getServoPacket(
          this->sock,
          this->fcu_address,
          this->fcu_port,
          wait_ms,
          this->model_name,
          pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }
    else
    {
      servo_packet_16 pkt;
      recv_size = getServoPacket(
          this->sock,
          this->fcu_address,
          this->fcu_port,
          wait_ms,
          this->model_name,
          pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }

    // didn't receive a packet, increment timeout count if online, then return
    if (recv_size == -1)
    {
        if (this->ardupilot_online)
        {
            if (++this->connection_timeout_count >
            this->plugin.connection_timeout_max_count)
            {
                this->connection_timeout_count = 0;

                // for lock-step resend last state rather than time out
                if (this->is_lock_step)
                {
                    this->_send_state();
                }
                else
                {
                    this->ardupilot_online = false;
                    std::cout << "[" << this->model_name << "] "
                        << "Broken ArduPilot connection,"
                        << " resetting motor control.\n";
                    //! @todo implement reset
                    // this->ResetPIDs();
                }
            }
        }
        return false;
    }

// #if DEBUG_JSON_IO
//     int max_servo_channels = this->have_32_channels ? 32 : 16;

//     // debug: inspect sitl packet
//     std::ostringstream oss;
//     oss << "recv " << recv_size << " bytes from "
//         << this->fcu_address << ":"
//         << this->fcu_port << "\n";
//     // oss << "magic: " << pkt_magic << "\n";
//     // oss << "frame_rate: " << pkt_frame_rate << "\n";
//     oss << "frame_count: " << pkt_frame_count << "\n";
//     // oss << "pwm: [";
//     // for (auto i=0; i<max_servo_channels - 1; ++i) {
//     //     oss << pkt_pwm[i] << ", ";
//     // }
//     // oss << pkt_pwm[max_servo_channels - 1] << "]\n";
//     std::cout << "\n" << oss.str() << std::endl;
// #endif

    // check magic, return if invalid
    constexpr uint16_t magic_16 = 18458;
    constexpr uint16_t magic_32 = 29569;
    uint16_t magic = this->have_32_channels ? magic_32 : magic_16;
    if (magic != pkt_magic)
    {
        std::cout << "Incorrect protocol magic "
                  << pkt_magic << " should be "
                  << magic << std::endl;
        return false;
    }

    // the controller is online
    if (!this->ardupilot_online)
    {
        this->ardupilot_online = true;

        std::cout << "[" << this->model_name << "] "
                  << "Connected to ArduPilot controller @ "
                  << this->fcu_address << ":" << this->fcu_port
                  << "\n";
    }

    // update frame rate
    this->fcu_frame_rate = pkt_frame_rate;

    // check for controller reset
    if (pkt_frame_count < this->fcu_frame_count)
    {
        //! @todo implement re-initialisation
        std::cout << "ArduPilot controller has reset" << std::endl;
    }

    // check for duplicate frame
    else if (pkt_frame_count == this->fcu_frame_count)
    {
        std::cout << "Duplicate input frame" << std::endl;

        // for lock-step resend last state rather than ignore
        if (this->is_lock_step)
        {
            this->_send_state();
        }

        return false;
    }

    // check for skipped frames
    else if (pkt_frame_count != this->fcu_frame_count + 1
        && this->ardupilot_online)
    {
        std::cout << "Missed "
            << pkt_frame_count - this->fcu_frame_count
            << " input frames" << std::endl;
    }

    // update frame count
    this->fcu_frame_count = pkt_frame_count;

    // reset the connection timeout so we don't accumulate
    this->connection_timeout_count = 0;

    //! @todo implement
    // this->UpdateMotorCommands(pkt_pwm);

    return true;
  }

  void _send_commands()
  {
    
  }

  void _create_state_json()
  {
    gz::msgs::IMU imu_msg;
    {
        std::lock_guard<std::mutex> lock(this->imu_mutex);
        // Wait until we've received a valid message.
        // if (!this->imu_msg_valid)
        // {
        //     return;
        // }
        imu_msg = this->imu_msg;
    }
    
    // Simulation time
    double timestamp = this->clock_msg.sim().sec()
        + 1.9e-9 * this->clock_msg.sim().nsec();

    //! @todo allow other IMU orientations
    //! @note it is assumed that the imu orientation conforms to the
    // FRD aircraft convention:
    //   x-forward
    //   y-right
    //   z-down

    // Linear acceleration
    gz::math::Vector3d linearAccel{
        imu_msg.linear_acceleration().x(),
        imu_msg.linear_acceleration().y(),
        imu_msg.linear_acceleration().z(),
    };

    // Linear acceleration
    gz::math::Vector3d angularVel{
        imu_msg.angular_velocity().x(),
        imu_msg.angular_velocity().y(),
        imu_msg.angular_velocity().z(),
    };


    gz::math::Pose3d wldAToBdyA;
    gz::math::Vector3d velWldA;

    // build JSON document
    rapidjson::StringBuffer string_buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(string_buf);

    writer.StartObject();

    writer.Key("timestamp");
    writer.Double(timestamp);

    writer.Key("imu");
    writer.StartObject();
    writer.Key("gyro");
    writer.StartArray();
    writer.Double(angularVel.X());
    writer.Double(angularVel.Y());
    writer.Double(angularVel.Z());
    writer.EndArray();
    writer.Key("accel_body");
    writer.StartArray();
    writer.Double(linearAccel.X());
    writer.Double(linearAccel.Y());
    writer.Double(linearAccel.Z());
    writer.EndArray();
    writer.EndObject();

    writer.Key("position");
    writer.StartArray();
    writer.Double(wldAToBdyA.Pos().X());
    writer.Double(wldAToBdyA.Pos().Y());
    writer.Double(wldAToBdyA.Pos().Z());
    writer.EndArray();

    // ArduPilot quaternion convention: q[0] = 1 for identity.
    writer.Key("quaternion");
    writer.StartArray();
    writer.Double(wldAToBdyA.Rot().W());
    writer.Double(wldAToBdyA.Rot().X());
    writer.Double(wldAToBdyA.Rot().Y());
    writer.Double(wldAToBdyA.Rot().Z());
    writer.EndArray();

    writer.Key("velocity");
    writer.StartArray();
    writer.Double(velWldA.X());
    writer.Double(velWldA.Y());
    writer.Double(velWldA.Z());
    writer.EndArray();

    // Ignore range sensor
    // Ignore wind sensor

    writer.EndObject();

    // Get JSON string
    this->json_data = "\n" + std::string(string_buf.GetString()) + "\n";
    std::cout << this->json_data << std::endl;
  }

  void _send_state()
  {
    auto bytes_sent = this->sock.sendto(
        this->json_data.c_str(),
        this->json_data.size(),
        this->fcu_address,
        this->fcu_port);

    std::cout << "Sent " << bytes_sent <<  " bytes to "
              << this->fcu_address << ":"
              << this->fcu_port << std::endl;
              // << "frame_count: " << this->frame_count
              // << std::endl;

  }

  void _clock_cb(const gz::msgs::Clock &_msg)
  {
    std::lock_guard<std::mutex> lock(this->clock_mutex);
    this->clock_msg = _msg;
    // std::cout << this->clock_msg.DebugString() << std::endl;
  }

  void _imu_cb(const gz::msgs::IMU &_msg)
  {
    std::lock_guard<std::mutex> lock(this->imu_mutex);
    this->imu_msg = _msg;
    // std::cout << this->imu_msg.DebugString() << std::endl;
  }

  void _odometry_cb(const gz::msgs::Odometry &_msg)
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex);
    this->odometry_msg = _msg;
    // std::cout << this->odometry_msg.DebugString() << std::endl;
  }

  void _pose_info_cb(const gz::msgs::Pose_V &_msg)
  {
    std::lock_guard<std::mutex> lock(this->pose_info_mutex);
    this->pose_info_msg = _msg;
    // std::cout << this->pose_info_msg.DebugString() << std::endl;
  }

  enum class UpdateState
  {
    PREUPDATE = 0,
    POSTUPDATE = 1
  };

  // Configuration
  std::string world_name;
  std::string model_name;

  // Socket address - flight dynamics model (physics)
  std::string fdm_address{"127.0.0.1"};
  uint16_t fdm_port{9002};
  // Socket address - flight contrioller unit (SITL)
  // std::string fcu_address;
  const char* fcu_address{nullptr};
  uint16_t fcu_port;

  bool is_lock_step{false};
  bool have_32_channels{false};
  uint16_t connection_timeout_max_count{0};
  uint16_t connection_timeout_count{0};
  uint16_t fcu_frame_rate;
  uint32_t fcu_frame_count{static_cast<uint32_t>(-1)};

  // Service call timeout
  uint32_t timeout;

  // Connection
  SocketUDP sock = SocketUDP(true, true);  
  bool connected = false;
  uint32_t last_sitl_frame;
  bool ardupilot_online{false};

  // Packet
  servo_packet_16 servo_packet;

  // Stats
  uint32_t frame_count{0};
  uint32_t print_frame_count{0};
  double pre_update_prev_time;
  double post_update_prev_time;

  // Payload
  std::string json_data;

  // Transport
  gz::transport::Node node;
  ArduPilotPlugin plugin;
  std::vector<Control> controls;
  std::vector<gz::transport::Node::Publisher> command_pubs;

  gz::msgs::Clock clock_msg;
  std::string clock_topic;
  std::mutex clock_mutex;
  std::chrono::steady_clock::duration clock_prev_recv_time{0};

  gz::msgs::IMU imu_msg;
  std::string imu_topic;
  std::mutex imu_mutex;
  std::chrono::steady_clock::duration imu_prev_recv_time{0};

  gz::msgs::Odometry odometry_msg;
  std::string odometry_topic;
  std::mutex odometry_mutex;
  std::chrono::steady_clock::duration odometry_prev_recv_time{0};

  gz::msgs::Pose_V pose_info_msg;
  std::string pose_info_topic;
  std::mutex pose_info_mutex;
  std::chrono::steady_clock::duration pose_info_prev_recv_time{0};

  // Track update state
  UpdateState update_state{UpdateState::PREUPDATE};
};

int main(int argc, char** argv)
{
  std::cout << "ArduPilot Gazebo Bridge" << std::endl;

  //! @todo replace hardcoded configuration
  std::string model_name = "r1_rover";
  std::string world_name = "runway";
  
  ArduPilotGazeboBridge gz_bridge;
  gz_bridge.configure();
  gz_bridge.run();

  return 0;
}