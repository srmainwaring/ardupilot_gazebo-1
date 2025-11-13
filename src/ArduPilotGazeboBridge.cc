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
#include <vector>

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

struct Control
{
  uint16_t channel{0};
  std::string type{"COMMAND"};
  bool use_force{true};
  std::string joint_name;
  std::string cmd_topic;
  double mutliplier{1.0};
  double offset{0.0};
  double servo_max{2000.0};
  double servo_min{1000.0};
};

struct ArduPilotPlugin
{
  std::string address{"127.0.0.1"};
  uint16_t port{9002};
  uint16_t connection_timeout_max_count{5};
  bool lock_step{true};
  bool have_32_channels{false};
  gz::math::Pose3d pose_flu_to_frd = gz::math::Pose3d(0, 0, 0, 0, 0, 0);
  gz::math::Pose3d pose_enu_to_ned = gz::math::Pose3d(0, 0, 0, 0, 0, 0);
  std::string imu_name;
};


class ArduPilotGazeboBridge
{
public:
  ArduPilotGazeboBridge() {}


private:
  void _clock_cb(const gz::msgs::Clock &_msg)
  {
    std::lock_guard<std::mutex> lock(this->clock_mutex);
    this->clock_msg = _msg;
  }

  void _imu_cb(const gz::msgs::IMU &_msg)
  {
    std::lock_guard<std::mutex> lock(this->imu_mutex);
    this->imu_msg = _msg;
  }

  void _odometry_cb(const gz::msgs::Odometry &_msg)
  {
    std::lock_guard<std::mutex> lock(this->odometry_mutex);
    this->odometry_msg = _msg;
  }

  void _pose_info_cb(const gz::msgs::Pose_V &_msg)
  {
    std::lock_guard<std::mutex> lock(this->pose_info_mutex);
    this->pose_info_msg = _msg;
  }

  // Configuration
  std::string world_name;
  std::string model_name;

  // Socket settings
  std::string address;
  uint16_t port;

  // Service call timeout
  uint32_t timeout;

  // Connection
  SocketUDP sock = SocketUDP(true, true);  
  bool connected = false;
  uint32_t last_sitl_frame;

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
  std::string clock_sub;

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
};


int main(int argc, char** argv)
{
  std::cout << "ArduPilot Gazebo Bridge" << std::endl;

  //! @todo replace hardcoded configuration
  std::string model_name = "r1_rover";
  std::string world_name = "runway";
  std::string address = "127.0.0.1";
  uint16_t port = 9002;
  uint32_t timeout = 500;
  

  gz::transport::Node node;


  return 0;
}