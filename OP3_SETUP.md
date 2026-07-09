# OP3 ArduHumanoid SITL Setup Guide

GSoC 2026 -- Neeta Misericordia

## Requirements

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic 8.11
- ArduPilot SITL (Python venv)
- GZ_IP=127.0.0.1 in ~/.bashrc (prevents multicast flooding WiFi)

## Repos

ArduPilot fork: github.com/Neetagrg/ardupilot branch neeta/ap-biomimetic
Gazebo bridge: github.com/Neetagrg/ardupilot_gazebo-1 branch neeta/op3-sitl

## Build ArduPilot

```bash
cd ~/ardupilot
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate
pip install -r requirements.txt
./waf configure --board sitl
./waf build --target bin/ardurover
```

## Environment setup

Add to ~/.bashrc:
```bash
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=/home/neeta-misericordia/ardupilot_gazebo/models:/home/neeta-misericordia/ardupilot_gazebo/worlds:/home/neeta-misericordia/humanoid-ardupilot-sitl/worlds:/home/neeta-misericordia/SITL_Models/Gazebo/models:/home/neeta-misericordia/SITL_Models/Gazebo/worlds:/home/neeta-misericordia/ros2_ws/install/ros_gz_h1_gazebo/share/ros_gz_h1_gazebo/worlds:/home/neeta-misericordia/ros2_ws/install/ros_gz_h1_description/share:/opt/ros/jazzy/share:/home/neeta-misericordia/ardupilot_gazebo/models:/home/neeta-misericordia/ardupilot_gazebo/worlds:/home/neeta-misericordia/humanoid-ardupilot-sitl/worlds:/home/neeta-misericordia/SITL_Models/Gazebo/models:/home/neeta-misericordia/SITL_Models/Gazebo/worlds::~/ardupilot_gazebo-1/models
```

## Running the simulation

Kill any previous instances first:
```bash
pkill -9 -f gz; pkill -9 -f ardurover; pkill -9 -f sim_vehicle; pkill -9 -f mavproxy; sleep 3
rm -rf ~/.gz/sim/log/
```

WARNING: Gazebo writes 100MB+ logs per session to ~/.gz/sim/log/.
Run the rm command above before every session or disk fills silently.

Terminal 1 -- Gazebo:
```bash
cd ~/ardupilot_gazebo-1 && gz sim -r worlds/op3_direct.sdf
```

Terminal 2 -- ardurover:
```bash
source ~/venv-ardupilot/bin/activate && cd ~/ardupilot
sim_vehicle.py -v Rover -f json --model JSON --console
```

After MANUAL> paste these params:
```
param set BIOM_HIP_P_STAND -3
param set BIOM_KNEE_STAND 15
param set BIOM_ANK_P_STAND 15
param set BIOM_GAIT_LEN -5
param set BIOM_GAIT_HGT 3
param set BIOM_GAIT_PERIOD 0.3
param set BIOM_STAND_RATE 50
param set CAN_P1_DRIVER 1
param set CAN_D1_PROTOCOL 1
param set LOG_DISARMED 1
reboot
```

After reconnect:
```
mode 20
```

Watch for AP_Biomimetic: standing in the console. Gait starts immediately after.

## Sim runs at real_time_factor=0.2

The SDF sets real_time_factor=0.2 so simulation runs at 1/5 real speed.
This is intentional -- the JPC controllers need small timesteps to stay stable.
GAIT_PERIOD=0.3 produces one full gait cycle every 1.5 real seconds.

## Known issues

Robot walks in a circular pattern rather than straight. Root cause is hip roll
phase timing in AP_Biomimetic::gait_step() -- fixing next session.

GAIT_LEN must be negative for forward walking due to sign convention in gait_step().
Positive values cause backward walking. Will be fixed in code so positive=forward.

## Debugging joints

Check if a joint is receiving commands while sim is running:
```bash
gz topic -e -t /op3/cmd_l_hip_pitch -d 5
gz topic -e -t /op3/cmd_r_hip_pitch -d 5
gz topic -e -t /op3/cmd_l_knee -d 5
```

If value is constant = gait not running or robot already fell.
If value alternates = gait cycling correctly.

## Channel mapping

AP_Biomimetic internal index to SDF channel:
- left:  hip_roll(0)->ch1, hip_yaw(1)->ch0, hip_pitch(2)->ch2, knee(3)->ch3, ank_pitch(4)->ch4, ank_roll(5)->ch5
- right: hip_roll(6)->ch7, hip_yaw(7)->ch6, hip_pitch(8)->ch8, knee(9)->ch9, ank_pitch(10)->ch10, ank_roll(11)->ch11
