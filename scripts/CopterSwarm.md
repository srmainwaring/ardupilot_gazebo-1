# Copter Swarm

A python script to launch a swarm of copters in Gazebo running ArduPilot SITL.

Services

```bash
/gazebo/resource_paths/add
/gazebo/resource_paths/get
/gazebo/resource_paths/resolve
/gazebo/worlds
/gui/camera/view_control
/gui/camera/view_control/reference_visual
/gui/camera/view_control/sensitivity
/gui/copy
/gui/move_to
/gui/move_to/pose
/gui/paste
/gui/screenshot
/gui/view/collisions
/gui/view/com
/gui/view/frames
/gui/view/inertia
/gui/view/joints
/gui/view/transparent
/gui/view/wireframes
/marker
/marker/list
/marker_array
/server_control
/world/runway/control
/world/runway/control/state
/world/runway/create
/world/runway/create/blocking
/world/runway/create_multiple
/world/runway/create_multiple/blocking
/world/runway/declare_parameter
/world/runway/disable_collision
/world/runway/disable_collision/blocking
/world/runway/enable_collision
/world/runway/enable_collision/blocking
/world/runway/entity/system/add
/world/runway/generate_world_sdf
/world/runway/get_parameter
/world/runway/gui/info
/world/runway/level/set_performer
/world/runway/light_config
/world/runway/light_config/blocking
/world/runway/list_parameters
/world/runway/model/axes/link/link/sensor/navsat_sensor/navsat/set_rate
/world/runway/playback/control
/world/runway/remove
/world/runway/remove/blocking
/world/runway/scene/graph
/world/runway/scene/info
/world/runway/set_parameter
/world/runway/set_physics
/world/runway/set_physics/blocking
/world/runway/set_pose
/world/runway/set_pose/blocking
/world/runway/set_pose_vector
/world/runway/set_pose_vector/blocking
/world/runway/set_spherical_coordinates
/world/runway/set_spherical_coordinates/blocking
/world/runway/state
/world/runway/state_async
/world/runway/system/info
/world/runway/visual_config
/world/runway/visual_config/blocking
/world/runway/wheel_slip
/world/runway/wheel_slip/blocking
```

```bash
gz service -i -s /world/runway/create
Service providers [Address, Request Message Type, Response Message Type]:
  tcp://127.0.0.1:55736, gz.msgs.EntityFactory, gz.msgs.Boolean
```

## Run Gazebo - no levels

```bash
gz sim -v4 -s -r runway.sdf
```

```bash
gz sim -v4 -g
```

```bash
python ./src/ardupilot_gazebo/scripts/copter_swarm.py --world runway --model iris_with_ardupilot --x-count 4 --y-count 1 --x-offset 10 --y-offset 10
```

## Run SITL

Following the guide on the dev wiki: [Using SITL¶](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)

Using the `--swarm` argument in `sim_vehicle.py`

Create a directory for running the swarm

```bash
mkdir swarm && cd swarm
```

Edit a file called `swarminit-4x1.txt` to contain:

```bash
#instance=East,North,Up(Offset in meters),Absolute Heading
1=0,0,0,90
2=1,0,0,90
3=2,0,0,90
4=3,0,0,90
```

Start SITL using the `--swarm` argument

```bash
sim_vehicle.py --debug -v Copter -f quad --count 4 --auto-sysid --location CMAC --swarm ./swarminit-4x1.txt --console --map
```

## Run Gazebo - distributed with levels

```bash
gz sim -v4 -s -r --levels --network-role primary  --network-secondaries 1 runway.sdf
```

```bash
gz sim -v4 -s -r --levels --network-role secondary runway.sdf
```

```bash
gz sim -v4 -g
```

```bash
python ./src/ardupilot_gazebo/scripts/copter_swarm.py --world runway --model iris_with_ardupilot --x-count 4 --y-count 1 --x-offset 10 --y-offset 10
```

### Issues

Statically defined models

- IMU sensor causing segmentation fault
- Fix - add check before dereferencing pointer Imu.cc, ImuPrivate::AddSensor, L:233
- Why is this not raising an exception in the non-distributed case?
- 

Dynamically defined models

- Not found on secondary

SITL Swarm

- Not working for `--count 1` - error loading params?

Gazebo

- What happens to the socket when a level is unloaded?

Levels

- Don't work well if non-performer entities are loaded dynamically
- GUI crashed as levels loaded / unloaded (repeatable?)
- Level tiles loading and unloading correctly if defined statically

Test 1:

```bash
ros2-ardupilot % gz sim -v4 -s -r --levels runway.sdf
```

- number of copters: 1
- dynamically load copters: no
- dynamically load performers: no
- number of levels: 25
- dynamically load level visuals: no
- distributed: no
- number of secondaries: 0
- status: ok

Test 2:

```bash
gz sim -v4 -s -r --levels --network-role primary --network-secondaries 0 runway.sdf
```

- number of copters: 1
- dynamically load copters: no
- dynamically load performers: no
- number of levels: 25
- dynamically load level visuals: no
- distributed: yes
- number of secondaries: 0
- status: ok

Test 3:

```bash
gz sim -v4 -s -r --levels --network-role primary --network-secondaries 1 runway.sdf
```

```bash
gz sim -v4 -s -r --levels --network-role secondary runway.sdf
```

- number of copters: 1
- dynamically load copters: no
- dynamically load performers: no
- number of levels: 25
- dynamically load level visuals: no
- distributed: yes
- number of secondaries: 1
- status: fail

primary

```bash
(2025-11-04 19:53:15.353) [debug] [SystemManager.cc:80] Loaded system [ArduPilotPlugin] for entity [43]
...
(2025-11-04 19:53:15.980) [info] [ArduPilotPlugin.cc:1147] Found IMU sensor with name [iris_with_standoffs::imu_link::imu_sensor]
(2025-11-04 19:53:15.980) [debug] [ArduPilotPlugin.cc:1161] Computed IMU topic to be: world/runway/model/iris_with_ardupilot_1/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
```

secondary

```bash
SocketUDP Bind failed: Address already in use
(2025-11-04 19:53:15.877) [error] [ArduPilotPlugin.cc:1282] [iris_with_ardupilot_1] failed to bind with 127.0.0.1:9002 aborting plugin.
...
(2025-11-04 19:53:16.598) [warning] [ArduPilotPlugin.cc:1568] Incorrect protocol magic 255 should be 18458
```

Sockets are going to present a problem - we cannot load a model twice using the same port. The binding occurs at model load, even though the entity is not loaded, the plugin is bound.

Need to seek an alternative approach...

- Use a proxy model to load the plugin, but assign outputs to the actual model.
- Still have the same problem - the model is loaded into all `gz sim instances`
  - unless we also have two separate worlds, one with the proxy, and one without
  - issue is that the entity ids will then be different - so inconsistent.

- Alternative settings for UDP sockets?
  - Can use `SO_REUSEPORT` and allow multiple sockets to bind to the same port, however this causes other errors at runtime, as processes where the entity is unloaded continue to send (invalid) data.
  - Can we check if an entity is loaded, and disable the plugin if that is the case?
  - Or use an entirely gz-transport based approach (c.f. PX4).



How does PX4 manage this?

- PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.hpp

GZBridge: 

Interface

- Class `GZBridge` constructed with world and model names.
- Subscriptions to topics
  - Clock
    - used to synchronise the FC clock with Gazebo
  - PoseInfo -> ground truth
    - pos_w
    - rot_w
    - vel_w (calculated)
    - acc_w (calculated)
  - Imu -> body lin_acc and ang_vel
    - accel_b = flu_to_frd.rotate(msg.linear_acceleration)
    - gyro_b  = flu_to_frd.rotate(msg.angular_velocity)
  - Mag
  - Odometry -> odometry
  - LaserScan
  - DistanceSensor
  - AirSpeed
  - AirPressure
  - NavSat
  - OpticalFlow
- Mixing interface objects are used for
  - Motors
  - Servos
  - Wheels (why different to motors?)
  - Gimbal
- Facility for adding noise (why not just use physics engine?)
- Separate object to handle one gimbal

Implementation

- init
  - subscribe to all topics
- Run
  - updates parameters
- subscribeXXX
  - Only support one IMU and must be on /link/base_link/sensor/imu_sensor/imu
  - ditto magnetometer
  - similarly for other sensors which have hardcoded link and sensor names
- clockCallback
  - keeps PX4 RT clock synchronised with the sim time
- sensor callbacks
  - Read messages, update devices directly
- Can't see any mechanism for maintaining lockstep?

MixingInterfaceESC : OutputModuleInterface

Interface

- MixingOutput _mixing_output

- init
  - fixed topic name /{model_name}/command/motor_speed
- updateOutputs
  - publish gz::msg::Actuators

## Notes

- Change iris_with_ardupilot to use JointController
- Publish commands from the ArduPilotPlugin
- Topic name /model/{model}/joint/{joint_name}/cmd_vel
- Consider using /model/{model}/joint/{joint_name}/command/motor_speed to standardise (PX4)?

- Why does sim_time not reset when server restarts if the gui is running?

## Lock Step Plugin

Message order

```bash
LockStepPlugin: Received Enable Lock Step: [1]
LockStepPlugin: Initiate Lock-Stepping

LockStepPlugin: Received Lock Step Start [sec, nsec]
LockStepPlugin: PreUpdate Complete
LockStepPlugin: Send Lock Step Complete [sec, nsec]
LockStepPlugin: PostUpdate Complete
```

Working, but slow. The issue is that plugins are run by each secondary...
So we are back to the same problem that we have with the ArduPilot Plugin.






### Fail to load default parameters

- Path to defaults not modified for vehicles requiring additional parameters
- Only the first item in the path is altered.
- Error occurs in the SITL binary: failed to find default file ... => dumpstack
- `sim_vehicle.py` : `L778`

### Fail to set rc override

- MAVProxy rc override is always sent to the vehicle with the highest sysid
