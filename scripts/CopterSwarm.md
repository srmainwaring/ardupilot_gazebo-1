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