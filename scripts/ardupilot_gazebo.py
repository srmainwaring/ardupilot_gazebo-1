"""
ArduPilot Gazebo Bridge - carry out similar functions to the ArduPilotPlugin

Based on example: ardupilot/libraries/SITL/examples/JSON/pybullet/robot.py

Example usage

python ./src/ardupilot_gazebo/scripts/ardupilot_gazebo.py --world iris_runway --model iris_with_ardupilot_1
"""

import copy
import json
import math
import numpy as np
import os
import socket
import struct
import time

from dataclasses import dataclass
from dataclasses import field
from argparse import ArgumentParser
from pathlib import Path
from threading import Lock
from transforms3d import euler

GZ_VERSION_GARDEN = "garden"
GZ_VERSION_HARMONIC = "harmonic"
GZ_VERSION_IONIC = "ionic"
GZ_VERSION_JETTY = "jetty"

# Index convention for transforms3d quaternions
QUAT_IDX_W = 0
QUAT_IDX_X = 1
QUAT_IDX_Y = 2
QUAT_IDX_Z = 3

# Constants
MAGIC = 18458
RATE_HZ = 1000
TIME_STEP = 1.0 / RATE_HZ


def gz_version():
    """Return the environment variable GZ_VERSION if set, else default to 'harmonic'"""
    return os.environ.get("GZ_VERSION", GZ_VERSION_HARMONIC)


if gz_version() == GZ_VERSION_JETTY:
    from gz.math import Pose3d
    from gz.math import Quaterniond
    from gz.math import Vector3d
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.clock_pb2 import Clock
    from gz.msgs.double_pb2 import Double
    from gz.msgs.entity_factory_pb2 import EntityFactory
    from gz.msgs.entity_factory_v_pb2 import EntityFactory_V
    from gz.msgs.imu_pb2 import IMU
    from gz.msgs.model_pb2 import Model
    from gz.msgs.pose_pb2 import Pose
    from gz.msgs.pose_v_pb2 import Pose_V

    # from gz.msgs.quaternion_pb2 import Quaternion
    from gz.msgs.stringmsg_pb2 import StringMsg


# Importing gz.transport into the module global scope causes an odd
# multiprocessing conflict with dronecan. This is a workaround.
def gz_node():
    if gz_version() == GZ_VERSION_JETTY:
        from gz.transport import Node

    return Node()


@dataclass
class Control:
    channel: int = 0
    type: str = "COMMAND"
    use_force: bool = True
    joint_name: str = ""
    cmd_topic: str = ""
    multiplier: float = 1.0
    offset: float = 0.0
    servo_min: float = 1000.0
    servo_max: float = 2000.0


@dataclass
class ArduPilotPlugin:
    fdm_addr: str = ""
    fdm_port: int = 9002
    connection_timeout_max_count: int = 5
    lock_step: bool = False
    have_32_channels: bool = False
    flu_to_frd: list[float] = field(default_factory=lambda: [0, 0, 0, 180, 0, 0])
    enu_to_ned: list[float] = field(default_factory=lambda: [0, 0, 0, 180, 0, 90])
    imu_name: str = ""


class ArduPilotGazeboBridge:
    """Python port of ArduPilotPlugin.hh"""

    def __init__(self, args):
        # Configuration
        self.world_name = args.world
        self.model_name = args.model

        # Socket settings
        self.address = args.address
        self.port = args.port

        # Service call timeout
        self.timeout = args.timeout

        # Connection
        self.sock = None
        self.last_sitl_frame = -1
        self.connected = False

        # Packet
        self.pwm = None

        # Stats
        self.frame_count = 0
        self.frame_time = time.monotonic()
        self.print_frame_count = 10000

        # Payload
        self.json_data = None
        self.start_time = time.monotonic()

        # Transport
        self.node = gz_node()
        self.plugin = None
        self.controls = []
        self.command_pubs = []

        self.clock_lock = Lock()
        self.clock_msg = Clock()
        self.clock_sub = None
        self.clock_topic = ""

        self.imu_lock = Lock()
        self.imu_msg = IMU()
        self.imu_sub = None
        self.imu_topic = ""

        self.model_lock = Lock()
        self.model_msg = IMU()
        self.model_sub = None
        self.model_topic = ""

        self.pose_info_lock = Lock()
        self.pose_info_msg = Pose_V()
        self.pose_info_sub = None
        self.pose_info_topic = ""

    def configure(self):
        # Load model
        self._load_model()

        # Setup Gazebo connections
        self._init_pubsub()

        # Setup SITL connection
        self._init_sockets()

    def run(self):
        global MAGIC
        global RATE_HZ
        global TIME_STEP

        while True:
            # time.sleep(0.1)
            # continue

            # pre-update
            # TODO
            # self._receive_servo_packet()
            try:
                data, address = self.sock.recvfrom(100)
            except Exception:
                time.sleep(0.01)
                continue

            parse_format = "HHI16H"
            if len(data) != struct.calcsize(parse_format):
                print(f"Bad packet size: {len(data)}")
                continue

            decoded = struct.unpack(parse_format, data)
            magic = MAGIC
            if decoded[0] != magic:
                print(f"Incorrect magic: {decoded[0]}")
                continue

            frame_rate_hz = decoded[1]
            frame_number = decoded[2]
            self.pwm = decoded[3:]

            if frame_rate_hz != RATE_HZ:
                RATE_HZ = frame_rate_hz
                TIME_STEP = 1.0 / RATE_HZ
                # p.setTimeStep(TIME_STEP)
                # print(f"Updated rate to {RATE_HZ} Hz")

            if frame_number < self.last_sitl_frame:
                # vehicle.reset()
                time_now = 0.0
                print("Controller reset")
            elif frame_number != self.last_sitl_frame + 1 and self.connected:
                print(f"Missed {frame_number - self.last_sitl_frame - 1} frames")

            self.last_sitl_frame = frame_number

            if not self.connected:
                self.connected = True
                print(f"Connected to {address}")

            self.frame_count += 1

            # publish actuator commands
            self._send_commands()

            # TODO how to wait for the physics update step to complete?

            # post-update
            self._create_state_json()
            self._send_state(address)

            # TODO keep for reference
            # phys_time = time.monotonic() - self.start_time
            # gyro = [0.0, 0.0, 0.0]
            # accel = [0.0, 0.0, 0.0]
            # pos = [0.0, 0.0, 0.0]
            # vel = [0.0, 0.0, 0.0]
            # quat = [0.0, 0.0, 0.0, 1.0]
            #
            # json_data = {
            #     "timestamp": phys_time,
            #     "imu": {"gyro": gyro, "accel_body": accel},
            #     "position": pos,
            #     "quaternion": quat,
            #     "velocity": vel,
            # }
            #
            # self.sock.sendto(
            #     (json.dumps(json_data, separators=(",", ":")) + "\n").encode("ascii"),
            #     address,
            # )

            if self.frame_count % self.print_frame_count == 0:
                phys_time = time.monotonic() - self.start_time
                now = time.time()
                total_time = now - self.frame_time
                print(
                    f"{self.print_frame_count/total_time:.2f} "
                    f"fps T={phys_time:.3f} "
                    f"dt={total_time:.3f}"
                )
                print(f"pwm: {self.pwm}")

                self.frame_time = now

    def _load_model(self):
        # TODO replace hardcoded quadcopter example
        self.plugin = ArduPilotPlugin()
        self.plugin.fdm_addr = self.address
        self.plugin.fdm_port = self.port
        self.plugin.connection_timeout_max_count
        self.plugin.lock_step = False
        self.plugin.have_32_channels = False
        self.plugin.flu_to_frd = [
            0,
            0,
            0,
            np.radians(180),
            np.radians(0),
            np.radians(0),
        ]
        self.plugin.enu_to_ned = [
            0,
            0,
            0,
            np.radians(180),
            np.radians(0),
            np.radians(90),
        ]
        self.plugin.imu_name = "iris_with_standoffs::imu_link::imu_sensor"

        control = Control()
        control.channel = 0
        control.type = "COMMAND"
        control.use_force = True
        control.joint_name = "iris_with_standoffs::rotor_0_joint"
        control.cmd_topic = f"/model/iris_with_ardupilot_1/joint/rotor_0_joint/cmd_vel"
        control.multiplier = 838
        control.offset = 0.0
        control.servo_max = 2000
        control.servo_min = 1000
        self.controls.append(control)

        control = Control()
        control.channel = 1
        control.type = "COMMAND"
        control.use_force = True
        control.joint_name = "iris_with_standoffs::rotor_1_joint"
        control.cmd_topic = "/model/iris_with_ardupilot_1/joint/rotor_1_joint/cmd_vel"
        control.multiplier = 838
        control.offset = 0.0
        control.servo_max = 2000
        control.servo_min = 1000
        self.controls.append(control)

        control = Control()
        control.channel = 2
        control.type = "COMMAND"
        control.use_force = True
        control.joint_name = "iris_with_standoffs::rotor_2_joint"
        control.cmd_topic = "/model/iris_with_ardupilot_1/joint/rotor_2_joint/cmd_vel"
        control.multiplier = -838
        control.offset = 0.0
        control.servo_max = 2000
        control.servo_min = 1000
        self.controls.append(control)

        control = Control()
        control.channel = 3
        control.type = "COMMAND"
        control.use_force = True
        control.joint_name = "iris_with_standoffs::rotor_3_joint"
        control.cmd_topic = "/model/iris_with_ardupilot_1/joint/rotor_3_joint/cmd_vel"
        control.multiplier = -838
        control.offset = 0.0
        control.servo_max = 2000
        control.servo_min = 1000
        self.controls.append(control)

    def _init_pubsub(self):
        # gz.msgs.Clock
        # /world/iris_runway/clock
        self.clock_topic = f"/world/{self.world_name}/clock"
        self.clock_sub = self.node.subscribe(Clock, self.clock_topic, self._clock_cb)
        print(f"Subscribing to Clock on: {self.clock_topic}")

        # gz.msgs.Pose_V
        # /world/iris_runway/dynamic_pose/info
        self.pose_info_topic = f"/world/{self.world_name}/dynamic_pose/info"
        self.pose_info_sub = self.node.subscribe(
            Pose_V, self.pose_info_topic, self._pose_info_cb
        )
        print(f"Subscribing to Pose_V on: {self.pose_info_topic}")

        # gz.msgs.Model
        # /world/iris_runway/model/iris_with_ardupilot_1/joint_state
        self.model_topic = (
            f"/world/{self.world_name}" f"/model/{self.model_name}" f"/joint_state"
        )
        self.model_sub = self.node.subscribe(Model, self.model_topic, self._model_cb)
        print(f"Subscribing to Model on: {self.model_topic}")

        # gz.msgs.IMU
        # iris_with_standoffs::imu_link::imu_sensor
        # /world/iris_runway/model/iris_with_ardupilot_1/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
        imu_split = self.plugin.imu_name.split("::")
        imu_model = imu_split[0]
        imu_link = imu_split[1]
        imu_sensor = imu_split[2]
        self.imu_topic = (
            f"/world/{self.world_name}"
            f"/model/{self.model_name}"
            f"/model/{imu_model}"
            f"/link/{imu_link}"
            f"/sensor/{imu_sensor}/imu"
        )
        self.imu_sub = self.node.subscribe(IMU, self.imu_topic, self._imu_cb)
        print(f"Subscribing to IMU on: {self.imu_topic}")

        # commands
        for control in self.controls:
            self.command_pubs.append(self.node.advertise(control.cmd_topic, Double))
            print(
                f"Advertising command for channel {control.channel} "
                f"on {control.cmd_topic}"
            )

    def _init_sockets(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.address, self.port))
        self.sock.settimeout(0.1)

    def _receive_servo_packet(self):
        pass
        # try:
        #     data, address = self.sock.recvfrom(100)
        # except Exception:
        #     time.sleep(0.01)
        #     continue

    def _send_commands(self):
        for i in range(len(self.controls)):
            control = self.controls[i]
            pub = self.command_pubs[i]
            pwm_i = self.pwm[i]
            pwm_max = control.servo_max
            pwm_min = control.servo_min
            multiplier = control.multiplier
            offset = control.offset

            # FC initialising
            if pwm_i == 0:
                continue

            normalised_cmd = (pwm_i - pwm_min) / (pwm_max - pwm_min)
            normalised_cmd = max(min(normalised_cmd, 1.0), 0.0)
            cmd = multiplier * (normalised_cmd + offset)

            cmd_msg = Double()
            cmd_msg.data = cmd
            pub.publish(cmd_msg)
            # /model/iris_with_ardupilot_1/joint/rotor_3_joint/cmd_vel
            # print(f"Publish command {cmd_msg.data} for channel: {i}")

    def _create_state_json(self):
        lin_acc = np.zeros(3)
        ang_vel = np.zeros(3)
        with self.imu_lock:
            lin_acc = Vector3d(
                self.imu_msg.linear_acceleration.x,
                self.imu_msg.linear_acceleration.y,
                self.imu_msg.linear_acceleration.z,
            )
            ang_vel = Vector3d(
                self.imu_msg.angular_velocity.x,
                self.imu_msg.angular_velocity.y,
                self.imu_msg.angular_velocity.z,
            )

        # Pose in Gazebo world frame
        world_pose = None
        for pose in self.pose_info_msg.pose:
            # Filter for the model
            if pose.name == self.model_name:
                world_pos = Vector3d(pose.position.x, pose.position.y, pose.position.z)
                world_rot = Quaterniond(
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
                world_pose = Pose3d(world_pos, world_rot)

        # Velocity in Gazebo world frame
        # TODO: hardcoded velocity
        world_lin_vel = Vector3d(0, 0, 0)
        # print(f"world_lin_vel: {world_lin_vel}")

        bdy_G_to_bdy_A = Pose3d(
            self.plugin.flu_to_frd[0],
            self.plugin.flu_to_frd[1],
            self.plugin.flu_to_frd[2],
            self.plugin.flu_to_frd[3],
            self.plugin.flu_to_frd[4],
            self.plugin.flu_to_frd[5],
        )
        bdy_A_to_bdy_G = bdy_G_to_bdy_A.inverse()
        # print(f"bdy_G_to_bdy_A: {bdy_G_to_bdy_A}")
        # print(f"bdy_A_to_bdy_G: {bdy_A_to_bdy_G}")

        wld_G_to_wld_A = Pose3d(
            self.plugin.enu_to_ned[0],
            self.plugin.enu_to_ned[1],
            self.plugin.enu_to_ned[2],
            self.plugin.enu_to_ned[3],
            self.plugin.enu_to_ned[4],
            self.plugin.enu_to_ned[5],
        )
        wld_A_to_wld_G = wld_G_to_wld_A.inverse()
        # print(f"wld_A_to_wld_G: {wld_A_to_wld_G}")
        # print(f"wld_G_to_wld_A: {wld_G_to_wld_A}")

        # Gazebo world to body transform
        wld_G_to_bdy_G = world_pose
        wld_A_to_bdy_A = wld_A_to_wld_G * wld_G_to_bdy_G * bdy_A_to_bdy_G.inverse()
        # print(f"wld_A_to_bdy_A: {wld_A_to_bdy_A}")

        # Velocity transform
        vel_wld_G = world_lin_vel
        vel_wld_A = wld_A_to_wld_G.rot() * vel_wld_G + wld_A_to_wld_G.pos()
        # print(f"vel_wld_A: {vel_wld_A}")

        # TODO use sim_time
        timestamp = time.monotonic() - self.start_time

        # Create JSON payload
        self.json_data = {
            "timestamp": timestamp,
            "imu": {
                "gyro": [
                    ang_vel.x(),
                    ang_vel.y(),
                    ang_vel.z(),
                ],
                "accel_body": [
                    lin_acc.x(),
                    lin_acc.y(),
                    lin_acc.z(),
                ],
            },
            "position": [
                wld_A_to_bdy_A.pos().x(),
                wld_A_to_bdy_A.pos().y(),
                wld_A_to_bdy_A.pos().z(),
            ],
            "quaternion": [
                wld_A_to_bdy_A.rot().w(),
                wld_A_to_bdy_A.rot().x(),
                wld_A_to_bdy_A.rot().y(),
                wld_A_to_bdy_A.rot().z(),
            ],
            "velocity": [
                vel_wld_A.x(),
                vel_wld_A.y(),
                vel_wld_A.z(),
            ],
        }
        # print(f"json_data: {self.json_data}")

    def _send_state(self, address):
        self.sock.sendto(
            (json.dumps(self.json_data, separators=(",", ":")) + "\n").encode("ascii"),
            address,
        )

    def _clock_cb(self, msg):
        with self.clock_lock:
            self.clock_msg = copy.deepcopy(msg)
            # print(self.clock_msg)

    def _pose_info_cb(self, msg):
        with self.pose_info_lock:
            self.pose_info_msg = copy.deepcopy(msg)
            # print(self.pose_info_msg)

    def _model_cb(self, msg):
        with self.model_lock:
            self.model_msg = copy.deepcopy(msg)
            # print(self.model_msg)

    def _imu_cb(self, msg):
        with self.imu_lock:
            self.imu_msg = copy.deepcopy(msg)
            # print(self.imu_msg)


def main():
    # Command line args
    parser = ArgumentParser(description="Launch ArduPilot Gazebo Bridge")
    parser.add_argument("--world", default="runway", type=str, help="world name")
    parser.add_argument(
        "--model", default="iris_with_ardupilot", type=str, help="model name"
    )
    parser.add_argument(
        "--address", default="127.0.0.1", type=str, help="SITL IPv4 address"
    )
    parser.add_argument("--port", default="9002", type=int, help="SITL port")
    parser.add_argument(
        "--timeout", default="5000", type=int, help="timeout for service calls"
    )

    args = parser.parse_args()

    # ======================================================================= #
    # Create bridge

    gz_bridge = ArduPilotGazeboBridge(args)
    gz_bridge.configure()
    gz_bridge.run()

    # ======================================================================= #


if __name__ == "__main__":
    main()
    # help(Pose3d)
    # help(Quaterniond)
