"""
ArduPilot Gazebo Bridge - carry out similar functions to the ArduPilotPlugin

Based on example: ardupilot/libraries/SITL/examples/JSON/pybullet/robot.py
"""

import copy
import json
import math
import os
import socket
import struct
import time

from dataclasses import dataclass
from argparse import ArgumentParser
from pathlib import Path
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
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.entity_factory_pb2 import EntityFactory
    from gz.msgs.entity_factory_v_pb2 import EntityFactory_V
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
    multipler: float = 1.0
    offset: float = 0.0
    servo_min: float = 1000.0
    servo_max: float = 2000.0
    output_ready: bool = False


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
        self.print_frame_count = 1000

        # Payload
        self.json_str = None
        self.start_time = time.monotonic()

        # Transport
        self.node = gz_node()
        self.controls = []
        self.pub_commands = []
        self.sub_clock = None
        self.sub_imu = None
        self.sub_pose_info = None

    def configure(self):
        # Configure model
        # TODO
        # Hardcoded example to start
        control = Control()
        control.


        # Setup connection
        self._init_sockets()

    def run(self):
        global MAGIC
        global RATE_HZ
        global TIME_STEP

        while True:
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

            # post-update
            # TODO
            # self._create_state_json()
            # self._send_state()
            phys_time = time.monotonic() - self.start_time
            gyro = [0.0, 0.0, 0.0]
            accel = [0.0, 0.0, 0.0]
            pos = [0.0, 0.0, 0.0]
            vel = [0.0, 0.0, 0.0]
            quat = [0.0, 0.0, 0.0, 1.0]

            json_data = {
                "timestamp": phys_time,
                "imu": {"gyro": gyro, "accel_body": accel},
                "position": pos,
                "quaternion": quat,
                "velocity": vel,
            }

            self.sock.sendto(
                (json.dumps(json_data, separators=(",", ":")) + "\n").encode("ascii"),
                address,
            )

            if self.frame_count % self.print_frame_count == 0:
                now = time.time()
                total_time = now - self.frame_time
                print(
                    f"{self.print_frame_count/total_time:.2f} "
                    f"fps T={phys_time:.3f} "
                    f"dt={total_time:.3f}"
                )
                print(f"pwm: {self.pwm}")

                self.frame_time = now

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

    def _create_state_json(self):
        pass
        # timestamp = time.monotonic() - self.start_time

        # self.json_str = json.dumps(
        #     {
        #         "timestamp": timestamp,
        #         "imu": {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, 0.0]},
        #         "position": [0.0, 0.0, 0.0],
        #         "quaternion": [0.0, 0.0, 0.0, 1.0],
        #         "velocity": [0.0, 0.0, 0.0],
        #     }
        # )
        # print(self.json_str)
        # print(len(self.json_str))

    def _send_state(self):
        pass
        # self.sock.sendto(self.json_str, self.address, self.port)


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

    json_str = json.dumps(
        {
            "timestamp": 1,
            "imu": {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, 0.0]},
            "position": [0.0, 0.0, 0.0],
            "quaternion": [0.0, 0.0, 0.0, 1.0],
            "velocity": [0.0, 0.0, 0.0],
        }
    )
    # print(json_str)
    # print(len(json_str))

    # ======================================================================= #
    # Create bridge

    gz_bridge = ArduPilotGazeboBridge(args)
    gz_bridge.configure()
    gz_bridge.run()

    # ======================================================================= #


if __name__ == "__main__":
    main()
