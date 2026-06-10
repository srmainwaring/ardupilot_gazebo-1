#!/usr/bin/env python3
"""
OP3 Joint State Bridge -- GSoC 2026 ArduHumanoid
Publishes ROBOTIS OP3 joint state (position, velocity) from Gazebo
to ArduPilot via DroneCAN actuator.Status messages.

Based on Rhys Mainwaring's dronecan_sensor.py prototype:
github.com/srmainwaring/ardupilot_gazebo-1/tree/wips/wip-dronecan-extra

Usage:
    python3 joint_state_bridge.py mcast:0

AP parameters to set:
    CAN_P1_DRIVER 1  then reboot AP

View in mavexplorer:
    graph CSRV[0].pos CSRV[1].pos CSRV[2].pos

actuator_id indexed from 1. Maps to SERVOx_FUNCTION in AP.
"""

import dronecan
import os
import threading
import time
from argparse import ArgumentParser
from contextlib import closing


def gz_version():
    return os.environ.get("GZ_VERSION", "harmonic")


if gz_version() == "garden":
    from gz.msgs9.model_pb2 import Model
elif gz_version() == "harmonic":
    from gz.msgs10.model_pb2 import Model
elif gz_version() == "ionic":
    from gz.msgs11.model_pb2 import Model
elif gz_version() == "jetty":
    from gz.msgs.model_pb2 import Model


def gz_node():
    if gz_version() == "garden":
        from gz.transport12 import Node
    elif gz_version() == "harmonic":
        from gz.transport13 import Node
    elif gz_version() == "ionic":
        from gz.transport14 import Node
    elif gz_version() == "jetty":
        from gz.transport import Node
    return Node()


OP3_LEG_JOINTS = [
    ("l_hip_yaw",   1),
    ("l_hip_roll",  2),
    ("l_hip_pitch", 3),
    ("l_knee",      4),
    ("l_ank_pitch", 5),
    ("l_ank_roll",  6),
    ("r_hip_yaw",   7),
    ("r_hip_roll",  8),
    ("r_hip_pitch", 9),
    ("r_knee",      10),
    ("r_ank_pitch", 11),
    ("r_ank_roll",  12),
]


class JointStatesConverter:
    def __init__(self, topic, debug):
        self._topic = topic
        self._debug = debug
        self._lock = threading.Lock()
        self._node = gz_node()
        self._model_msg = None
        self._model_sub = self._node.subscribe(Model, self._topic, self._model_cb)
        print(f"Subscribed to: {self._topic}")

    def _model_cb(self, msg):
        with self._lock:
            self._model_msg = msg
        if self._debug:
            print(msg)

    def joint_by_name(self, name):
        with self._lock:
            model = self._model_msg
        if model is None:
            return None
        matches = [x for x in model.joint if x.name == name]
        return matches[0] if matches else None

    def joint_position(self, name):
        j = self.joint_by_name(name)
        return float("nan") if j is None else j.axis1.position

    def joint_velocity(self, name):
        j = self.joint_by_name(name)
        return float("nan") if j is None else j.axis1.velocity

    def all_joint_names(self):
        with self._lock:
            model = self._model_msg
        return [] if model is None else [j.name for j in model.joint]


class DroneCANNode:
    def __init__(self, uri, node_id, rate, debug):
        self._uri = uri
        self._node_id = node_id
        self._rate = rate
        self._debug = debug
        self._actuator_status = {}
        self._lock = threading.Lock()
        self._node = None
        self._task_thread = threading.Thread(target=self._run)
        self._task_thread.start()

    def _run(self):
        node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
        node_info.name = "org.ardupilot.gazebo.op3"
        node_info.software_version.major = 0
        node_info.hardware_version.unique_id = b"op3_bridge_001"
        with closing(
            dronecan.make_node(
                self._uri, node_id=self._node_id,
                bitrate=1000000, node_info=node_info)
        ) as self._node:
            self._node.periodic(1.0 / self._rate, self._pub_actuator_status)
            self._node.mode = dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL
            self._node.health = dronecan.uavcan.protocol.NodeStatus().HEALTH_OK
            while True:
                try:
                    self._node.spin()
                except dronecan.UAVCANException as ex:
                    print(f"Node error: {ex}")
                except dronecan.transport.TransferError as ex:
                    print(f"Node error: {ex}")

    def _pub_actuator_status(self):
        with self._lock:
            for key in self._actuator_status:
                msg = self._actuator_status[key]
                self._node.broadcast(msg)
                if self._debug:
                    print(dronecan.to_yaml(msg))

    def set_actuator_status(self, joint_name, actuator_id, position, velocity):
        msg = dronecan.uavcan.equipment.actuator.Status()
        msg.actuator_id = actuator_id
        msg.position = position
        msg.speed = velocity
        msg.force = float("nan")
        msg.power_rating_pct = msg.POWER_RATING_PCT_UNKNOWN
        with self._lock:
            self._actuator_status[joint_name] = msg


def main():
    parser = ArgumentParser(description="OP3 joint state bridge")
    parser.add_argument("uri", type=str, help="DroneCAN URI e.g. mcast:0")
    parser.add_argument("--node-id", default=100, type=int)
    parser.add_argument("--rate", type=float, default=50)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--topic", default="/op3_gz_joint_states", type=str)
    args = parser.parse_args()

    print(f"Starting OP3 joint state bridge")
    print(f"  Topic:   {args.topic}")
    print(f"  CAN URI: {args.uri}")
    print(f"  Rate:    {args.rate} Hz")

    joint_states = JointStatesConverter(args.topic, args.debug)
    dronecan_node = DroneCANNode(args.uri, args.node_id, args.rate, args.debug)

    print("Waiting for joint states from Gazebo...")
    start = time.time()
    while time.time() - start < 10.0:
        if joint_states.all_joint_names():
            print(f"Receiving joint states. Publishing {len(OP3_LEG_JOINTS)} leg joints.")
            break
        time.sleep(0.1)
    else:
        print("WARNING: No joint states received. Is Gazebo running?")

    while True:
        for joint_name, actuator_id in OP3_LEG_JOINTS:
            pos = joint_states.joint_position(joint_name)
            vel = joint_states.joint_velocity(joint_name)
            dronecan_node.set_actuator_status(joint_name, actuator_id, pos, vel)
        time.sleep(1.0 / args.rate)


if __name__ == "__main__":
    main()
