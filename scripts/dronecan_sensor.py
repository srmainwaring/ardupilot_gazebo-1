#!/usr/bin/env python3
"""
Publish sensor data using DroneCAN
"""

import dronecan
import math
import threading
import time

from argparse import ArgumentParser

GZ_VERSION_GARDEN = "garden"
GZ_VERSION_HARMONIC = "harmonic"
GZ_VERSION_IONIC = "ionic"

GZ_VERSION = GZ_VERSION_HARMONIC

if GZ_VERSION == GZ_VERSION_GARDEN:
    from gz.msgs9.model_pb2 import Model
elif GZ_VERSION == GZ_VERSION_HARMONIC:
    # from gz.msgs10.air_speed_pb2 import AirSpeed
    # from gz.msgs10.altimeter_pb2 import Altimeter
    # from gz.msgs10.axis_pb2 import Axis
    # from gz.msgs10.axis_aligned_box_pb2 import AxisAlignedBox
    # from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    # from gz.msgs10.imu_pb2 import IMU
    # from gz.msgs10.joint_pb2 import Joint
    # from gz.msgs10.link_pb2 import Link
    # from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.model_pb2 import Model
    # from gz.msgs10.navsat_pb2 import NavSat
    # from gz.msgs10.pose_pb2 import Pose
    # from gz.msgs10.sensor_pb2 import Sensor
    # from gz.msgs10.vector3d_pb2 import Vector3d
    # from gz.msgs10.visual_pb2 import Visual
    # from gz.msgs10.header_pb2 import Header
elif GZ_VERSION == GZ_VERSION_IONIC:
    from gz.msgs11.model_pb2 import Model


# def sensor_topics():
#     world = "iris_runway"
#     model = "iris_with_gimbal"
#     sub_model = "iris_with_standoffs"
#     link = "base_link"

#     topic = (
#         f"/world/{world}/model/{model}"
#         f"/model/{sub_model}/link/{link}"
#         f"/sensor/air_pressure_sensor/air_pressure"
#     )

#     topic = (
#         f"/world/{world}/model/{model}"
#         f"/model/{sub_model}/link/base_link"
#         f"/sensor/air_speed_sensor/air_speed"
#     )

#     topic = (
#         f"/world/{world}/model/{model}"
#         f"/model/{sub_model}/link/{link}"
#         f"/sensor/altimeter_sensor/altimeter"
#     )

#     topic = (
#         f"/world/{world}/model/{model}"
#         f"/model/{sub_model}/link/{link}"
#         f"/sensor/magnetometer_sensor/magnetometer"
#     )

#     topic = (
#         f"/world/{world}/model/{model}"
#         f"/model/{sub_model}/link/{link}"
#         f"/sensor/navsat_sensor/navsat"
#     )

#     link = "imu_link"

#     topic = (
#         f"/world/{world}/model/{model}"
#         f"/model/{sub_model}/link/{link}"
#         f"/sensor/imu_sensor/imu"
#     )


# Importing gz.transport13 into the module global scope causes an odd conflict
# with dronecan. This is a workaround.
def gz_node():
    if GZ_VERSION == GZ_VERSION_GARDEN:
        from gz.transport12 import Node
    elif GZ_VERSION == GZ_VERSION_HARMONIC:
        from gz.transport13 import Node
    elif GZ_VERSION == GZ_VERSION_IONIC:
        from gz.transport14 import Node

    return Node()


class JointStates:
    def __init__(self, world, model, debug):

        self._world = world
        self._model = model
        self._debug = debug

        self._lock = threading.Lock()
        self._node = gz_node()

        # Model
        self._model_do_print_msg = self._debug
        self._model_msg = None
        self._model_topic = f"/world/{self._world}/model/{self._model}/joint_state"
        self._model_sub = self._node.subscribe(Model, self._model_topic, self._model_cb)

    def _model_cb(self, msg):
        with self._lock:
            self._model_msg = msg
            do_print_msg = self._model_do_print_msg

        # if do_print_msg:
        #     print(msg)

    def joint_by_name(self, joint_name):
        with self._lock:
            model = self._model_msg

        if model is None:
            return None

        # Find the first joint that matches
        matches = [x for x in model.joint if x.name == joint_name]
        if matches:
            return matches[0]
        else:
            return None

    def joint_velocity(self, joint_name):
        joint = self.joint_by_name(joint_name)
        if joint is None:
            return float("nan")

        axis1 = joint.axis1
        vel_rads = axis1.velocity
        return vel_rads


class DroneCANNode:
    def __init__(self, uri, node_id, rate, debug):
        self._uri = uri
        self._node_id = node_id
        self._rate = rate
        self._debug = debug

        self._rpm = [0, 0, 0, 0]

        self._node = None
        self._lock = threading.Lock()
        self._task_thread = threading.Thread(target=self._run)
        self._task_thread.start()

    def _run(self):
        with self._lock:
            uri = self._uri
            node_id = self._node_id
            rate = self._rate

        # Initialise a DroneCAN node instance.
        self._node = dronecan.make_node(uri, node_id=node_id, bitrate=1000000)

        # Setup to publish sensor measurement
        self._node.periodic(1.0 / rate, self._pub_esc_status)

        # Running the node
        while True:
            try:
                self._node.spin()
            except dronecan.transport.TransferError as ex:
                print(ex)
                pass

    def _pub_esc_status(self):
        with self._lock:
            debug = self._debug
            rpm = self._rpm

        # fake esc data
        # s = math.sin(time.time() * math.pi * 2)
        # rpm[0] = int(10000 + 1000 * s)
        # rpm[1] = int(10500 + 1000 * s)
        # rpm[2] = int(11000 + 1000 * s)
        # rpm[3] = int(11500 + 1000 * s)

        msg = dronecan.uavcan.equipment.esc.Status()

        # esc 0, 1, 2, 3
        msg.error_count = 0
        msg.voltage = 4.0 * 4.2
        msg.current = 5.0
        msg.temperature = 45.0 + 273.15
        msg.power_rating_pct = 50

        msg.esc_index = 0
        msg.rpm = abs(rpm[msg.esc_index])
        self._node.broadcast(msg)

        msg.esc_index = 1
        msg.rpm = abs(rpm[msg.esc_index])
        self._node.broadcast(msg)

        msg.esc_index = 2
        msg.rpm = abs(rpm[msg.esc_index])
        self._node.broadcast(msg)

        msg.esc_index = 3
        msg.rpm = abs(rpm[msg.esc_index])
        self._node.broadcast(msg)

        if debug:
            # print(f"idx {msg.esc_index}, rpm: {msg.rpm}")
            print(dronecan.to_yaml(msg))

    def set_rpm(self, esc_index, rpm):
        with self._lock:
            self._rpm[esc_index] = rpm


def main():
    # Command line args
    parser = ArgumentParser(description="Publish DroneCAN ESC")
    parser.add_argument("uri", default=None, type=str, help="CAN URI")
    parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
    parser.add_argument("--rate", type=float, default=50, help="broadcast rate Hz")
    parser.add_argument("--debug", action="store_true", help="enable debug")
    parser.add_argument("--world", default="iris_runway", type=str, help="world name")
    parser.add_argument(
        "--model", default="iris_with_gimbal", type=str, help="model name"
    )
    args = parser.parse_args()

    # Subscribe to joint states
    joint_states = JointStates(args.world, args.model, args.debug)

    # DroneCAN node
    dronecan_node = DroneCANNode(args.uri, args.node_id, args.rate, args.debug)

    def rads_to_rpm(vel_rads):
        if math.isnan(vel_rads):
            return 0
        else:
            return vel_rads * 30.0 / math.pi

    # Run the node
    while True:
        # Read from joint states
        rpm0 = int(rads_to_rpm(joint_states.joint_velocity("rotor_0_joint")))
        rpm1 = int(rads_to_rpm(joint_states.joint_velocity("rotor_1_joint")))
        rpm2 = int(rads_to_rpm(joint_states.joint_velocity("rotor_2_joint")))
        rpm3 = int(rads_to_rpm(joint_states.joint_velocity("rotor_3_joint")))

        # Write to dronecan node
        dronecan_node.set_rpm(0, rpm0)
        dronecan_node.set_rpm(1, rpm1)
        dronecan_node.set_rpm(2, rpm2)
        dronecan_node.set_rpm(3, rpm3)

        time.sleep(0.01)


if __name__ == "__main__":
    main()
