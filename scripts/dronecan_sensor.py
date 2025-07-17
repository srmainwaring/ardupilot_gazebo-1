#!/usr/bin/env python3
"""
Publish sensor data using DroneCAN
"""

import dronecan
import math
import os
import threading
import time

from argparse import ArgumentParser
from contextlib import closing

GZ_VERSION_GARDEN = "garden"
GZ_VERSION_HARMONIC = "harmonic"
GZ_VERSION_IONIC = "ionic"


def gz_version():
    """Return the environment variable GZ_VERSION if set, else default to 'harmonic'"""
    return os.environ.get("GZ_VERSION", GZ_VERSION_HARMONIC)


if gz_version() == GZ_VERSION_GARDEN:
    from gz.msgs9.model_pb2 import Model
elif gz_version() == GZ_VERSION_HARMONIC:
    from gz.msgs10.model_pb2 import Model
elif gz_version() == GZ_VERSION_IONIC:
    from gz.msgs11.model_pb2 import Model


# Importing gz.transport into the module global scope causes an odd
# multiprocessing conflict with dronecan. This is a workaround.
def gz_node():
    if gz_version() == GZ_VERSION_GARDEN:
        from gz.transport12 import Node
    elif gz_version() == GZ_VERSION_HARMONIC:
        from gz.transport13 import Node
    elif gz_version() == GZ_VERSION_IONIC:
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

        if do_print_msg:
            print(msg)

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

        self._lock = threading.Lock()
        self._node = None
        self._esc_pub = None
        self._task_thread = threading.Thread(target=self._run)
        self._task_thread.start()

    def _run(self):
        with self._lock:
            uri = self._uri
            node_id = self._node_id
            rate = self._rate

        node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
        node_info.name = "org.ardupilot.gazebo"
        node_info.software_version.major = 0
        node_info.hardware_version.unique_id = b"12345"
        # etc.

        # Initialise a DroneCAN node instance.
        with closing(
            dronecan.make_node(
                uri, node_id=node_id, bitrate=1000000, node_info=node_info
            )
        ) as self._node:
            # Setup to publish sensor measurement
            self._esc_pub = self._node.periodic(1.0 / rate, self._pub_esc_status)

            # Set mode and health status
            self._node.mode = dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL
            self._node.health = dronecan.uavcan.protocol.NodeStatus().HEALTH_OK
            # self._node.vendor_specific_status_code = 12345
            # etc.

            # Run the node
            while True:
                try:
                    self._node.spin()
                except dronecan.UAVCANException as ex:
                    print(f"Node error: {ex}")
                except dronecan.transport.TransferError as ex:
                    print(f"Node error: {ex}")

    def _pub_esc_status(self):
        with self._lock:
            debug = self._debug
            rpm = self._rpm

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
            print(dronecan.to_yaml(msg))

    def set_rpm(self, esc_index, rpm):
        with self._lock:
            self._rpm[esc_index] = rpm


# converters
def to_uavcan_equipment_air_data_StaticPressure():
    msg = dronecan.uavcan.equipment.air_data.StaticPressure()
    return msg


def to_uavcan_equipment_air_data_StaticTemperature():
    msg = dronecan.uavcan.equipment.air_data.StaticTemperature()
    return msg


def to_uavcan_equipment_power_BatteryInfo():
    msg = dronecan.uavcan.equipment.power.BatteryInfo()
    return msg


def to_ardupilot_equipment_power_BatteryInfoAux():
    msg = dronecan.ardupilot.equipment.power.BatteryInfoAux()
    return msg


def to_ardupilot_indication_Button():
    msg = dronecan.ardupilot.indication.Button()
    return msg


def to_uavcan_equipment_ahrs_MagneticFieldStrength():
    msg = dronecan.uavcan.equipment.ahrs.MagneticFieldStrength()
    msg.magnetic_field_ga = [0.0 for x in range(3)]
    msg.magnetic_field_covariance = [0.0 for x in range(9)]
    return msg


def to_uavcan_equipment_ahrs_MagneticFieldStrength2():
    msg = dronecan.uavcan.equipment.ahrs.MagneticFieldStrength2()
    msg.sensor_id = 0
    msg.magnetic_field_ga = [0.0 for x in range(3)]
    msg.magnetic_field_covariance = [0.0 for x in range(9)]
    return msg


def to_dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes():
    msg = dronecan.dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes()
    msg.sensor_id = 0
    msg.magnetic_field_ga = [0.0 for x in range(3)]
    return msg


def to_dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes():
    msg = dronecan.dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes()
    msg.sensor_id = 0
    msg.magnetic_field_ga = [0.0 for x in range(3)]
    return msg


def to_uavcan_equipment_ice_reciprocating_Status():
    msg = dronecan.uavcan.equipment.ice.reciprocating.Status()
    return msg


def to_uavcan_equipment_esc_Status():
    msg = dronecan.uavcan.equipment.esc.Status()
    return msg


def to_uavcan_equipment_esc_StatusExtended():
    msg = dronecan.uavcan.equipment.esc.StatusExtended()
    return msg


def to_uavcan_equipment_gnss_Fix2():
    msg = dronecan.uavcan.equipment.gnss.Fix2()
    return msg


def to_uavcan_equipment_gnss_Auxiliary():
    msg = dronecan.uavcan.equipment.gnss.Auxiliary()
    return msg


def to_ardupilot_gnss_Heading():
    msg = dronecan.ardupilot.gnss.Heading()
    return msg


def to_ardupilot_gnss_Status():
    msg = dronecan.ardupilot.gnss.Status()
    return msg


def to_ardupilot_gnss_MovingBaselineData():
    msg = dronecan.ardupilot.gnss.MovingBaselineData()
    return msg


def to_ardupilot_gnss_RelPosHeading():
    msg = dronecan.ardupilot.gnss.RelPosHeading()
    return msg


def to_com_hex_equipment_flow_Measurement():
    msg = dronecan.com.hex.equipment.flow.Measurement()
    return msg


def to_ardupilot_equipment_proximity_sensor_Proximity():
    msg = dronecan.ardupilot.equipment.proximity_sensor.Proximity()
    return msg


def to_uavcan_equipment_range_sensor_Measurement():
    msg = dronecan.uavcan.equipment.range_sensor.Measurement()
    return msg


def to_dronecan_sensors_rc_RCInput():
    msg = dronecan.dronecan.sensors.rc.RCInput()
    return msg


def to_dronecan_sensors_rpm_RPM():
    msg = dronecan.dronecan.sensors.rpm.RPM()
    return msg


def to_uavcan_equipment_actuator_Status():
    msg = dronecan.uavcan.equipment.actuator.Status()
    return msg


def to_uavcan_equipment_device_Temperature():
    msg = dronecan.uavcan.equipment.device.Temperature()
    return msg


def test_converters():
    # Barometer
    print(to_uavcan_equipment_air_data_StaticPressure())
    print(to_uavcan_equipment_air_data_StaticTemperature())

    # Battery Monitor
    print(to_uavcan_equipment_power_BatteryInfo())
    print(to_ardupilot_equipment_power_BatteryInfoAux())

    # Button
    print(to_ardupilot_indication_Button())

    # Compass
    print(to_uavcan_equipment_ahrs_MagneticFieldStrength())
    print(to_uavcan_equipment_ahrs_MagneticFieldStrength2())
    print(to_dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes())

    # EFI
    print(to_uavcan_equipment_ice_reciprocating_Status())

    # ESC
    print(to_uavcan_equipment_esc_Status())
    print(to_uavcan_equipment_esc_StatusExtended())

    # GPS
    print(to_uavcan_equipment_gnss_Fix2())
    print(to_uavcan_equipment_gnss_Auxiliary())
    print(to_ardupilot_gnss_Heading())
    print(to_ardupilot_gnss_Status())
    print(to_ardupilot_gnss_MovingBaselineData())
    print(to_ardupilot_gnss_RelPosHeading())

    # OpticalFlow
    print(to_com_hex_equipment_flow_Measurement())

    # Proximity
    print(to_ardupilot_equipment_proximity_sensor_Proximity())

    # Rangefinder
    print(to_uavcan_equipment_range_sensor_Measurement())

    # RCProtocol
    print(to_dronecan_sensors_rc_RCInput())

    # RPM
    print(to_dronecan_sensors_rpm_RPM())

    # Servo
    print(to_uavcan_equipment_actuator_Status())

    # Temperature
    print(to_uavcan_equipment_device_Temperature())


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


def show_msg_types():
    """Print all DroneCAN message types handled by ArduPilot"""

    print("Airspeed")
    print(dronecan.uavcan.equipment.air_data.RawAirData())
    print(dronecan.dronecan.sensors.hygrometer.Hygrometer())
    print("--------------------")

    print("Barometer")
    print(dronecan.uavcan.equipment.air_data.StaticPressure())
    print(dronecan.uavcan.equipment.air_data.StaticTemperature())
    print("--------------------")

    print("Battery Monitor")
    print(dronecan.uavcan.equipment.power.BatteryInfo())
    print(dronecan.ardupilot.equipment.power.BatteryInfoAux())
    # print(dronecan.mppt.Stream())
    print("--------------------")

    print("Button")
    print(dronecan.ardupilot.indication.Button())
    print("--------------------")

    print("Compass")
    print(dronecan.uavcan.equipment.ahrs.MagneticFieldStrength())
    print(dronecan.uavcan.equipment.ahrs.MagneticFieldStrength2())
    print(dronecan.dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes())
    print("--------------------")

    print("EFI")
    print(dronecan.uavcan.equipment.ice.reciprocating.Status())
    print("--------------------")

    print("ESC")
    print(dronecan.uavcan.equipment.esc.Status())
    print(dronecan.uavcan.equipment.esc.StatusExtended())
    print("--------------------")

    print("GPS")
    print(dronecan.uavcan.equipment.gnss.Fix2())
    print(dronecan.uavcan.equipment.gnss.Auxiliary())
    print(dronecan.ardupilot.gnss.Heading())
    print(dronecan.ardupilot.gnss.Status())
    print(dronecan.ardupilot.gnss.MovingBaselineData())
    print(dronecan.ardupilot.gnss.RelPosHeading())
    print("--------------------")

    print("LED")
    print("--------------------")

    print("OpticalFlow")
    print(dronecan.com.hex.equipment.flow.Measurement())
    print("--------------------")

    print("Proximity")
    print(dronecan.ardupilot.equipment.proximity_sensor.Proximity())
    print("--------------------")

    print("Rangefinder")
    print(dronecan.uavcan.equipment.range_sensor.Measurement())
    print("--------------------")

    print("RCProtocol")
    print(dronecan.dronecan.sensors.rc.RCInput())
    print("--------------------")

    print("RPM")
    print(dronecan.dronecan.sensors.rpm.RPM())
    print("--------------------")

    print("Servo")
    print(dronecan.uavcan.equipment.actuator.Status())
    print("--------------------")

    print("Temperature")
    print(dronecan.uavcan.equipment.device.Temperature())
    print("--------------------")

    print("Vendor")
    print(dronecan.com.hex.equipment.flow.Measurement())
    print(dronecan.com.himark.servo.ServoInfo())
    print(dronecan.com.volz.servo.ActuatorStatus())
    print(dronecan.com.hobbywing.esc.GetEscID())
    print(dronecan.com.hobbywing.esc.StatusMsg1())
    print(dronecan.com.hobbywing.esc.StatusMsg2())
    print("--------------------")


if __name__ == "__main__":
    # main()
    show_msg_types()
    test_converters()
