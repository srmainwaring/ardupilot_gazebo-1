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
from dataclasses import dataclass

GZ_VERSION_GARDEN = "garden"
GZ_VERSION_HARMONIC = "harmonic"
GZ_VERSION_IONIC = "ionic"


def gz_version():
    """Return the environment variable GZ_VERSION if set, else default to 'harmonic'"""
    return os.environ.get("GZ_VERSION", GZ_VERSION_HARMONIC)


if gz_version() == GZ_VERSION_GARDEN:
    from gz.msgs9.air_speed_pb2 import AirSpeed
    from gz.msgs9.altimeter_pb2 import Altimeter
    from gz.msgs9.battery_state_pb2 import BatteryState
    from gz.msgs9.fluid_pressure_pb2 import FluidPressure
    from gz.msgs9.magnetometer_pb2 import Magnetometer
    from gz.msgs9.model_pb2 import Model
    from gz.msgs9.navsat_pb2 import NavSat
elif gz_version() == GZ_VERSION_HARMONIC:
    from gz.msgs10.air_speed_pb2 import AirSpeed
    from gz.msgs10.altimeter_pb2 import Altimeter
    from gz.msgs10.battery_state_pb2 import BatteryState
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.navsat_pb2 import NavSat
elif gz_version() == GZ_VERSION_IONIC:
    from gz.msgs11.air_speed_pb2 import AirSpeed
    from gz.msgs11.altimeter_pb2 import Altimeter
    from gz.msgs11.battery_state_pb2 import BatteryState
    from gz.msgs11.fluid_pressure_pb2 import FluidPressure
    from gz.msgs11.magnetometer_pb2 import Magnetometer
    from gz.msgs11.model_pb2 import Model
    from gz.msgs11.navsat_pb2 import NavSat


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


def celsius_to_kelvin(value):
    ZERO_CELSIUS_KELVIN = float(273.15)
    return ZERO_CELSIUS_KELVIN + value


class JointStatesConverter:
    def __init__(self, world, model, debug):

        self._world = world
        self._model = model
        self._debug = debug

        self._lock = threading.Lock()
        self._node = gz_node()

        # Model
        self._model_do_print_msg = self._debug
        self._model_msg = None
        self._model_topic = (
            f"/world/{self._world}" f"/model/{self._model}" f"/joint_state"
        )
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


class MagnetometerConverter:
    def __init__(self, world, model, link, sensor_name, debug):

        self._world = world
        self._model = model
        self._link = link
        self._sensor_type = "magnetometer"
        self._sensor_name = sensor_name
        self._debug = debug

        self._lock = threading.Lock()
        self._node = gz_node()

        # Magnetometer
        self._do_print_msg = self._debug
        self._msg = None
        self._topic = (
            f"/world/{self._world}"
            f"/model/{self._model}"
            f"/link/{self._link}"
            f"/sensor/{self._sensor_name}/{self._sensor_type}"
        )
        self._sub = self._node.subscribe(Magnetometer, self._topic, self._cb)

    def _cb(self, msg):
        with self._lock:
            self._msg = msg
            do_print_msg = self._do_print_msg

        if do_print_msg:
            print(msg)

    def field_tesla(self):
        with self._lock:
            msg = self._msg

        if msg is None:
            return float("nan")

        return msg.field_tesla

    def field_gauss(self):
        with self._lock:
            msg = self._msg

        if msg is None:
            return [float("nan") for x in range(3)]

        return [
            msg.field_tesla.x * 1.0e4,
            msg.field_tesla.y * 1.0e4,
            msg.field_tesla.z * 1.0e4,
        ]


class NavSatConverter:
    def __init__(self, world, model, link, sensor_name, debug):

        self._world = world
        self._model = model
        self._link = link
        self._sensor_type = "navsat"
        self._sensor_name = sensor_name
        self._debug = debug

        self._lock = threading.Lock()
        self._node = gz_node()

        # Magnetometer
        self._do_print_msg = self._debug
        self._msg = None
        self._topic = (
            f"/world/{self._world}"
            f"/model/{self._model}"
            f"/link/{self._link}"
            f"/sensor/{self._sensor_name}/{self._sensor_type}"
        )
        self._sub = self._node.subscribe(NavSat, self._topic, self._cb)

    def _cb(self, msg):
        with self._lock:
            self._msg = msg
            do_print_msg = self._do_print_msg

        if do_print_msg:
            print(msg)

    def last_msg(self):
        with self._lock:
            msg = self._msg

        return msg

    def uavcan_equipment_gnss_fix2(self):
        with self._lock:
            gz_msg = self._msg

        if gz_msg is None:
            return None

        # Gazebo sim time
        sec = gz_msg.header.stamp.sec
        nsec = gz_msg.header.stamp.nsec
        usec = int(1000000 * sec + nsec / 1000)

        lat_deg = gz_msg.latitude_deg
        lon_deg = gz_msg.longitude_deg
        alt_wgs84_m = gz_msg.altitude
        vel_e = gz_msg.velocity_east
        vel_n = gz_msg.velocity_north
        vel_u = gz_msg.velocity_up

        # TODO: look up the geoid height and adjust
        geoid_m = 0
        alt_msl_m = alt_wgs84_m + geoid_m

        # TODO: need a conversion from height_ellipsoid_mm to height_msl_mm

        # TODO: consolidate conversions - perhaps use a @dataclass to collect
        # Gazebo messages required for the conversion, where more than one
        # message is required, or additional information needs to be supplied
        # from configuration.
        # Additional information
        @dataclass
        class NatSatAuxData:
            sats_used: int = 0
            pdop: float = float("nan")

        msg = dronecan.uavcan.equipment.gnss.Fix2()

        # assume that all timestamps are UTC
        msg.timestamp.usec = usec
        msg.gnss_timestamp.usec = usec
        msg.gnss_time_standard = msg.GNSS_TIME_STANDARD_UTC
        msg.num_leap_seconds = msg.NUM_LEAP_SECONDS_UNKNOWN

        msg.longitude_deg_1e8 = int(lon_deg * 1e8)
        msg.latitude_deg_1e8 = int(lat_deg * 1e8)
        msg.height_ellipsoid_mm = int(alt_wgs84_m * 1e3)
        msg.height_msl_mm = int(alt_msl_m * 1e3)
        msg.ned_velocity = [vel_n, vel_e, vel_u * -1.0]
        msg.sats_used = 0
        msg.status = msg.STATUS_3D_FIX
        msg.mode = msg.MODE_DGPS
        msg.sub_mode = msg.SUB_MODE_DGPS_OTHER
        msg.covariance == []
        msg.pdop = float("nan")
        # TODO: provide a conversion to ECEF coordinates
        # dronecan.uavcan.equipment.gnss.ECEFPositionVelocity()
        # XYZ velocity in m/s
        # msg.ecef_position_velocity.velocity_xyz = [0.0, 0.0, 0.0]
        # XYZ-axis coordinates in mm
        # msg.ecef_position_velocity.position_xyz_mm = [0, 0, 0]
        # Position and velocity covariance in the ECEF frame. Units are m^2 for position,
        # (m/s)^2 for velocity, and m^2/s for position/velocity.
        # msg.ecef_position_velocity.covariance = []
        return msg


class DroneCANNode:
    def __init__(self, uri, node_id, rate, debug):
        self._uri = uri
        self._node_id = node_id
        self._rate = rate
        self._debug = debug

        # TODO add methods to size
        self._rpm = [0, 0, 0, 0]
        self._magnetic_field_ga = [0]
        self._gnss_fix2 = [None]

        self._lock = threading.Lock()
        self._node = None

        self._esc_pub = None
        self._mag_pub = None
        self._gnss_fix2_pub = None

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
            self._mag_pub = self._node.periodic(
                1.0 / rate, self._pub_magnetic_field_strength
            )
            self._gnss_fix2_pub = self._node.periodic(
                1.0 / rate, self._pub_gnss_fix2
            )

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
        msg.temperature = celsius_to_kelvin(45.0)
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

    def _pub_magnetic_field_strength(self):
        with self._lock:
            debug = self._debug
            magnetic_field_ga = self._magnetic_field_ga

        msg = dronecan.uavcan.equipment.ahrs.MagneticFieldStrength2()

        msg.sensor_id = 0
        # print(f"magnetic_field_ga: {magnetic_field_ga[0]}")
        msg.magnetic_field_ga = magnetic_field_ga[0]
        msg.magnetic_field_covariance = []
        self._node.broadcast(msg)

        if debug:
            print(dronecan.to_yaml(msg))

    def _pub_gnss_fix2(self):
        with self._lock:
            debug = self._debug
            gnss_fix2 = self._gnss_fix2

        for msg in gnss_fix2:
            if msg is not None:
                self._node.broadcast(msg)

            if debug:
                print(dronecan.to_yaml(msg))

    def set_rpm(self, index, value):
        with self._lock:
            self._rpm[index] = value

    def set_magnetic_field_ga(self, index, value):
        with self._lock:
            self._magnetic_field_ga[index] = value

    def set_gnss_fix2(self, index, value):
        with self._lock:
            self._gnss_fix2[index] = value

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
    joint_states = JointStatesConverter(args.world, args.model, args.debug)

    # Subscribe to magnetic field
    magnetometer = MagnetometerConverter(
        args.world,
        args.model,
        link="base_link",
        sensor_name="magnetometer_sensor",
        debug=args.debug,
    )

    # Subscribe to navsat
    navsat = NavSatConverter(
        args.world,
        args.model,
        link="base_link",
        sensor_name="navsat_sensor",
        debug=args.debug,
    )

    # DroneCAN node
    dronecan_node = DroneCANNode(args.uri, args.node_id, args.rate, args.debug)

    def rads_to_rpm(vel_rads):
        if math.isnan(vel_rads):
            return 0
        else:
            return vel_rads * 30.0 / math.pi

    # Run the node
    while True:
        # ESC
        rpm0 = int(rads_to_rpm(joint_states.joint_velocity("rotor_0_joint")))
        rpm1 = int(rads_to_rpm(joint_states.joint_velocity("rotor_1_joint")))
        rpm2 = int(rads_to_rpm(joint_states.joint_velocity("rotor_2_joint")))
        rpm3 = int(rads_to_rpm(joint_states.joint_velocity("rotor_3_joint")))
        dronecan_node.set_rpm(0, rpm0)
        dronecan_node.set_rpm(1, rpm1)
        dronecan_node.set_rpm(2, rpm2)
        dronecan_node.set_rpm(3, rpm3)

        # Magnetic field
        mag0 = magnetometer.field_gauss()
        dronecan_node.set_magnetic_field_ga(0, mag0)

        # NavSat
        gnss_fix2 = navsat.uavcan_equipment_gnss_fix2()
        dronecan_node.set_gnss_fix2(0, gnss_fix2)

        time.sleep(0.01)


# Converter stubs
def to_uavcan_equipment_air_data_StaticPressure():
    msg = dronecan.uavcan.equipment.air_data.StaticPressure()
    msg.static_pressure = 0.0
    msg.static_pressure_variance = 0.0
    return msg


def to_uavcan_equipment_air_data_StaticTemperature():
    msg = dronecan.uavcan.equipment.air_data.StaticTemperature()
    msg.static_temperature = 0.0
    msg.static_temperature_variance = 0.0
    return msg


def to_uavcan_equipment_power_BatteryInfo():
    msg = dronecan.uavcan.equipment.power.BatteryInfo()
    # Kelvin
    msg.temperature = celsius_to_kelvin(20.0)
    # Volt
    msg.voltage = 6 * 4.0
    # Ampere
    msg.current = 0.0
    # Watt
    msg.average_power_10sec = 0.0
    # Watt hours
    msg.remaining_capacity_wh = 0.0
    # Watt hours
    msg.full_charge_capacity_wh = 0.0
    # hours
    msg.hours_to_full_charge = 0.0
    msg.status_flags = msg.STATUS_FLAG_IN_USE
    msg.state_of_health_pct = msg.STATE_OF_HEALTH_UNKNOWN
    msg.state_of_charge_pct = 100
    msg.state_of_charge_pct_stdev = 1
    msg.battery_id = 0
    msg.model_instance_id = 0
    msg.model_name = "MODEL_NAME_UNKNOWN"
    return msg


def to_ardupilot_equipment_power_BatteryInfoAux():
    msg = dronecan.ardupilot.equipment.power.BatteryInfoAux()
    msg.timestamp.usec = msg.timestamp.UNKNOWN
    msg.voltage_cell = [4.0 for x in range(6)]
    msg.cycle_count = 0
    msg.over_discharge_count = 0
    # Ampere
    msg.max_current = 0.0
    # Volt
    msg.nominal_voltage = 0.0
    msg.is_powering_off = False
    msg.battery_id = 0
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
    msg.state = 0
    msg.flags = 0
    msg.engine_load_percent = 0
    msg.engine_speed_rpm = 0
    msg.spark_dwell_time_ms = 0.0
    msg.atmospheric_pressure_kpa = 0.0
    msg.intake_manifold_pressure_kpa = 0.0
    msg.intake_manifold_temperature = 0.0
    msg.coolant_temperature = 0.0
    msg.oil_pressure = 0.0
    msg.oil_temperature = 0.0
    msg.fuel_pressure = 0.0
    msg.fuel_consumption_rate_cm3pm = 0.0
    msg.estimated_consumed_fuel_volume_cm3 = 0.0
    msg.throttle_position_percent = 0
    msg.ecu_index = 0
    msg.spark_plug_usage = 0
    # cylinder_status=ArrayValue(type=uavcan.equipment.ice.reciprocating.CylinderStatus[<=16], items=[])
    return msg


def to_uavcan_equipment_esc_Status():
    msg = dronecan.uavcan.equipment.esc.Status()
    msg.error_count = 0
    # Volt
    msg.voltage = 0.0
    # Ampere
    msg.current = 0.0
    # Kelvin
    msg.temperature = celsius_to_kelvin(20.0)
    msg.rpm = 0
    msg.power_rating_pct = 0
    msg.esc_index = 0
    return msg


def to_uavcan_equipment_esc_StatusExtended():
    msg = dronecan.uavcan.equipment.esc.StatusExtended()
    msg.input_pct = 0
    msg.output_pct = 0
    # Celsius in [-256, 255]
    msg.motor_temperature_degC = 20
    # Angle in degrees
    msg.motor_angle = 0
    msg.status_flags = 0
    msg.esc_index = 0
    return msg


def to_uavcan_equipment_gnss_Fix2():
    msg = dronecan.uavcan.equipment.gnss.Fix2()
    msg.timestamp.usec = msg.timestamp.UNKNOWN
    msg.gnss_timestamp.usec = msg.gnss_timestamp.UNKNOWN
    msg.gnss_time_standard = msg.GNSS_TIME_STANDARD_UTC
    msg.num_leap_seconds = msg.NUM_LEAP_SECONDS_UNKNOWN
    msg.longitude_deg_1e8 = 0
    msg.latitude_deg_1e8 = 0
    msg.height_ellipsoid_mm = 0
    msg.height_msl_mm = 0
    msg.ned_velocity = [0.0, 0.0, 0.0]
    msg.sats_used = 0
    msg.status = msg.STATUS_3D_FIX
    msg.mode = msg.MODE_DGPS
    msg.sub_mode = msg.SUB_MODE_DGPS_OTHER
    msg.covariance == []
    msg.pdop = float("nan")
    # dronecan.uavcan.equipment.gnss.ECEFPositionVelocity()
    # XYZ velocity in m/s
    msg.ecef_position_velocity.velocity_xyz = [0.0, 0.0, 0.0]
    # XYZ-axis coordinates in mm
    msg.ecef_position_velocity.position_xyz_mm = [0, 0, 0]
    # Position and velocity covariance in the ECEF frame. Units are m^2 for position,
    # (m/s)^2 for velocity, and m^2/s for position/velocity.
    msg.ecef_position_velocity.covariance = []
    return msg


def to_uavcan_equipment_gnss_Auxiliary():
    msg = dronecan.uavcan.equipment.gnss.Auxiliary()
    msg.gdop = float("nan")
    msg.pdop = float("nan")
    msg.hdop = float("nan")
    msg.vdop = float("nan")
    msg.tdop = float("nan")
    msg.ndop = float("nan")
    msg.edop = float("nan")
    msg.sats_visible = 12
    msg.sats_used = 8
    return msg


def to_ardupilot_gnss_Heading():
    msg = dronecan.ardupilot.gnss.Heading()
    msg.heading_valid = False
    msg.heading_accuracy_valid = False
    # radians
    msg.heading_rad = 0.0
    # radians
    msg.heading_accuracy_rad = 0.0
    return msg


def to_ardupilot_gnss_Status():
    msg = dronecan.ardupilot.gnss.Status()
    msg.error_codes = 0
    msg.healthy = False
    msg.status = msg.STATUS_ARMABLE
    return msg


def to_ardupilot_gnss_MovingBaselineData():
    msg = dronecan.ardupilot.gnss.MovingBaselineData()
    # length of data is set per the number of bytes for pkt in
    # libraries/AP_GPS/RTCM3_Parser.h
    msg.data = [int(0) for x in range(10)]
    return msg


def to_ardupilot_gnss_RelPosHeading():
    msg = dronecan.ardupilot.gnss.RelPosHeading()
    msg.timestamp.usec = msg.timestamp.UNKNOWN
    msg.reported_heading_acc_available = False
    # degrees
    msg.reported_heading_deg = float("nan")
    # degrees
    msg.reported_heading_acc_deg = float("nan")
    # metres
    msg.relative_distance_m = float("nan")
    # metres
    msg.relative_down_pos_m = float("nan")
    return msg


def to_com_hex_equipment_flow_Measurement():
    msg = dronecan.com.hex.equipment.flow.Measurement()
    # Integration Interval in seconds
    msg.integration_interval = 0.0
    # Integrated Gyro Data in radians
    msg.rate_gyro_integral = [0.0, 0.0]
    # Integrated LOS Data in radians
    msg.flow_integral = [0.0, 0.0]
    # Flow Data Quality Lowest(0)-Highest(255) Unitless
    quality = 0
    return msg


def to_ardupilot_equipment_proximity_sensor_Proximity():
    msg = dronecan.ardupilot.equipment.proximity_sensor.Proximity()
    msg.sensor_id = 0
    msg.reading_type = msg.READING_TYPE_GOOD
    msg.flags = msg.FLAGS_NONE
    # degrees in body frame
    msg.yaw = 0.0
    # degrees in body frame
    msg.pitch = 0.0
    # metres
    msg.distance = 0.0
    return msg


def to_uavcan_equipment_range_sensor_Measurement():
    roll_rad = 0.0
    pitch_rad = 0.0
    yaw_rad = 0.0
    msg = dronecan.uavcan.equipment.range_sensor.Measurement()
    msg.timestamp.usec = msg.timestamp.UNKNOWN
    msg.sensor_id = 0

    msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw = [
        int(msg.beam_orientation_in_body_frame.ANGLE_MULTIPLIER * roll_rad),
        int(msg.beam_orientation_in_body_frame.ANGLE_MULTIPLIER * pitch_rad),
        int(msg.beam_orientation_in_body_frame.ANGLE_MULTIPLIER * yaw_rad),
    ]
    msg.beam_orientation_in_body_frame.orientation_defined = True
    msg.field_of_view = 0.0
    msg.sensor_type = msg.SENSOR_TYPE_LIDAR
    msg.reading_type = msg.READING_TYPE_VALID_RANGE
    # metre
    msg.range = 2.0
    return msg


def to_dronecan_sensors_rc_RCInput():
    msg = dronecan.dronecan.sensors.rc.RCInput()
    msg.status = msg.STATUS_QUALITY_VALID
    msg.quality = 255
    msg.id = 0
    msg.rcin = [0 for x in range(16)]
    return msg


def to_dronecan_sensors_rpm_RPM():
    msg = dronecan.dronecan.sensors.rpm.RPM()
    msg.sensor_id = 0
    msg.flags = msg.FLAGS_UNHEALTHY
    msg.rpm = 0.0
    return msg


def to_uavcan_equipment_actuator_Status():
    msg = dronecan.uavcan.equipment.actuator.Status()
    msg.actuator_id = 0
    # metre or radian
    msg.position = float("nan")
    # Newton or Newton metre
    msg.force = float("nan")
    # metre per second or radian per second
    msg.speed = float("nan")
    msg.power_rating_pct = msg.POWER_RATING_PCT_UNKNOWN
    return msg


def to_uavcan_equipment_device_Temperature():
    msg = dronecan.uavcan.equipment.device.Temperature()
    msg.device_id = 0
    # Kelvin
    msg.temperature = celsius_to_kelvin(20.0)
    msg.error_flags = 0
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
    main()
    # show_msg_types()
    # test_converters()
