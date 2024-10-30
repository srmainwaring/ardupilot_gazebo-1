"""
Publish sensor data using DroneCAN
"""

import dronecan
import threading
import time

GZ_VERSION_GARDEN = "garden"
GZ_VERSION_HARMONIC = "harmonic"
GZ_VERSION_IONIC = "ionic"

GZ_VERSION = GZ_VERSION_HARMONIC

if GZ_VERSION == GZ_VERSION_GARDEN:
    from gz.msgs9.fluid_pressure_pb2 import FluidPressure
    from gz.msgs9.air_speed_pb2 import AirSpeed
    from gz.msgs9.altimeter_pb2 import Altimeter
    from gz.msgs9.magnetometer_pb2 import Magnetometer
    from gz.msgs9.navsat_pb2 import NavSat
    from gz.msgs9.imu_pb2 import IMU

    from gz.transport12 import Node

elif GZ_VERSION == GZ_VERSION_HARMONIC:
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.air_speed_pb2 import AirSpeed
    from gz.msgs10.altimeter_pb2 import Altimeter
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.msgs10.imu_pb2 import IMU

    from gz.transport13 import Node

elif GZ_VERSION == GZ_VERSION_IONIC:
    from gz.msgs11.fluid_pressure_pb2 import FluidPressure
    from gz.msgs11.air_speed_pb2 import AirSpeed
    from gz.msgs11.altimeter_pb2 import Altimeter
    from gz.msgs11.magnetometer_pb2 import Magnetometer
    from gz.msgs11.navsat_pb2 import NavSat
    from gz.msgs11.imu_pb2 import IMU

    from gz.transport14 import Node


def sensor_topics():
    world = "iris_runway"
    model = "iris_with_gimbal"
    sub_model = "iris_with_standoffs"
    link = "base_link"

    topic = (
        f"/world/{world}/model/{model}"
        f"/model/{sub_model}/link/{link}"
        f"/sensor/air_pressure_sensor/air_pressure"
    )

    topic = (
        f"/world/{world}/model/{model}"
        f"/model/{sub_model}/link/base_link"
        f"/sensor/air_speed_sensor/air_speed"
    )

    topic = (
        f"/world/{world}/model/{model}"
        f"/model/{sub_model}/link/{link}"
        f"/sensor/altimeter_sensor/altimeter"
    )

    topic = (
        f"/world/{world}/model/{model}"
        f"/model/{sub_model}/link/{link}"
        f"/sensor/magnetometer_sensor/magnetometer"
    )

    topic = (
        f"/world/{world}/model/{model}"
        f"/model/{sub_model}/link/{link}"
        f"/sensor/navsat_sensor/navsat"
    )

    link = "imu_link"

    topic = (
        f"/world/{world}/model/{model}"
        f"/model/{sub_model}/link/{link}"
        f"/sensor/imu_sensor/imu"
    )


class SensorSubscribers:
    def __init__(self):

        world = "iris_runway"
        model = "iris_with_gimbal"
        sub_model = "iris_with_standoffs"
        link = "base_link"

        self._lock = threading.Lock()
        self._node = Node()

        # Magnetometer
        self._sensor_do_print_msg = True
        self._sensor_msg = None
        self._sensor_topic = (
            f"/world/{world}/model/{model}"
            f"/model/{sub_model}/link/{link}"
            f"/sensor/magnetometer_sensor/magnetometer"
        )

        self._sensor_sub = self._node.subscribe(
            Magnetometer, self._sensor_topic, self._sensor_cb
        )

    def _sensor_cb(self, msg):
        with self._lock:
            self._sensor_msg = msg
            do_print_msg = self._sensor_do_print_msg

        if do_print_msg:
            print(msg)


class DroneCANNode():
    def __init__(self):
        pass





def main():
    sensor_topics()

    sensor_subs = SensorSubscribers()

    while True:
        time.sleep(0.01)


if __name__ == "__main__":
    main()
