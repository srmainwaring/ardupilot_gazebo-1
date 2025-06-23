"""
Script to autotune the PIDs of a joint position controller using Ziegler-Nichols method


Bridge topics from Gazebo to ROS 2

  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/scripts/autotune_bridge.yaml

"""

import math
import numpy as np
import time
from typing import List, Tuple

from gz.msgs11.clock_pb2 import Clock
from gz.msgs11.double_pb2 import Double
from gz.msgs11.model_pb2 import Model
from gz.transport14 import Node

from ardupilot_gazebo.scripts.pid_params import set_param, InvalidParameterError


class ClockSubscriber:
    def __init__(self):
        super().__init__()
        self.node = Node()
        self.last_clock = Clock()
        self.topic = "/clock"

        def callback(msg: Clock) -> None:
            self.last_clock = msg

        if self.node.subscribe(Clock, self.topic, callback):
            print("Subscribing to type {} on topic [{}]".format(Clock, self.topic))
        else:
            print("Error subscribing to topic [{}]".format(self.topic))
            return

    def get_sim_time(self):
        sec = self.last_clock.sim.sec
        nsec = self.last_clock.sim.nsec
        return sec + 1.0e-9 * nsec


class JointStateSubscriber:
    """Subscribe to the joint states of a model"""

    def __init__(self, world_name, model_name):
        super().__init__()
        self._world_name = world_name
        self._model_name = model_name

        self.node = Node()
        self.last_model = Model()

        self.subscribe(world_name, model_name)

    def subscribe(self, world_name, model_name):
        """Subscribe to a joint state"""
        topic = f"/world/{world_name}/model/{model_name}/joint_state"
        topic = self.make_valid_topic(topic)

        # Callback to capture the last msgs received
        def callback(msg: Model) -> None:
            self.last_model = msg

        # Subscribe to a topic by registering a callback
        if self.node.subscribe(Model, topic, callback):
            print("Subscribing to type {} on topic [{}]".format(Model, topic))
        else:
            print("Error subscribing to topic [{}]".format(topic))
            return

    def make_valid_topic(self, topic):
        """Utility to replace `::` with `_` to ensure the topic name is valid"""
        return topic.replace("::", "_")

    def get_joint_state(self, joint_name):
        """Get the joint state for the named joint"""
        for joint in self.last_model.joint:
            if joint.name == joint_name:
                return joint
        return None

    @staticmethod
    def get_axis1_position(joint_state):
        """Utility to access the position of axis1 in a joint state"""
        return joint_state.axis1.position


class JointCommandPublisher:
    def __init__(self, topic: str):
        super().__init__()
        self.topic = topic
        self.node = Node()
        self.pub = self.node.advertise(self.topic, Double)

    def set_position(self, position: float):
        msg = Double()
        msg.data = position
        self.pub.publish(msg)


class JointAutotuner:
    def __init__(
        self, world_name: str, model_name: str, joint_name: str, timeout_ms: int = 2000
    ):
        self.world_name = world_name
        self.model_name = model_name
        self.joint_name = joint_name
        self.timeout = timeout_ms

        self.registry = f"/world/{world_name}"
        self.prefix = f"JointPositionController.{world_name}.{model_name}.{joint_name}."

        self.node = Node()
        self.sample_time = 0.01  # 10ms

        self.joint_state_sub = JointStateSubscriber(world_name, model_name)

    def set_gains(self, p: float, i: float, d: float) -> None:
        """Set PID gains for the joint"""
        try:
            set_param(self.registry, f"{self.prefix}p_gain", p, self.timeout)
            set_param(self.registry, f"{self.prefix}i_gain", i, self.timeout)
            set_param(self.registry, f"{self.prefix}d_gain", d, self.timeout)
        except InvalidParameterError as e:
            print(f"Failed to set gains: {e}")

    def get_position(self) -> float:
        """Get current joint position"""
        joint_state = self.joint_state_sub.get_joint_state(self.joint_name)
        if joint_state is not None:
            pos = JointStateSubscriber.get_axis1_position(joint_state)
            return pos
        else:
            return 0.0

    def find_ultimate_gain(self) -> Tuple[float, float]:
        """Find ultimate gain Ku and period Tu using Ziegler-Nichols method"""
        Tu = 1.0
        Ku = 512  # Initial P gain
        Ku_max = 10000
        step = 2.0  # Initial step size (multiplier)
        oscillating = False
        positions: List[float] = []

        while not oscillating and Ku < Ku_max:
            # Set P controller only
            print(f"Set p_gain: {Ku}")
            self.set_gains(Ku, 0.0, 0.0)

            # Record positions for 5 seconds
            start_time = time.time()
            print("Record positions")
            while time.time() - start_time < 5.0:
                positions.append(self.get_position())
                time.sleep(self.sample_time)

            # Check for sustained oscillations
            peaks = self._find_peaks(positions)
            if len(peaks) >= 4:  # Need at least 2 complete cycles
                # Check if amplitude is consistent
                amplitudes = np.diff(peaks)
                print(f"Amplitudes: {amplitudes}")
                if np.std(amplitudes) < 0.1 * np.mean(amplitudes):
                    print(f"Is oscillating")
                    oscillating = True
                    Tu = self._calculate_period(peaks)
                    break

            # Ku += step
            Ku *= step
            positions.clear()

        return Ku, Tu

    def _find_peaks(self, data: List[float]) -> List[float]:
        """Find peaks in position data"""
        peaks = []
        for i in range(1, len(data) - 1):
            if data[i] > data[i - 1] and data[i] > data[i + 1]:
                peaks.append(data[i])
        return peaks

    def _calculate_period(self, peaks: List[float]) -> float:
        """Calculate oscillation period from peaks"""
        return len(peaks) * self.sample_time

    def autotune(self) -> Tuple[float, float, float]:
        """Run autotuning process and return PID gains"""
        print("Starting autotuning process...")

        # Find ultimate gain and period
        Ku, Tu = self.find_ultimate_gain()
        print(f"Found ultimate gain Ku: {Ku:.3f}, period Tu: {Tu:.3f}s")

        # Calculate PID gains using Ziegler-Nichols method
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / Tu
        Kd = 0.075 * Ku * Tu

        # Set final gains
        self.set_gains(Kp, Ki, Kd)

        print(f"Autotuning complete. Gains: P={Kp:.3f}, I={Ki:.3f}, D={Kd:.3f}")
        return Kp, Ki, Kd


def main():
    print("Running autotune")

    # Configuration
    world_name = "default"
    model_name = "joint_position_controller_demo"
    joint_name = "j1"
    timeout_ms = 1000

    # Create and run autotuner
    # tuner = JointAutotuner(world_name, model_name, joint_name, timeout_ms)
    # tuner.autotune()

    # Create subscribers
    clock_sub = ClockSubscriber()
    joint_sub = JointStateSubscriber(world_name, model_name)

    # Create publishers
    joint_cmd = JointCommandPublisher("/rotor_cmd")

    # Dwell period
    period = 2.0
    limit_max = math.radians(60.0)
    limit_min = -math.radians(60.0)

    # control the twitch freqency
    start_time = time.monotonic()
    twitch_position = limit_max
    twitch_period = 5.0
    last_twitch_time = start_time

    # control the update rate
    update_rate = 100.0
    update_period = 1.0 / update_rate

    pos_tgt = 0.0
    pos_act = 0.0
    pos_err = 0.0

    try:
        while True:
            # update the target position
            if (time.monotonic() - last_twitch_time) > twitch_period:
                pos_tgt = twitch_position
                twitch_position *= -1.0
                last_twitch_time = time.monotonic()

            # command the joint to move
            joint_cmd.set_position(pos_tgt)

            # monitor the response
            joint_state = joint_sub.get_joint_state(joint_name)
            if joint_state is not None:
                pos_act = JointStateSubscriber.get_axis1_position(joint_state)
                pos_err = pos_tgt - pos_act
                # print(
                #     f"pos_tgt: {pos_tgt:.3f}, " f"pos_act: {pos_act:.3f}",
                #     f"pos_err: {pos_err:.3f}",
                # )

            time.sleep(update_period)

    except KeyboardInterrupt:
        print("Exiting")


if __name__ == "__main__":
    main()
