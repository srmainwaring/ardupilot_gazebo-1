"""
Script to subscribe to forces and torques published by the
VisualizeForces plugin.

For this script:
- heli_dual_transverse mounted on gimble
- all axes of rotation locked (angle derived from blade world pose)


"""

import copy
import math
import numpy as np
import time

import matplotlib.pyplot as plt
import numpy as np

from typing import List, Tuple

from transforms3d import euler
from transforms3d import quaternions

from gz.msgs11.entity_wrench_map_pb2 import EntityWrenchMap
from gz.msgs11.entity_wrench_pb2 import EntityWrench
from gz.msgs11.double_pb2 import Double
from gz.msgs11.model_pb2 import Model
from gz.transport14 import Node


def wrap_pi(angle: float) -> float:
    """
    Return an angle constrained to [-pi, pi]

    :param angle: An angle in radians
    :type angle: float
    :return: The angle in radians in [-pi, pi]
    :rtype: float
    """
    while math.fabs(angle) > math.pi:
        if angle > 0:
            angle = angle - 2 * math.pi
        else:
            angle = angle + 2 * math.pi

    return angle


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


class ForceTorqueSubscriber:
    def __init__(self):
        model = "rotor_head_ccw"
        link = "blade_1_link"

        self.node = Node()
        self.last_msg = EntityWrenchMap()
        self.topic = f"/model/{model}/link/{link}/forces/lift_drag"

        def callback(msg: EntityWrenchMap) -> None:
            self.last_msg = msg

        if self.node.subscribe(EntityWrenchMap, self.topic, callback):
            print(
                "Subscribing to type {} on topic [{}]".format(
                    EntityWrenchMap, self.topic
                )
            )
        else:
            print("Error subscribing to topic [{}]".format(self.topic))
            return

    @staticmethod
    def lift_wrench(msg: EntityWrenchMap) -> EntityWrench:
        wrench = msg.wrenches["Lift"]
        return wrench

    @staticmethod
    def drag_wrench(msg: EntityWrenchMap) -> EntityWrench:
        wrench = msg.wrenches["Drag"]
        return wrench


def main():
    print("Running heli rotor force analysis")

    # Configuration
    world_name = "empty"
    model_name = "rotor_head_ccw"

    # Create subscribers
    force_sub = ForceTorqueSubscriber()
    joint_sub = JointStateSubscriber(
        world_name,
        f"gimbal/model/helicopter_dual_transverse/model/{model_name}",
    )

    # control the update rate
    start_time = time.monotonic()
    update_rate = 1000.0
    update_period = 1.0 / update_rate
    last_update_time = start_time

    # control the sample rate    start_time = time.monotonic()
    sample_rate = 200.0
    sample_period = 1.0 / sample_rate
    last_sample_time = start_time

    sample_count_max = 1000
    sample_count = 0
    sample_angles = []
    sample_lift_f_z = []
    sample_lift_t_x = []
    sample_lift_t_y = []
    sample_lift_t_z = []
    sample_drag_f_xy = []
    sample_blade_pitch = []
    sample_rotor_angle = []
    sample_cp_pos_x = []
    sample_cp_pos_y = []

    """
    Notes:

    Relationship between rotor_head_ccw main joint position and blade_1_link
    position in world frame.

    After reset (t=0)

    blade joint state
    - message is a Model
    - extract the Joint from the model
    - in the helicopter model the joint x-axis intially points to the front
      of the vehicle (which is initially aligned to the world y-axis)   

    blade force
    - message is EntityWrench
    - entity (link) pose is in the world frame
    - pose is zero when the link x-axis aligns with the world x-axis (east)
    - this is 90 deg out of phase with the rotor main_joint  
    - there is a further 13 deg offset between the rotor main link and the
      rotor base

    blade_1_link x-axis aligned to front of vehicle (north) 
    blade angle = 90
    rotor angle = 167 = 180 - 13

    To align:
    blade angle -= 90
    rotor angle -= 167


    Observations:
    - blade lift is getting clipped once the blade pitch reaches cla_atall
      which is about 11 deg
    - the clipping gets worse as the collective is increased
    

    Additional measurements required
    - what are the torques about the x-axis and y-axis when the swashplate is
      pitched forward (rc 2 1000)



    """

    blade_angle_offset = math.radians(-90)
    rotor_angle_offset = math.radians(-180 + 13)

    try:
        while sample_count < sample_count_max:
            # update the target position
            if (time.monotonic() - last_sample_time) > sample_period:

                # process joint states
                blade_joint_state = joint_sub.get_joint_state("blade_grip_1_joint")
                if blade_joint_state is not None:
                    joint_pos = JointStateSubscriber.get_axis1_position(
                        blade_joint_state
                    )
                    joint_pos_deg = math.degrees(joint_pos)
                    sample_blade_pitch.append(joint_pos_deg)
                    # print(f"blade pitch: {joint_pos_deg:.1f}")

                rotor_joint_state = joint_sub.get_joint_state("main_joint")
                if rotor_joint_state is not None:
                    joint_pos = JointStateSubscriber.get_axis1_position(
                        rotor_joint_state
                    )
                    joint_pos_rad = wrap_pi(joint_pos + rotor_angle_offset)
                    joint_pos_deg = math.degrees(joint_pos_rad)
                    sample_rotor_angle.append(joint_pos_deg)
                    # print(f"blade angle: {joint_pos_deg:.1f}")

                # rotor angle and blade pitch
                # print(f"angle: {joint_pos_deg:.1f}, pitch: {joint_pos_deg:.1f}")

                # process wrench message
                wrench = copy.deepcopy(force_sub.last_msg)
                lift_wrench = ForceTorqueSubscriber.lift_wrench(wrench)
                drag_wrench = ForceTorqueSubscriber.drag_wrench(wrench)

                lift_q = [
                    lift_wrench.pose.orientation.w,
                    lift_wrench.pose.orientation.x,
                    lift_wrench.pose.orientation.y,
                    lift_wrench.pose.orientation.z,
                ]
                lift_angles = euler.quat2euler(lift_q)

                # drag_q = [
                #     drag_wrench.pose.orientation.w,
                #     drag_wrench.pose.orientation.x,
                #     drag_wrench.pose.orientation.y,
                #     drag_wrench.pose.orientation.z,
                # ]
                # drag_angles = euler.quat2euler(drag_q)

                lift_f = lift_wrench.wrench.force
                # lift_t = lift_wrench.wrench.torque
                drag_f = drag_wrench.wrench.force
                # drag_t = drag_wrench.wrench.torque

                angle_rad = wrap_pi(lift_angles[2] + blade_angle_offset)
                angle_deg = math.degrees(angle_rad)
                drag_f_xy = math.sqrt(drag_f.x * drag_f.x + drag_f.y * drag_f.y)

                # calculate the torque about the rotor shaft
                # r_mag calculated from max(cp_pos_x) - min(cp_pos_x)
                r_mag = 0.225
                r_x = r_mag * math.cos(angle_rad)
                r_y = r_mag * math.sin(angle_rad)
                r = np.array([r_x, r_y, 0.0])
                f = np.array([0.0, 0.0, lift_f.z])
                lift_t = np.cross(r, f) 

                sample_angles.append(angle_deg)
                sample_lift_f_z.append(lift_f.z)
                sample_lift_t_x.append(lift_t[0])
                sample_lift_t_y.append(lift_t[1])
                sample_lift_t_z.append(lift_t[2])
                sample_drag_f_xy.append(drag_f_xy)
                sample_cp_pos_x.append(lift_wrench.pose.position.x)
                sample_cp_pos_y.append(lift_wrench.pose.position.y)

                # print(
                #     f"angle: {angle_deg:.1f}, "
                #     f"lift: {lift_f.z:.1f}, "
                #     f"drag: {drag_f_xy:.1f}, "
                #     f"lift_t.x: {lift_t.x:.1f}, "
                #     f"lift_t.y: {lift_t.y:.1f}"
                # )

                sample_count += 1

                last_sample_time = time.monotonic()

            # throttle update time
            if (time.monotonic() - last_update_time) > update_period:
                time.sleep(0.1 * update_period)
                last_update_time = time.monotonic()

    except KeyboardInterrupt:
        print("Exiting")

    # labels
    roll = 1500
    pitch = 1000
    collective = 1000

    # convert to np.array and flip sign on blade pitch (due to joint orientation / rotor direction)
    sample_angles = np.array(sample_angles)
    sample_lift_f_z = np.array(sample_lift_f_z)
    sample_drag_f_xy = np.array(sample_drag_f_xy)
    sample_lift_t_x = np.array(sample_lift_t_x)
    sample_lift_t_y = np.array(sample_lift_t_y)
    sample_lift_t_z = np.array(sample_lift_t_z)
    sample_rotor_angle = np.array(sample_rotor_angle)
    sample_blade_pitch = np.array(sample_blade_pitch) * -1.0

    def plot_blade_pitch(sample_rotor_angle, sample_blade_pitch):
        indices = np.argsort(sample_rotor_angle)

        # setup plot
        ax = plt.figure().add_subplot()
        ax.grid(True, which='both')
        ax.set_xlim(-180.0, 180.0)
        ax.set_ylim(-15.0, 25.0)
        ax.set_xlabel(f"rotor angle (deg)")
        ax.set_ylabel(f"blade pitch (deg)")
        ax.set_title(
            f"Cyclic blade pitch: roll: {roll}, pitch: {pitch}, collective: {collective}"
        )
        ax.step(sample_rotor_angle[indices], sample_blade_pitch[indices], label=r"pitch")
        ax.legend()

        plt.show()

    def plot_blade_lift_f(sample_angles, sample_lift_f_z, sample_drag_f_xy):
        indices = np.argsort(sample_angles)
        # unique_sample_angles, indices, unique_inverse, unique_counts = np.unique(
        #     sample_angles, return_index=True, return_inverse=True, return_counts=True)
        # print(unique_sample_angles.shape)
        # print([x for x in unique_counts if x > 1])

        # setup plot
        ax = plt.figure().add_subplot()
        ax.grid(True, which='both')
        ax.set_xlim(-180.0, 180.0)
        ax.set_ylim(-50.0, 50.0)
        ax.set_xlabel(f"rotor angle (deg)")
        ax.set_ylabel(f"lift (N)")
        ax.set_title(
            f"Cyclic blade lift: roll: {roll}, pitch: {pitch}, collective: {collective}"
        )
        ax.step(sample_angles[indices], sample_lift_f_z[indices], label=r"$F_z$")
        ax.legend()

        plt.show()

    def plot_blade_lift_t(sample_angles, sample_lift_t_x, sample_lift_t_y, sample_lift_t_z):
        indices = np.argsort(sample_angles)

        # setup plot
        ax = plt.figure().add_subplot()
        ax.grid(True, which='both')
        ax.set_xlim(-180.0, 180.0)
        ax.set_ylim(-10.0, 10.0)
        ax.set_xlabel(f"rotor angle (deg)")
        ax.set_ylabel(f"lift torque (N.m)")
        ax.set_title(
            f"Cyclic blade lift torque: roll: {roll}, pitch: {pitch}, collective: {collective}"
        )
        ax.step(sample_angles[indices], sample_lift_t_x[indices], label=r"$\tau_x$")
        ax.step(sample_angles[indices], sample_lift_t_y[indices], label=r"$\tau_y$")
        ax.step(sample_angles[indices], sample_lift_t_z[indices], label=r"$\tau_z$")
        ax.legend()

        plt.show()

    print(f"pitch: max: {np.max(sample_blade_pitch)}, min: {np.min(sample_blade_pitch)}")
    print(f"lift: max: {np.max(sample_lift_f_z)}, min: {np.min(sample_lift_f_z)}")
    print(f"drag: max: {np.max(sample_drag_f_xy)}, min: {np.min(sample_drag_f_xy)}")
    print(f"t_x: avg: {np.average(sample_lift_t_x)}")
    print(f"t_y: avg: {np.average(sample_lift_t_y)}")
    print(f"pos_x: max: {np.max(sample_cp_pos_x)}, min: {np.min(sample_cp_pos_x)}")
    print(f"pos_y: max: {np.max(sample_cp_pos_y)}, min: {np.min(sample_cp_pos_y)}")

    plot_blade_pitch(sample_rotor_angle, sample_blade_pitch)
    plot_blade_lift_f(sample_angles, sample_lift_f_z, sample_drag_f_xy)
    plot_blade_lift_t(sample_angles, sample_lift_t_x, sample_lift_t_y, sample_lift_t_z)


if __name__ == "__main__":
    main()
