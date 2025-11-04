"""
Launch multiple copters in Gazebo

Assume Gazebo Jetty (no namespace required for the imports)
"""

import copy
import math
import os

from argparse import ArgumentParser
from pathlib import Path
from transforms3d import euler

GZ_VERSION_GARDEN = "garden"
GZ_VERSION_HARMONIC = "harmonic"
GZ_VERSION_IONIC = "ionic"
GZ_VERSION_JETTY = "jetty"

# TODO: hardcoded path - need to search the GZ_RESOURCE_PATH
GZ_RESOURCE_PATH = (
    "/Users/rhys/Code/ros2/jazzy/ros2-ardupilot/src/ardupilot_gazebo/models"
)

# Index convention for transforms3d quaternions
QUAT_IDX_W = 0
QUAT_IDX_X = 1
QUAT_IDX_Y = 2
QUAT_IDX_Z = 3


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


class GazeboModelFactory:
    def __init__(self, args):
        # Configuration
        self.world_name = args.world
        self.model_name = args.model

        # The sysid and instance may not be coincident
        self.first_sysid = args.first_sysid
        self.first_instance = args.first_instance

        # Layout
        self.x_offset = args.x_offset
        self.y_offset = args.y_offset
        self.z_offset = args.z_offset
        self.x_count = args.x_count
        self.y_count = args.y_count

        # Service call timeout
        self.timeout = args.timeout

        # TODO: hardcoded path - need to search the GZ_RESOURCE_PATH
        # Load the sdf file
        self.path_to_models = Path(GZ_RESOURCE_PATH)

    def create_models(self):
        # Configuration
        world_name = self.world_name
        model_name = self.model_name
        first_sysid = self.first_sysid
        first_instance = self.first_instance
        x_offset = self.x_offset
        y_offset = self.y_offset
        z_offset = self.z_offset
        x_count = self.x_count
        y_count = self.y_count
        timeout = self.timeout
        path_to_models = self.path_to_models

        model_sdf = None
        with open(path_to_models / model_name / "model.sdf", mode="r") as f:
            model_sdf = f.read()
        # print(model_sdf)

        # Set up the service call
        node = gz_node()
        service_name = f"/world/{world_name}/create_multiple"

        # Create all models in a single request
        request = EntityFactory_V()

        # Set global orientation and initial sysid and instance
        q = euler.euler2quat(math.radians(0), math.radians(0), math.radians(90))
        sysid = first_sysid
        instance = first_instance
        for xi in range(x_count):
            for yi in range(y_count):
                # Replace address and port details in sdf
                fdm_addr = "127.0.0.1"
                fdm_port_in = 9002 + 10 * instance

                instance_model_sdf = model_sdf.replace(
                    f"<fdm_addr>127.0.0.1</fdm_addr>",
                    f"<fdm_addr>{fdm_addr}</fdm_addr>",
                )

                instance_model_sdf = instance_model_sdf.replace(
                    f"<fdm_port_in>9002</fdm_port_in>",
                    f"<fdm_port_in>{fdm_port_in}</fdm_port_in>",
                )

                # Create entity
                entity_factory = EntityFactory()
                entity_factory.sdf = instance_model_sdf
                entity_factory.pose.position.x = x_offset * xi
                entity_factory.pose.position.y = y_offset * yi
                entity_factory.pose.position.z = z_offset

                entity_factory.pose.orientation.w = q[QUAT_IDX_W]
                entity_factory.pose.orientation.x = q[QUAT_IDX_X]
                entity_factory.pose.orientation.y = q[QUAT_IDX_Y]
                entity_factory.pose.orientation.z = q[QUAT_IDX_Z]

                entity_factory.name = f"{model_name}_{sysid}"
                entity_factory.allow_renaming = False

                # Append to request
                request.data.append(entity_factory)

                sysid += 1
                instance += 1

        response = Boolean()

        print(f"Creating models: {model_name} x {x_count * y_count}")
        result, response = node.request(
            service_name, request, EntityFactory_V, Boolean, timeout
        )
        print(f"Creating models: result: {result}, response: {response.data}")

    def create_performers(self):
        # gz service
        #     -s /world/levels/level/set_performer
        #     --reqtype gz.msgs.StringMsg
        #     --reptype gz.msgs.Boolean
        #     --timeout 2000
        #     --req 'data: "vehicle_blue"'

        # Configuration
        world_name = self.world_name
        model_name = self.model_name
        first_sysid = self.first_sysid
        first_instance = self.first_instance
        x_offset = self.x_offset
        y_offset = self.y_offset
        z_offset = self.z_offset
        x_count = self.x_count
        y_count = self.y_count
        timeout = self.timeout
        path_to_models = self.path_to_models

        # Set up the service call
        node = gz_node()
        service_name = f"/world/{world_name}/level/set_performer"

        # Create all models in a single request
        request = StringMsg()

        sysid = first_sysid
        for xi in range(x_count):
            for yi in range(y_count):
                entity_name = f"{model_name}_{sysid}"
                print(f"Creating performer: {entity_name} at {xi}, {yi}")

                request.data = entity_name
                response = Boolean()
                result, response = node.request(
                    service_name, request, StringMsg, Boolean, timeout
                )
                print(f"Creating performer: result: {result}, response: {response.data}")
                sysid += 1


class SitlLauncher:
    def __init__(self):
        pass

    def launch(self):
        pass


def main():
    # Command line args
    parser = ArgumentParser(description="Launch Copter Swarm")
    parser.add_argument("--world", default="runway", type=str, help="world name")
    parser.add_argument(
        "--model", default="iris_with_ardupilot", type=str, help="model name"
    )

    parser.add_argument(
        "--x-count", default="1", type=int, help="number of models along the x-axis"
    )
    parser.add_argument(
        "--y-count", default="1", type=int, help="number of models along the y-axis"
    )
    parser.add_argument(
        "--x-offset",
        default="1.0",
        type=float,
        help="offset between models along the x-axis",
    )
    parser.add_argument(
        "--y-offset",
        default="1.0",
        type=float,
        help="offset between models along the y-axis",
    )
    parser.add_argument(
        "--z-offset",
        default="1.0",
        type=float,
        help="offset of the model from the ground",
    )
    parser.add_argument(
        "--timeout", default="5000", type=int, help="timeout for service calls"
    )
    parser.add_argument(
        "--first-sysid", default="1", type=int, help="sysid of the first copter"
    )
    parser.add_argument(
        "--first-instance",
        default="0",
        type=int,
        help="instance index of the first copter",
    )

    args = parser.parse_args()

    # ======================================================================= #
    # Create copter models

    gz_model_factory = GazeboModelFactory(args)
    gz_model_factory.create_models()

    # TODO: check if we are using levels
    gz_model_factory.create_performers()

    # TODO
    # Add levels
    # Add transparent cells / tiles to highlight level boundaries
    gz_level_factory = GazeboModelFactory(args)
    gz_level_factory.model_name = "level_tile"

    gz_level_factory.x_count = 5
    gz_level_factory.y_count = 5
    gz_level_factory.x_offset = 20
    gz_level_factory.y_offset = 20
    gz_level_factory.z_offset = 0.01
    gz_level_factory.create_models()

    # ======================================================================= #
    # Launch SITL

    # Single session to start...
    sitl_launcher = SitlLauncher()
    sitl_launcher.launch()

    # ======================================================================= #


if __name__ == "__main__":
    main()
