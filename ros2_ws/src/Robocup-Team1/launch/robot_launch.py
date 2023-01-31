"""Template Launch File """
import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('Robocup-Team1')
    ## Change urdf with urdf path
    ## Add control configuration Yaml
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', '')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_configuration.yml')

    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    # The accepted arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.
    webots = WebotsLauncher(
        #  Change name to name of world file
        world=os.path.join(package_dir, 'worlds', '')
    )

    # The Ros2Supervisor node is a special node interacting with the simulation.
    # For example, it publishes the /clock topic of the simulation or permits to spawn robot from URDF files.
    # By default, the respawn option is set to True.
    ros2_supervisor = Ros2SupervisorLauncher()

    # The node which interacts with a robot in the Webots simulation is located in the `webots_ros2_driver` package under name `driver`.
    # It is necessary to run such a node for each robot in the simulation.
    # The `WEBOTS_CONTROLLER_URL` variable has to match the robot name in the world file.
    # Typically, we can provide it the `robot_description` parameters from a URDF file in order to configure the interface, `set_robot_state_publisher` in order to let the node give the URDF generated from Webots to the `robot_state_publisher` and `ros2_control_params` from the `ros2_control` configuration file.
    webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': 'robot'},
        parameters=[
            {'robot_description': robot_description,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
    )

    # Often we want to publish robot transforms, so we use the `robot_state_publisher` node for that.
    # If robot model is not specified in the URDF file then Webots can help us with the URDF exportation feature.
    # Since the exportation feature is available only once the simulation has started and the `robot_state_publisher` node requires a `robot_description` parameter before we have to specify a dummy robot.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([

        # Start the Webots node
        webots,

        # Start the Ros2Supervisor node
        ros2_supervisor,

        # Start the Webots robot driver
        webots_robot_driver,

        # Start the robot_state_publisher
        robot_state_publisher,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])

