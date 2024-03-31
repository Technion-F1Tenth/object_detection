from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare a launch argument for the container image and venv path
    container_image_arg = DeclareLaunchArgument(
        'container_image',
        default_value='f1tenth_object_detection:latest',
        description='Docker container image name'
    )
    venv_path_arg = DeclareLaunchArgument(
        'venv_path',
        default_value='/home/doof-wagon/f1tenth_ws/object_detection/object_detection/venv/',
        description='Path to the virtual environment inside the container'
    )

    # Define the launch description with the container and venv activation
    return LaunchDescription([
        container_image_arg,
        venv_path_arg,

        # Activate the virtual environment in the container
        Command(
            name='activate_venv',
            cmd=['source', LaunchConfiguration('venv_path') + '/bin/activate']
        ),

        ComposableNodeContainer(
            name='docker_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='object_detection',
                    plugin='object_detection/object_detection_node',
                    name='object_detection_node',
                    namespace='',
                    # parameters=[{'param_name': 'param_value'}]
                )
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
