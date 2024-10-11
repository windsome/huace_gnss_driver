import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

# Start as component:

def generate_launch_description():
    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "base_link", "--child-frame-id", "imu"]
    )

    tf_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "imu", "--child-frame-id", "gnss"]
    )

    tf_vsm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "imu", "--child-frame-id", "vsm"]
    )

    tf_aux1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "imu", "--child-frame-id", "aux1"]
    )

    default_file_name = 'huace.yaml'
    name_arg_file_name = "file_name"
    arg_file_name = DeclareLaunchArgument(name_arg_file_name,
                                          default_value=TextSubstitution(text=str(default_file_name)))
    name_arg_file_path = 'path_to_config'
    arg_file_path = DeclareLaunchArgument(name_arg_file_path,
                                          default_value=[get_package_share_directory('huace_gnss_driver'), '/config/', LaunchConfiguration(name_arg_file_name)])

    node = Node(
        package='huace_gnss_driver',
        executable='huace_gnss_node_exe',
        name='huace_gnss_node',
        emulate_tty=True,
        sigterm_timeout = '20',
        parameters=[LaunchConfiguration(name_arg_file_path)],
        output='screen'
    )

    rviz_dir = os.path.join(get_package_share_directory('huace_gnss_driver'), 'rviz_cfg', 'huace_gnss_imu.rviz')
    rviz_node = Node(
        package='rviz2',
        namespace='cx',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')

    return launch.LaunchDescription([arg_file_name, arg_file_path, node, tf_imu, tf_gnss, tf_vsm, tf_aux1, rviz_node])
