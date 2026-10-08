import launch
import launch_ros

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time')
	port_name = LaunchConfiguration('port_name')
	odom_frame = LaunchConfiguration('odom_frame')
	base_frame = LaunchConfiguration('base_frame')
	odom_topic_name = LaunchConfiguration('odom_topic_name')
	cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
	robot_model = LaunchConfiguration('robot_model')
	simulated_robot = LaunchConfiguration('simulated_robot')
	control_rate = LaunchConfiguration('control_rate')
	publish_odom_tf = LaunchConfiguration('publish_odom_tf')

	hunter_base_node = launch_ros.actions.Node(
		package='hunter_base',
		executable='hunter_base_node',
		output='screen',
		emulate_tty=True,
		parameters=[{
			'use_sim_time': use_sim_time,
			'port_name': port_name,
			'odom_frame': odom_frame,
			'base_frame': base_frame,
			'odom_topic_name': odom_topic_name,
			'simulated_robot': simulated_robot,
			'control_rate': control_rate,
			'robot_model': robot_model,
			'publish_odom_tf': publish_odom_tf,
		}],
		remappings=[('/cmd_vel', cmd_vel_topic)],
	)

	return LaunchDescription([
		DeclareLaunchArgument('use_sim_time', default_value='false'),
		DeclareLaunchArgument('port_name', default_value='can0'),
		DeclareLaunchArgument('odom_frame', default_value='odom'),
		DeclareLaunchArgument('base_frame', default_value='base_link'),
		DeclareLaunchArgument('odom_topic_name', default_value='odom'),
		DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
		DeclareLaunchArgument('robot_model', default_value='hunter2'),
		DeclareLaunchArgument('simulated_robot', default_value='false'),
		DeclareLaunchArgument('control_rate', default_value='50'),
		DeclareLaunchArgument(
			'publish_odom_tf',
			default_value='true',
			description='Whether hunter_base broadcasts odom to base TF',
		),
		hunter_base_node,
	])
