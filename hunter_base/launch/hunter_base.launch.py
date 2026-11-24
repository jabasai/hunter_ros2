import os

import launch_ros

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def _hunter_base_node(context):
	# Per-robot parameters (wheel-odometry calibration) from the robot
	# configuration: robot_configurations params merged by zgb-client into
	# /robot_configs/ros_params/hunter_base.yaml. Without the file the driver
	# runs uncalibrated (raw chassis speed/steering), exactly as before.
	params_file = LaunchConfiguration('params_file').perform(context)
	parameters = []
	actions = []
	if params_file and os.path.isfile(params_file):
		parameters.append(params_file)
		actions.append(LogInfo(msg=f'hunter_base: parameters from {params_file}'))
	else:
		actions.append(LogInfo(
			msg=f'hunter_base: no parameter file at "{params_file}"; '
			'wheel odometry is uncalibrated'))
	parameters.append({
		'use_sim_time': LaunchConfiguration('use_sim_time'),
		'port_name': LaunchConfiguration('port_name'),
		'odom_frame': LaunchConfiguration('odom_frame'),
		'base_frame': LaunchConfiguration('base_frame'),
		'odom_topic_name': LaunchConfiguration('odom_topic_name'),
		'simulated_robot': LaunchConfiguration('simulated_robot'),
		'control_rate': LaunchConfiguration('control_rate'),
		'robot_model': LaunchConfiguration('robot_model'),
		'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
		'state_glitch_filter_sec': LaunchConfiguration('state_glitch_filter_sec'),
	})
	actions.append(launch_ros.actions.Node(
		package='hunter_base',
		executable='hunter_base_node',
		output='screen',
		emulate_tty=True,
		parameters=parameters,
		remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
	))
	return actions


def generate_launch_description():
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
		DeclareLaunchArgument(
			'state_glitch_filter_sec',
			default_value='1.0',
			description='Debounce (s) for vehicle_state/error_code changes '
			'while the base reports RC signal loss; 0 disables',
		),
		DeclareLaunchArgument(
			'params_file',
			default_value=EnvironmentVariable(
				'HUNTER_BASE_PARAMS_FILE',
				default_value='/robot_configs/ros_params/hunter_base.yaml'),
			description='ROS parameter file with the per-robot wheel-odometry '
			'calibration (linear_velocity_scale, steering_angle_scale, '
			'steering_angle_offset, steering_angle_left_right_asymmetry, '
			'steering_angle_forward_reverse_asymmetry, odom_reference_offset_x); '
			'ignored if missing',
		),
		OpaqueFunction(function=_hunter_base_node),
	])
