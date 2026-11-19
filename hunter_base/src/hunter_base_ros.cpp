/*
 * hunter_base_ros.cpp
 *
 * Created on: Oct 15, 2021 14:35
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "hunter_base/hunter_base_ros.hpp"

#include "hunter_base/hunter_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
HunterBaseRos::HunterBaseRos(std::string node_name)
		: rclcpp::Node(node_name), keep_running_(false) {
	this->declare_parameter("port_name", rclcpp::ParameterValue("can0"));
	this->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
	this->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
	this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));
	this->declare_parameter("robot_model", rclcpp::ParameterValue("hunter2"));
	this->declare_parameter("is_hunter_mini", rclcpp::ParameterValue(false));
	this->declare_parameter("is_omni_wheel", rclcpp::ParameterValue(false));
	this->declare_parameter("simulated_robot", rclcpp::ParameterValue(false));
	this->declare_parameter("control_rate", rclcpp::ParameterValue(50));
	this->declare_parameter("publish_odom_tf", rclcpp::ParameterValue(true));
	this->declare_parameter("state_glitch_filter_sec",
													rclcpp::ParameterValue(1.0));
	// Wheel-odometry calibration (jabas_calibration). Defaults = uncalibrated.
	this->declare_parameter("linear_velocity_scale", rclcpp::ParameterValue(1.0));
	this->declare_parameter("steering_angle_scale", rclcpp::ParameterValue(1.0));
	this->declare_parameter("steering_angle_offset", rclcpp::ParameterValue(0.0));
	this->declare_parameter("odom_reference_offset_x", rclcpp::ParameterValue(0.0));

	LoadParameters();
}

void HunterBaseRos::LoadParameters() {
	this->get_parameter_or<std::string>("port_name", port_name_, "can0");
	this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
	this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
	this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
																			"odom");
	this->get_parameter_or<std::string>("robot_model", robot_model_, "hunter2");
	this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
	this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);
	this->get_parameter_or<bool>("publish_odom_tf", publish_odom_tf_, true);
	this->get_parameter_or<double>("state_glitch_filter_sec",
																 state_glitch_filter_sec_, 1.0);
	this->get_parameter_or<double>("linear_velocity_scale",
																 calibration_.linear_velocity_scale, 1.0);
	this->get_parameter_or<double>("steering_angle_scale",
																 calibration_.steering_angle_scale, 1.0);
	this->get_parameter_or<double>("steering_angle_offset",
																 calibration_.steering_angle_offset, 0.0);
	this->get_parameter_or<double>("odom_reference_offset_x",
																 calibration_.odom_reference_offset_x, 0.0);
	// fail at startup on an implausible calibration rather than drive with it
	calibration_.Validate();

	std::cout << "Loading parameters: " << std::endl;
	std::cout << "- port name: " << port_name_ << std::endl;
	std::cout << "- odom frame name: " << odom_frame_ << std::endl;
	std::cout << "- base frame name: " << base_frame_ << std::endl;
	std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;
	std::cout << "- publish odom TF: " << std::boolalpha << publish_odom_tf_
						<< std::endl;
	std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
						<< std::endl;
	std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
	std::cout << "- state glitch filter (s): " << state_glitch_filter_sec_
						<< std::endl;
	std::cout << "- wheel calibration: "
						<< (calibration_.IsIdentity() ? "none (raw chassis values)" : "applied")
						<< std::endl;
	std::cout << "    linear_velocity_scale: " << calibration_.linear_velocity_scale << std::endl;
	std::cout << "    steering_angle_scale: " << calibration_.steering_angle_scale << std::endl;
	std::cout << "    steering_angle_offset (rad): " << calibration_.steering_angle_offset << std::endl;
	std::cout << "    odom_reference_offset_x (m): " << calibration_.odom_reference_offset_x << std::endl;
	std::cout << "----------------------------" << std::endl;
}

bool HunterBaseRos::Initialize() {
	std::cout << "Robot base: Hunter" << std::endl;
	ProtocolDetector detector;
	if (detector.Connect(port_name_)) {
		auto proto = detector.DetectProtocolVersion(5);
		if (proto == ProtocolVersion::AGX_V1) {
			std::cout << "Detected protocol: AGX_V1" << std::endl;
			robot_ = std::unique_ptr<HunterRobot>(
					new HunterRobot(ProtocolVersion::AGX_V1));
		} else if (proto == ProtocolVersion::AGX_V2) {
			std::cout << "Detected protocol: AGX_V2" << std::endl;
			robot_ = std::unique_ptr<HunterRobot>(
					new HunterRobot(ProtocolVersion::AGX_V2));
		} else {
			std::cout << "Detected protocol: UNKONWN" << std::endl;
			return false;
		}
	} else {
		return false;
	}

	return true;
}

void HunterBaseRos::Stop() { keep_running_ = false; }

void HunterBaseRos::Run() {
	std::unique_ptr<HunterMessenger<HunterRobot>> messenger =
			std::unique_ptr<HunterMessenger<HunterRobot>>(
					new HunterMessenger<HunterRobot>(robot_, this));

	if (robot_model_ == "hunter_se") {
		messenger->SetTrack(HunterV1Params::track);
		messenger->SetWeelbase(HunterV1Params::wheelbase);
		messenger->SetMaxSteerAngleCentral(HunterV1Params::max_steer_angle_central);
		messenger->SetMaxSteerAngle(HunterV1Params::max_steer_angle);
	} else if (robot_model_ == "hunter2") {
		messenger->SetTrack(HunterV2Params::track);
		messenger->SetWeelbase(HunterV2Params::wheelbase);
		messenger->SetMaxSteerAngleCentral(HunterV2Params::max_steer_angle_central);
		messenger->SetMaxSteerAngle(HunterV2Params::max_steer_angle);
	} else {
		messenger->SetTrack(HunterV1Params::track);
		messenger->SetWeelbase(HunterV1Params::wheelbase);
		messenger->SetMaxSteerAngleCentral(HunterV1Params::max_steer_angle_central);
		messenger->SetMaxSteerAngle(HunterV1Params::max_steer_angle);
	}

	messenger->SetOdometryFrame(odom_frame_);
	messenger->SetBaseFrame(base_frame_);
	messenger->SetOdometryTopicName(odom_topic_name_);
	messenger->SetPublishOdometryTF(publish_odom_tf_);
	messenger->SetStateGlitchFilterWindow(state_glitch_filter_sec_);
	messenger->SetWheelCalibration(calibration_);
	if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

	if (port_name_.find("can") != std::string::npos) {
		if (robot_->Connect(port_name_)) {
			robot_->EnableCommandedMode();
			std::cout << "Using CAN bus to talk with the robot" << std::endl;
		} else {
			std::cout << "Failed to connect to the robot CAN bus" << std::endl;
			return;
		}
	} else {
		std::cout << "Please check the specified port name is a CAN port"
							<< std::endl;
		return;
	}

	messenger->SetupSubscription();
	rclcpp::Rate rate(50);
	keep_running_ = true;
	while (keep_running_) {
		messenger->PublishStateToROS();
		rclcpp::spin_some(shared_from_this());
		rate.sleep();
	}
}
}  // namespace westonrobot
