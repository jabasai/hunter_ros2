# ROS2 Packages for Hunter Mobile Base

## Packages

* hunter_base: a ROS wrapper around Hunter SDK to monitor and control the robot
* hunter_msgs: hunter related message definitions


## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/ugv_sdk) of "ugv_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package


1. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ```
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/agilexrobotics/ugv_sdk.git
    $ git clone https://github.com/agilexrobotics/hunter_ros2.git
    $ cd ..
    $ colcon build
    ```
    
2. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
    
* first time use hunter-ros2 package
   ```
   $ cd ~/your_ws/src/ugv_sdk/scripts/
   $ bash setup_can2usb.bash
   ```
   
* if not the first time use hunter-ros2 package(Run this command every time you turn off the power) 
   ```
   $ cd ~/catkin_ws/src/ugv_sdk/scripts/
   $ bash bringup_can2usb_500k.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```

3. Launch ROS nodes

* Start the base node for the real robot

    ```
    $ ros2 launch hunter_base hunter_base.launch.py
    ```
* Then you can send command to the robot
    ```
    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0" 

    ```

**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 

## Wheel-odometry calibration

`hunter_base` corrects the chassis' reported speed and steering angle with per-robot values estimated against RTK GNSS by the `jabas_calibration` service (jabas_deploy, `CALIBRATION.md`). Parameters on the node `hunter`:

| Parameter | Meaning | Default |
|---|---|---|
| `linear_velocity_scale` | true speed = scale × reported speed | 1.0 |
| `steering_angle_scale` | true central steering angle = scale × reported + offset | 1.0 |
| `steering_angle_offset` | steering zero offset (rad, central angle) | 0.0 |
| `odom_reference_offset_x` | distance (m) of `base_frame` ahead of the rear-axle centre | 0.0 |

With the defaults the driver behaves exactly as before. When set:
- **Odometry** integrates the true speed and steering, and the pose and twist are published for `base_frame`; the twist carries `vy = ω × offset`.
- **`/cmd_vel`** is mapped through the inverse, so the commanded speed and curvature are driven.
- **`/hunter_status`** keeps the raw chassis values.
- **Implausible values** stop the node at startup.

`hunter_base.launch.py` loads the parameters from `params_file`. Its default is `$HUNTER_BASE_PARAMS_FILE` or `/robot_configs/ros_params/hunter_base.yaml` (JABAS robot configuration), and a missing file means uncalibrated. The math lives in `include/hunter_base/wheel_calibration.hpp` and is unit-tested in `test/test_wheel_calibration.cpp` (`colcon test --packages-select hunter_base`).
