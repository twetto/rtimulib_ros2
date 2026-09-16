from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # trigger_divisor sets the camera rate: IMU runs at ~117.5 Hz, so 4 gives
    # ~29.4 Hz and 2 gives ~58.8 Hz. Note the frame-to-trigger lag is ~10 ms,
    # which exceeds half a period once the divisor drops to 2; matching is
    # lag-compensated so that is handled, but watch residual>half-period.
    args = [
        DeclareLaunchArgument("trigger_divisor", default_value="4"),
        DeclareLaunchArgument("trigger_pulse_us", default_value="1000"),
        DeclareLaunchArgument("trigger_enable", default_value="true"),
    ]

    return LaunchDescription(args + [
        Node(
            package="rtimulib_ros2",
            executable="rtimulib_node",
            name="rtimulib_node",
            output="screen",
            parameters=[{
                "calibration_file_path": PathJoinSubstitution([
                    FindPackageShare("rtimulib_ros2"),
                    "config",
                ]),
                "calibration_file_name": "RTIMULib",
                "frame_id": "imu",
                "use_compass": False,
                "publish_mag": True,
                "publish_pressure": False,
                # Camera hardware trigger. BCM GPIO 17 is physical pin 11, the
                # line the old ROS 1 node drove through WiringPi pin 0.
                "trigger_enable": ParameterValue(
                    LaunchConfiguration("trigger_enable"), value_type=bool),
                "trigger_chip": "/dev/gpiochip0",
                "trigger_line": 17,
                "trigger_divisor": ParameterValue(
                    LaunchConfiguration("trigger_divisor"), value_type=int),
                "trigger_pulse_us": ParameterValue(
                    LaunchConfiguration("trigger_pulse_us"), value_type=int),
                "trigger_active_low": False,
                "trigger_frame_id": "camera",
                # Per-axis accel scale from scripts/solve_accel_calib.py, six
                # static poses, 2026-09-16. Bias is deliberately left at zero:
                # the VIO estimator models it online, and the measured values
                # (-0.034, -0.135, +0.289) would go stale with temperature.
                # Consequence: |a| at rest still reads about 9.52, not 9.807,
                # because the bias is still present. That is intended.
                "accel_scale": [0.995780, 0.991649, 0.993818],
                "accel_bias": [0.0, 0.0, 0.0],
            }],
        ),
    ])
