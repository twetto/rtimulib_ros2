# rtimulib_ros2

ROS 2 Humble node for the Raspberry Pi Sense HAT IMU through RTIMULib.

Build from the Ubuntu 22.04 distrobox:

```bash
source /opt/ros/humble/setup.bash
cd ~/src
colcon build --packages-select rtimulib_ros2
```

Run:

```bash
source ~/src/install/setup.bash
ros2 launch rtimulib_ros2 rtimulib.launch.py
```

Published topics:

- `imu`: `sensor_msgs/msg/Imu`
- `imu/mag`: `sensor_msgs/msg/MagneticField`
- `imu/pressure`: `sensor_msgs/msg/FluidPressure`, disabled by default until the barometer path is validated
