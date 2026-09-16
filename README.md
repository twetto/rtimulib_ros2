# rtimulib_ros2

ROS 2 Humble node for the Raspberry Pi Sense HAT IMU through RTIMULib.

## Requires a patched RTIMULib

Carried as a submodule at `third_party/RTIMULib`, pinned to the
`lsm9ds1-block-reads` branch of the fork. Clone with submodules, or fetch after:

```bash
git submodule update --init --recursive
```

This is the only place RTIMULib comes from. There is deliberately no fallback to
a checkout sitting next to this package: at build time an unpinned tree looks
identical to the pinned one, so a stale or unpatched copy would be used silently.
To build against a tree elsewhere, set `RTIMULIB_SOURCE_DIR` explicitly, which is
visible in the build output.

To edit RTIMULib itself, work inside `third_party/RTIMULib`, push to the fork,
then bump the pin here with `git add third_party/RTIMULib`.

Stock upstream RTIMULib compiles and runs, but degrades silently in two ways
that nothing announces:

- Without the block-read patch, one sample costs 8.39 ms of I2C against an
  8.40 ms budget at the 119 Hz ODR, so roughly a fifth of samples are dropped
  and the node publishes about 92 Hz instead of 117.5.
- Without the correction switches, `gyro_bias_correction:=false` has no effect,
  so the gyro arrives with its bias already removed by a 101-second software
  high-pass. Any noise characterisation measured from that stream is wrong at
  long tau, and a VIO estimator is handed a stream whose bias has been partly
  taken out from under it.

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
