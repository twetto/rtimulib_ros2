#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <RTIMULib.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <rtimulib_ros2/msg/camera_trigger.hpp>

#include "gpio_trigger.hpp"

namespace
{
constexpr double G_TO_MPSS = 9.80665;
constexpr double MICROTESLA_TO_TESLA = 1.0e-6;
constexpr double HPA_TO_PA = 100.0;

sensor_msgs::msg::Imu toImuMessage(
  const RTIMU_DATA & data, const std::string & frame_id, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.orientation.x = data.fusionQPose.y();
  msg.orientation.y = data.fusionQPose.x();
  msg.orientation.z = -data.fusionQPose.z();
  msg.orientation.w = data.fusionQPose.scalar();

  msg.angular_velocity.x = data.gyro.x();
  msg.angular_velocity.y = data.gyro.y();
  msg.angular_velocity.z = data.gyro.z();

  //  All three axes are negated. Negating only x and y leaves the accelerometer
  //  in a frame that is inconsistent with the gyro, because RTIMULib's LSM9DS1
  //  driver flips gyro z but accel x and y, and the two sensors do not share
  //  axis sign conventions on this part.
  //
  //  Verified against 114 s of handheld motion by integrating the gyro over
  //  quasi-static windows and predicting where gravity should move in the body
  //  frame: the median angular error drops from 6.72 deg to 3.93 deg with z
  //  negated. Negating x and y instead scores identically - the two differ only
  //  by an overall sign - but that choice would flip the IMU frame and
  //  invalidate the existing T_cam_imu, so z is the one to negate.
  //
  //  Consequence: linear_acceleration.z reads about -9.4 at rest, not +9.4.
  msg.linear_acceleration.x = -data.accel.x() * G_TO_MPSS;
  msg.linear_acceleration.y = -data.accel.y() * G_TO_MPSS;
  msg.linear_acceleration.z = -data.accel.z() * G_TO_MPSS;

  return msg;
}

sensor_msgs::msg::MagneticField toMagMessage(
  const RTIMU_DATA & data, const std::string & frame_id, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.magnetic_field.x = data.compass.x() * MICROTESLA_TO_TESLA;
  msg.magnetic_field.y = data.compass.y() * MICROTESLA_TO_TESLA;
  msg.magnetic_field.z = data.compass.z() * MICROTESLA_TO_TESLA;
  return msg;
}

sensor_msgs::msg::FluidPressure toPressureMessage(
  const RTIMU_DATA & data, const std::string & frame_id, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::FluidPressure msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.fluid_pressure = data.pressure * HPA_TO_PA;
  return msg;
}
}  // namespace

class RTIMULibNode : public rclcpp::Node
{
public:
  RTIMULibNode()
  : Node("rtimulib_node")
  {
    const auto calibration_file_path =
      declare_parameter<std::string>("calibration_file_path", "");
    const auto calibration_file_name =
      declare_parameter<std::string>("calibration_file_name", "RTIMULib");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu");
    const auto use_compass = declare_parameter<bool>("use_compass", false);
    const auto publish_mag = declare_parameter<bool>("publish_mag", true);
    const auto publish_pressure = declare_parameter<bool>("publish_pressure", false);
    //  Default to raw. RTIMULib subtracts a continuously tracked gyro bias
    //  (time constant ~101 s at this ODR) and rescales accel by stored .ini
    //  min/max. Both are wrong for VIO, where the estimator models bias itself,
    //  and they make noise characterisation impossible: the bias tracker is a
    //  high-pass that removes the random walk an Allan variance is trying to
    //  measure. Set these true only for attitude display.
    const auto gyro_bias_correction = declare_parameter<bool>("gyro_bias_correction", false);
    const auto accel_calibration = declare_parameter<bool>("accel_calibration", false);
    const auto slerp_power = declare_parameter<double>("slerp_power", 0.02);
    const auto idle_sleep_ms = declare_parameter<int>("idle_sleep_ms", 1);

    const auto trigger_enable = declare_parameter<bool>("trigger_enable", true);
    const auto trigger_chip = declare_parameter<std::string>("trigger_chip", "/dev/gpiochip0");
    const auto trigger_line = declare_parameter<int>("trigger_line", 17);
    const auto trigger_active_low = declare_parameter<bool>("trigger_active_low", false);
    trigger_divisor_ = declare_parameter<int>("trigger_divisor", 4);
    trigger_pulse_us_ = declare_parameter<int>("trigger_pulse_us", 1000);
    trigger_frame_id_ = declare_parameter<std::string>("trigger_frame_id", "camera");

    if (calibration_file_path.empty()) {
      throw std::runtime_error("calibration_file_path must be set");
    }

    settings_ = std::make_unique<RTIMUSettings>(
      calibration_file_path.c_str(), calibration_file_name.c_str());
    imu_.reset(RTIMU::createIMU(settings_.get()));

    if (!imu_ || imu_->IMUType() == RTIMU_TYPE_NULL) {
      throw std::runtime_error("No IMU found");
    }

    if (!imu_->IMUInit()) {
      throw std::runtime_error("IMUInit failed");
    }

    imu_->setGyroBiasCorrection(gyro_bias_correction);
    imu_->setAccelCalibrationEnable(accel_calibration);
    imu_->setSlerpPower(static_cast<RTFLOAT>(slerp_power));
    imu_->setGyroEnable(true);
    imu_->setAccelEnable(true);
    imu_->setCompassEnable(use_compass);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    if (publish_mag) {
      mag_pub_ =
        create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());
    }
    if (publish_pressure) {
      pressure_.reset(RTPressure::createPressure(settings_.get()));
      if (pressure_ && pressure_->pressureInit()) {
        pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
          "imu/pressure", rclcpp::SensorDataQoS());
      } else {
        pressure_.reset();
        RCLCPP_WARN(get_logger(), "Pressure sensor requested but not available");
      }
    }

    if (trigger_divisor_ < 1) {
      throw std::runtime_error("trigger_divisor must be >= 1");
    }

    if (trigger_enable) {
      //  A missing or unreadable GPIO must not take the IMU down with it: the
      //  chardev is root-only until a udev rule grants group access, and an IMU
      //  stream without triggers is still useful. Warn and carry on instead.
      try {
        trigger_ = std::make_unique<GpioTrigger>(
          trigger_chip, static_cast<unsigned int>(trigger_line), trigger_active_low);
        trigger_pub_ = create_publisher<rtimulib_ros2::msg::CameraTrigger>(
          "camera/trigger", rclcpp::SensorDataQoS());
        RCLCPP_INFO(
          get_logger(), "Trigger on %s line %d, every %d samples, %d us pulse",
          trigger_chip.c_str(), static_cast<int>(trigger_line), trigger_divisor_,
          trigger_pulse_us_);
      } catch (const std::exception & err) {
        RCLCPP_ERROR(get_logger(), "Camera trigger disabled: %s", err.what());
        trigger_.reset();
      }
    }

    idle_sleep_ = std::chrono::milliseconds(std::max<int64_t>(0, idle_sleep_ms));

    RCLCPP_INFO(
      get_logger(), "RTIMULib node started with %s | gyro_bias_correction=%s accel_calibration=%s",
      imu_->IMUName(), gyro_bias_correction ? "ON" : "off (raw)",
      accel_calibration ? "ON" : "off (raw)");
    if (!gyro_bias_correction) {
      RCLCPP_INFO(
        get_logger(),
        "publishing RAW gyro: bias is NOT removed, model it in the estimator");
    }
    worker_ = std::thread([this]() { readLoop(); });
  }

  ~RTIMULibNode() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void readLoop()
  {
    while (rclcpp::ok() && running_) {
      if (!imu_->IMURead()) {
        std::this_thread::sleep_for(idle_sleep_);
        continue;
      }

      const auto stamp = now();
      RTIMU_DATA data = imu_->getIMUData();
      if (pressure_) {
        pressure_->pressureRead(data);
      }

      if (data.fusionQPoseValid && data.gyroValid && data.accelValid) {
        imu_pub_->publish(toImuMessage(data, frame_id_, stamp));

        //  Tie triggering to published IMU samples so the divisor means what it
        //  says. At the 119Hz ODR a divisor of 4 gives roughly 29.4Hz frames.
        if (++imu_sample_count_ % static_cast<uint64_t>(trigger_divisor_) == 0) {
          emitTrigger();
        }
      }
      if (mag_pub_ && data.compassValid) {
        mag_pub_->publish(toMagMessage(data, frame_id_, stamp));
      }
      if (pressure_pub_ && data.pressureValid) {
        pressure_pub_->publish(toPressureMessage(data, frame_id_, stamp));
      }
    }
  }

  void emitTrigger()
  {
    if (!trigger_) {
      return;
    }

    trigger_->set(true);
    //  Stamp as close to the edge as possible; this is the timebase the camera
    //  frames will be matched against.
    const auto edge = now();
    busyWaitUs(trigger_pulse_us_);
    trigger_->set(false);

    rtimulib_ros2::msg::CameraTrigger msg;
    msg.header.stamp = edge;
    msg.header.frame_id = trigger_frame_id_;
    msg.trigger_count = ++trigger_count_;
    msg.imu_sample_count = imu_sample_count_;
    msg.pulse_us = static_cast<uint32_t>(trigger_pulse_us_);
    trigger_pub_->publish(msg);
  }

  //  Busy-wait rather than sleep_for: the pulse is short enough that scheduler
  //  granularity would dominate, and pulse width is a hardware contract.
  static void busyWaitUs(int usec)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::microseconds(usec);
    while (std::chrono::steady_clock::now() < deadline) {
    }
  }

  std::unique_ptr<RTIMUSettings> settings_;
  std::unique_ptr<RTIMU> imu_;
  std::unique_ptr<RTPressure> pressure_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<rtimulib_ros2::msg::CameraTrigger>::SharedPtr trigger_pub_;
  std::unique_ptr<GpioTrigger> trigger_;
  std::string frame_id_;
  std::string trigger_frame_id_;
  int trigger_divisor_{4};
  int trigger_pulse_us_{1000};
  uint64_t imu_sample_count_{0};
  uint64_t trigger_count_{0};
  std::chrono::milliseconds idle_sleep_{1};
  std::thread worker_;
  std::atomic_bool running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RTIMULibNode>());
  } catch (const std::exception & err) {
    RCLCPP_FATAL(rclcpp::get_logger("rtimulib_node"), "%s", err.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
