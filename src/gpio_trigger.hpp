#pragma once

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

//  Minimal wrapper over the kernel GPIO character device, v2 ABI. The line is
//  requested once and held for the life of the object, so emitting an edge costs
//  a single ioctl. The sysfs interface would need an open/write/close per edge,
//  which is far too slow to sit inside the IMU read loop.

class GpioTrigger
{
public:
  GpioTrigger(const std::string & chip_path, unsigned int line, bool active_low)
  {
    chip_fd_ = ::open(chip_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd_ < 0) {
      throw std::runtime_error("cannot open " + chip_path + ": " + std::strerror(errno));
    }

    struct gpio_v2_line_request req;
    std::memset(&req, 0, sizeof(req));
    req.offsets[0] = line;
    req.num_lines = 1;
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    if (active_low) {
      //  Let the kernel invert the line so the caller always drives logical
      //  "active" and the wiring polarity stays a configuration detail.
      req.config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
    }
    std::snprintf(req.consumer, sizeof(req.consumer), "rtimulib_trigger");

    if (::ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
      const std::string err = std::strerror(errno);
      ::close(chip_fd_);
      chip_fd_ = -1;
      throw std::runtime_error(
              "cannot request " + chip_path + " line " + std::to_string(line) + ": " + err);
    }

    line_fd_ = req.fd;
    set(false);
  }

  ~GpioTrigger()
  {
    if (line_fd_ >= 0) {
      set(false);
      ::close(line_fd_);
    }
    if (chip_fd_ >= 0) {
      ::close(chip_fd_);
    }
  }

  GpioTrigger(const GpioTrigger &) = delete;
  GpioTrigger & operator=(const GpioTrigger &) = delete;

  bool set(bool active)
  {
    struct gpio_v2_line_values vals;
    vals.mask = 1;
    vals.bits = active ? 1 : 0;
    return ::ioctl(line_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals) >= 0;
  }

private:
  int chip_fd_{-1};
  int line_fd_{-1};
};
