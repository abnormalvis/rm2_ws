//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <cstdint>

namespace rm2_ecat {
namespace standard {
struct RxPdo {
  uint32_t controlword_;
  int16_t can0MotorCommnads_[8];
  int16_t can1MotorCommnads_[8];
  uint8_t digital_outputs_;
} __attribute__((packed));
}  // namespace standard
}  // namespace rm2_ecat
