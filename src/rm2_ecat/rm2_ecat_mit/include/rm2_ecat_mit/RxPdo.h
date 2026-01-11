//
// Created by kook on 12/29/23.
//

#pragma once

#include <cstdint>

namespace rm2_ecat {
namespace mit {

struct RxPdo {
  uint32_t controlword_;
  uint64_t can0Commnads_[4];
  uint64_t can1Commnads_[4];
  uint8_t digitalOutputs_;
} __attribute__((packed));
}  // namespace mit
}  // namespace rm2_ecat
