//
// Created by kook on 12/29/23.
//

#pragma once

#include <cstdint>

namespace rm2_ecat {
namespace mit {

struct TxPdo {
  uint64_t can0Measurement_[4];
  uint64_t can1Measurement_[4];
  uint8_t digitalInputs_;
  uint32_t statusword_;
} __attribute__((packed));
}  // namespace mit
}  // namespace rm2_ecat
