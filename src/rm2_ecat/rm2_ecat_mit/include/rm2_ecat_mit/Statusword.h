//
// Created by kook on 12/29/23.
//

#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include "rm2_ecat_mit/Configuration.h"

namespace rm2_ecat {
namespace mit {

class Statusword {
 public:
  friend std::ostream& operator<<(std::ostream& os, const Statusword& statusword);

  uint32_t getRaw() const { return statusword_; }
  void setRaw(uint32_t raw) { statusword_ = raw; }

  // Motor
  bool isOnline(CanBus bus, size_t id) const;

 private:
  uint32_t statusword_{0};
};
}  // namespace mit
}  // namespace rm2_ecat
