//
// Created by qiayuan on 23-5-14.
//

#include "rm2_ecat_standard_slave/Statusword.h"

namespace rm2_ecat {
namespace standard {
bool Statusword::isOnline(CanBus bus, size_t id) const {
  size_t i = getIndex(bus, id);
  return (statusword_ & (1 << i)) > 0;
}

bool Statusword::isAngularVelocityUpdated(CanBus bus) const {
  return (statusword_ & (1 << (4 * static_cast<uint8_t>(bus) + 16))) > 0;
}

bool Statusword::isLinearAccelerationUpdated(CanBus bus) const {
  return (statusword_ & (1 << (4 * static_cast<uint8_t>(bus) + 16 + 1))) > 0;
}

bool Statusword::isTriggered(CanBus bus) const {
  return (statusword_ & (1 << (4 * static_cast<uint8_t>(bus) + 16 + 2))) > 0;
}

bool Statusword::isTriggerEnabled(CanBus bus) const {
  return (statusword_ & (1 << (4 * static_cast<uint8_t>(bus) + 16 + 3))) > 0;
}

std::ostream& operator<<(std::ostream& os, const Statusword& statusword) {
  using std::setfill;
  using std::setw;
  int gapSize2 = 6;
  os << std::left << std::boolalpha << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(gapSize2 + 27) << setfill(' ') << "| Statusword"
     << "|\n"
     << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(25) << setfill(' ') << "| Name of Bit" << setw(gapSize2 + 2) << "| Value"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') << std::noboolalpha << std::right << setfill(' ');

  return os;
}
}  // namespace standard
}  // namespace rm2_ecat
