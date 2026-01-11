//
// Created by kook on 12/29/23.
//

#include "rm2_ecat_mit/Statusword.h"

namespace rm2_ecat {
namespace mit {
bool Statusword::isOnline(CanBus bus, size_t id) const {
  size_t i = getIndex(bus, id);
  return (statusword_ & (1 << i)) > 0;
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
}  // namespace mit
}  // namespace rm2_ecat
