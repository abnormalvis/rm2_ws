//
// Created by kook on 12/29/23.
//

#include "rm2_ecat_mit/Controlword.h"

namespace rm2_ecat {
namespace mit {

uint32_t controlwordToId(Controlword controlword) {
  switch (controlword) {
    case Controlword::DisableToEnable:
      return 1;
    case Controlword::EnableToDisable:
      return 2;
    default:
      return 0;
  }
}
}  // namespace mit
}  // namespace rm2_ecat
