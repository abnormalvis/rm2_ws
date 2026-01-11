//
// Created by kook on 12/29/23.
//

#pragma once

#include <cstdint>
#include <iostream>

namespace rm2_ecat {
namespace mit {
enum class Controlword : uint32_t {
  DisableToEnable = 1,
  EnableToDisable = 2,
  NA = 0,
};

uint32_t controlwordToId(Controlword controlword);

}  // namespace mit
}  // namespace rm2_ecat
