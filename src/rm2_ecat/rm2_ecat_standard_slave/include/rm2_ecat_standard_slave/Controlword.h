//
// Created by qiayuan on 23-5-14.
//

#pragma once

#include <cstdint>
#include <iostream>

namespace rm2_ecat {
namespace standard {
enum class Controlword : uint32_t {};

uint32_t controlwordToId(Controlword controlword);
}  // namespace standard
}  // namespace rm2_ecat
