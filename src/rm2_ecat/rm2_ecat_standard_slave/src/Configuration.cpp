//
// Created by qiayuan on 23-5-14.
//

#include <algorithm>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "rm2_ecat_standard_slave/Configuration.h"

namespace rm2_ecat {
namespace standard {
bool Configuration::sanityCheck(bool silent) const {
  bool success = true;
  std::string message;
  auto check_and_inform = [&message, &success](const std::pair<bool, std::string>& test) {
    if (test.first) {
      message += "\033[32m✓\t";
      message += test.second;
      message += "\033[m\n";
      success &= true;
    } else {
      message += "\033[31m❌\t";
      message += test.second;
      message += "\033[m\n";
      success = false;
    }
  };

  //  const std::vector<std::pair<bool, std::string>> sanity_tests = {
  //      {(desiredTemperature_ > 0), "desired_temperature > 0"},
  //  };
  //
  //  std::for_each(sanity_tests.begin(), sanity_tests.end(), check_and_inform);
  //  if (!silent) {
  //    std::cout << message << std::endl;
  //  }

  return success;
}

uint8_t Configuration::getGpioModes() const {
  uint8_t gpioModes = 0;
  for (const auto& gpio : gpioConfigurations_) {
    gpioModes |= gpio.second.mode_ << gpio.first;
  }
  return gpioModes;
}
}  // namespace standard
}  // namespace rm2_ecat
