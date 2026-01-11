//
// Created by kook on 12/29/23.
//

#include <cmath>
#include <mutex>
#include <unordered_map>

#include "rm2_ecat_mit/Reading.h"

namespace rm2_ecat {
namespace mit {
namespace {
constexpr int kEncoderRangeCounts = 1 << 16;
constexpr int kHalfRangeCounts = kEncoderRangeCounts / 2;
constexpr double kPi = 3.14159265358979323846;
constexpr uint64_t kByteMask = 0xFF;
constexpr uint64_t kNibbleMask = 0xF;

struct MultiTurnEntry {
  double continuous = 0.0;
  int64_t revolutions = 0;
  uint16_t last_code = 0;
  bool has_reference = false;
  double alignment_offset = 0.0;
};

struct SingleTurnWrap {
  double wrapped_angle = 0.0;
  double alignment_offset = 0.0;
};

std::mutex multi_turn_mutex;
std::unordered_map<std::string, MultiTurnEntry> multi_turn_entries;

inline std::string makeStateKey(CanBus bus, size_t id, const std::string& name) {
  if (!name.empty()) {
    return name;
  }
  return "bus" + std::to_string(static_cast<int>(bus)) + "_id" + std::to_string(id);
}

inline void resetMultiTurnEntry(const std::string& key) {
  if (key.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(multi_turn_mutex);
  multi_turn_entries[key] = MultiTurnEntry{};
}

inline MultiTurnEntry& getMultiTurnEntry(const std::string& key) {
  return multi_turn_entries[key];
}

inline double wrapAngleToInterval(double angle, double lower_bound, double upper_bound) {
  const double width = upper_bound - lower_bound;
  if (width <= 0.0) {
    return lower_bound;
  }

  double wrapped = std::fmod(angle - lower_bound, width);
  if (wrapped < 0.0) {
    wrapped += width;
  }
  wrapped += lower_bound;

  if (wrapped < lower_bound) {
    return lower_bound;
  }
  if (wrapped > upper_bound) {
    return upper_bound;
  }
  return wrapped;
}

inline SingleTurnWrap wrapToSingleTurn(double angle) {
  SingleTurnWrap result{angle, 0.0};
  if (!std::isfinite(angle)) {
    result.wrapped_angle = 0.0;
    result.alignment_offset = -angle;
    return result;
  }

  const double wrapped = wrapAngleToInterval(angle, -kPi, kPi);
  result.wrapped_angle = wrapped;
  result.alignment_offset = wrapped - angle;
  return result;
}

inline uint16_t extractPositionCounts(uint64_t raw_reading) {
  const auto high_byte = static_cast<uint16_t>((raw_reading >> 8) & kByteMask);
  const auto low_byte = static_cast<uint16_t>((raw_reading >> 16) & kByteMask);
  return static_cast<uint16_t>((high_byte << 8) | low_byte);
}

inline uint16_t extractVelocityCounts(uint64_t raw_reading) {
  const auto high_byte = static_cast<uint16_t>((raw_reading >> 24) & kByteMask);
  const auto low_nibble = static_cast<uint16_t>((raw_reading >> 36) & kNibbleMask);
  return static_cast<uint16_t>((high_byte << 4) | low_nibble);
}

inline uint16_t extractCurrentCounts(uint64_t raw_reading) {
  const auto high_nibble = static_cast<uint16_t>((raw_reading >> 32) & kNibbleMask);
  const auto low_byte = static_cast<uint16_t>((raw_reading >> 40) & kByteMask);
  return static_cast<uint16_t>((high_nibble << 8) | low_byte);
}
}  // namespace

Reading::Reading() {
  for (auto& velocityFilter : velocityFilters_) {
    velocityFilter = std::make_shared<LowPassFilter>(100);
  }
}

std::string Reading::getMotorName(CanBus bus, size_t id) const {
  return names_[getIndex(bus, id)];
}

std::vector<size_t> Reading::getEnabledMotorIds(CanBus bus) const {
  std::vector<size_t> enabledMotorIds;
  for (size_t id = 1; id <= motorNumEachBus; ++id) {
    if (isMotorEnabled_[getIndex(bus, id)]) {
      enabledMotorIds.push_back(id);
    }
  }
  return enabledMotorIds;
}

std::vector<uint8_t> Reading::getEnabledDigitalInputIds() const {
  std::vector<uint8_t> ids;
  for (uint8_t id = 0; id < 8; ++id) {
    if (isDigitalInputEnabled_[id]) {
      ids.push_back(id);
    }
  }
  return ids;
}

void Reading::configureReading(const Configuration& configuration) {
  for (const auto& [id, motorConfiguration] : configuration.can0MotorConfigurations_) {
    size_t index = getIndex(CanBus::CAN0, id);
    names_[index] = motorConfiguration.name_;
    isMotorEnabled_[index] = true;
    hasRawReading_[index] = false;
    positionOffset[index] = motorConfiguration.positionOffset;
    velocityOffset[index] = motorConfiguration.velocityOffset;
    torqueOffset[index] = motorConfiguration.torqueOffset;
    positionFactorIntegerToRad_[index] = motorConfiguration.positionFactorIntegerToRad_;
    velocityFactorIntegerPerMinusToRadPerSec_[index] = motorConfiguration.velocityFactorIntegerPerMinusToRadPerSec_;
    torqueFactorIntegerToNm_[index] = motorConfiguration.torqueFactorIntegerToNm_;
    resetMultiTurnEntry(makeStateKey(CanBus::CAN0, id, names_[index]));
  }
  for (const auto& [id, motorConfiguration] : configuration.can1MotorConfigurations_) {
    size_t index = getIndex(CanBus::CAN1, id);
    names_[index] = motorConfiguration.name_;
    isMotorEnabled_[index] = true;
    hasRawReading_[index] = false;
    positionOffset[index] = motorConfiguration.positionOffset;
    velocityOffset[index] = motorConfiguration.velocityOffset;
    torqueOffset[index] = motorConfiguration.torqueOffset;
    positionFactorIntegerToRad_[index] = motorConfiguration.positionFactorIntegerToRad_;
    velocityFactorIntegerPerMinusToRadPerSec_[index] = motorConfiguration.velocityFactorIntegerPerMinusToRadPerSec_;
    torqueFactorIntegerToNm_[index] = motorConfiguration.torqueFactorIntegerToNm_;
    resetMultiTurnEntry(makeStateKey(CanBus::CAN1, id, names_[index]));
  }
  for (const auto& [id, gpioConfiguration] : configuration.gpioConfigurations_) {
    if (gpioConfiguration.mode_ == 0) {
      isDigitalInputEnabled_[id] = true;
      gpioNames_[id] = gpioConfiguration.name_;
    }
  }
}

double Reading::getPosition(CanBus bus, size_t id) const {
  const size_t index = getIndex(bus, id);
  const auto state_key = makeStateKey(bus, id, names_[index]);
  std::lock_guard<std::mutex> lock(multi_turn_mutex);
  auto& state = getMultiTurnEntry(state_key);

  if (!hasRawReading_[index]) {
    return state.has_reference ? state.continuous : 0.0;
  }

  const uint16_t position_counts = extractPositionCounts(rawReadings_[index]);
  const double counts_to_rad = positionFactorIntegerToRad_[index];
  const double full_scale_radians = counts_to_rad * static_cast<double>(kEncoderRangeCounts);
  const double scaled_position = static_cast<double>(position_counts) * counts_to_rad + positionOffset[index];

  if (!state.has_reference) {
    const auto wrap = wrapToSingleTurn(scaled_position);
    state.alignment_offset = wrap.alignment_offset;
    state.continuous = wrap.wrapped_angle;
    state.revolutions = 0;
    state.last_code = position_counts;
    state.has_reference = true;
    return state.continuous;
  }

  const int32_t delta = static_cast<int32_t>(position_counts) - static_cast<int32_t>(state.last_code);
  if (delta > kHalfRangeCounts) {
    --state.revolutions;
  } else if (delta < -kHalfRangeCounts) {
    ++state.revolutions;
  }

  state.last_code = position_counts;
  state.continuous = scaled_position + state.alignment_offset + static_cast<double>(state.revolutions) * full_scale_radians;
  return state.continuous;
}

double Reading::getVelocity(CanBus bus, size_t id) const {
  const size_t index = getIndex(bus, id);
  const uint16_t velocity_counts = extractVelocityCounts(rawReadings_[index]);

  //  double time = getTime(stamp_);

  //  velocityFilters_[i]->input(velocity, time);
  //  velocity = velocityFilters_[i]->output();
  return static_cast<double>(velocity_counts * velocityFactorIntegerPerMinusToRadPerSec_[index] + velocityOffset[index]);
}

double Reading::getTorque(CanBus bus, size_t id) const {
  const size_t index = getIndex(bus, id);
  const uint16_t current_counts = extractCurrentCounts(rawReadings_[index]);
  return static_cast<double>(current_counts * torqueFactorIntegerToNm_[index] + torqueOffset[index]);
}

uint16_t Reading::getRawReading(CanBus bus, size_t id) const {
  return rawReadings_[getIndex(bus, id)];
}

bool Reading::getDigitalInput(uint8_t id) const {
  return digitalInputs_[id];
}

void Reading::setRawReading(CanBus bus, size_t id, uint64_t data) {
  const size_t index = getIndex(bus, id);
  rawReadings_[index] = data;
  hasRawReading_[index] = true;
}

void Reading::setDigitalInputs(uint8_t value) {
  for (size_t i = 0; i < 8; ++i) {
    digitalInputs_[i] = ((value & (1 << i)) != 0);
  }
}

std::string Reading::getGpioName(uint8_t id) const {
  return gpioNames_[id];
}

}  // namespace mit
}  // namespace rm2_ecat
