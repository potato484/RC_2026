#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include "rc26_serial/protocol.hpp"

namespace rc26_telecontrol {

enum class PushrodCommand : uint8_t {
  kExtend = 0,
  kRetract,
};

class PushrodDpadLogic
{
public:
  std::optional<PushrodCommand> update(double axis_value) noexcept
  {
    const AxisZone zone = zoneForAxis(axis_value);
    if (zone == last_zone_) {
      return std::nullopt;
    }

    last_zone_ = zone;
    switch (zone) {
      case AxisZone::kLeft:
        return PushrodCommand::kExtend;
      case AxisZone::kRight:
        return PushrodCommand::kRetract;
      case AxisZone::kCenter:
      default:
        return std::nullopt;
    }
  }

private:
  enum class AxisZone : uint8_t {
    kCenter = 0,
    kLeft,
    kRight,
  };

  static AxisZone zoneForAxis(double axis_value) noexcept
  {
    if (!std::isfinite(axis_value)) {
      return AxisZone::kCenter;
    }

    const double normalized = std::clamp(axis_value, -1.0, 1.0);
    if (normalized <= -0.5) {
      return AxisZone::kLeft;
    }
    if (normalized >= 0.5) {
      return AxisZone::kRight;
    }
    return AxisZone::kCenter;
  }

  AxisZone last_zone_{AxisZone::kCenter};
};

inline uint8_t commandIdForPushrodCommand(PushrodCommand command) noexcept
{
  using CommandID = rc26_serial::CommandID;
  switch (command) {
    case PushrodCommand::kExtend:
      return static_cast<uint8_t>(CommandID::PUSHROD_EXTEND);
    case PushrodCommand::kRetract:
    default:
      return static_cast<uint8_t>(CommandID::PUSHROD_RETRACT);
  }
}

inline const char * pushrodCommandName(PushrodCommand command) noexcept
{
  switch (command) {
    case PushrodCommand::kExtend:
      return "extend";
    case PushrodCommand::kRetract:
    default:
      return "retract";
  }
}

}  // namespace rc26_telecontrol
