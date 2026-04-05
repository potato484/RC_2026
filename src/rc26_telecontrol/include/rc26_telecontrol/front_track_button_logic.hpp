#pragma once

#include <cstdint>
#include <optional>

#include "rc26_serial/protocol.hpp"

namespace rc26_telecontrol {

constexpr int k_front_track_a_button = 0;
constexpr int k_front_track_y_button = 3;

enum class FrontTrackButtonCommand : uint8_t {
  kNone = 0,
  kFrontTrackUp,
  kFrontTrackDown,
  kConflict,
};

class FrontTrackButtonLogic
{
public:
  FrontTrackButtonCommand update(bool y_pressed, bool a_pressed) const noexcept
  {
    if (y_pressed && a_pressed) {
      return FrontTrackButtonCommand::kConflict;
    }
    if (y_pressed) {
      return FrontTrackButtonCommand::kFrontTrackUp;
    }
    if (a_pressed) {
      return FrontTrackButtonCommand::kFrontTrackDown;
    }
    return FrontTrackButtonCommand::kNone;
  }
};

inline std::optional<uint8_t> commandIdForCommand(FrontTrackButtonCommand command) noexcept
{
  using CommandID = rc26_serial::CommandID;
  switch (command) {
    case FrontTrackButtonCommand::kFrontTrackUp:
      return static_cast<uint8_t>(CommandID::FRONT_TRACK_UP);
    case FrontTrackButtonCommand::kFrontTrackDown:
      return static_cast<uint8_t>(CommandID::FRONT_TRACK_DOWN);
    default:
      return std::nullopt;
  }
}

}  // namespace rc26_telecontrol
