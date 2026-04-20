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
  std::optional<FrontTrackButtonCommand> update(bool y_pressed, bool a_pressed) noexcept
  {
    if (y_pressed && a_pressed) {
      const bool entering_conflict = !(last_y_pressed_ && last_a_pressed_);
      last_y_pressed_ = true;
      last_a_pressed_ = true;
      if (entering_conflict) {
        return FrontTrackButtonCommand::kConflict;
      }
      return std::nullopt;
    }

    std::optional<FrontTrackButtonCommand> event;
    if (y_pressed && !last_y_pressed_) {
      event = FrontTrackButtonCommand::kFrontTrackUp;
    }
    if (a_pressed && !last_a_pressed_) {
      event = FrontTrackButtonCommand::kFrontTrackDown;
    }

    last_y_pressed_ = y_pressed;
    last_a_pressed_ = a_pressed;
    return event;
  }

private:
  bool last_y_pressed_{false};
  bool last_a_pressed_{false};
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
