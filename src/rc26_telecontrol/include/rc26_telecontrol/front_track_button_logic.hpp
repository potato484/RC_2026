#pragma once

#include <cstdint>

#include "rc26_serial/protocol.hpp"

namespace rc26_telecontrol {

constexpr int k_front_track_a_button = 0;
constexpr int k_front_track_y_button = 3;
constexpr float k_front_track_timeout_sec = 8.0F;

enum class FrontTrackButtonEvent : uint8_t {
  kNone = 0,
  kFrontTrackUp,
  kFrontTrackDown,
  kConflict,
  kBusyIgnored,
};

class FrontTrackButtonLogic
{
public:
  FrontTrackButtonEvent update(bool y_pressed, bool a_pressed, bool busy) noexcept
  {
    const bool y_rising = y_pressed && !prev_y_pressed_;
    const bool a_rising = a_pressed && !prev_a_pressed_;
    prev_y_pressed_ = y_pressed;
    prev_a_pressed_ = a_pressed;

    if (y_rising && a_rising) {
      return FrontTrackButtonEvent::kConflict;
    }
    if (!y_rising && !a_rising) {
      return FrontTrackButtonEvent::kNone;
    }
    if (busy) {
      return FrontTrackButtonEvent::kBusyIgnored;
    }
    if (y_rising) {
      return FrontTrackButtonEvent::kFrontTrackUp;
    }
    return FrontTrackButtonEvent::kFrontTrackDown;
  }

private:
  bool prev_y_pressed_{false};
  bool prev_a_pressed_{false};
};

inline uint8_t commandIdForEvent(FrontTrackButtonEvent event) noexcept
{
  using CommandID = rc26_serial::CommandID;
  switch (event) {
    case FrontTrackButtonEvent::kFrontTrackUp:
      return static_cast<uint8_t>(CommandID::FRONT_TRACK_UP);
    case FrontTrackButtonEvent::kFrontTrackDown:
      return static_cast<uint8_t>(CommandID::FRONT_TRACK_DOWN);
    default:
      return 0;
  }
}

}  // namespace rc26_telecontrol
