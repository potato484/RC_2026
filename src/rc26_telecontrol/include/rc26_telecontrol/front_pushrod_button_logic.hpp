#pragma once

#include <cstdint>
#include <optional>

#include "rc26_serial/protocol.hpp"

namespace rc26_telecontrol {

constexpr int k_front_pushrod_retract_button = 0;
constexpr int k_front_pushrod_extend_button = 3;

enum class FrontPushrodButtonCommand : uint8_t {
  kNone = 0,
  kExtend,
  kRetract,
  kConflict,
};

class FrontPushrodButtonLogic
{
public:
  std::optional<FrontPushrodButtonCommand> update(
    bool extend_pressed, bool retract_pressed) noexcept
  {
    if (extend_pressed && retract_pressed) {
      const bool entering_conflict = !(last_extend_pressed_ && last_retract_pressed_);
      last_extend_pressed_ = true;
      last_retract_pressed_ = true;
      if (entering_conflict) {
        return FrontPushrodButtonCommand::kConflict;
      }
      return std::nullopt;
    }

    std::optional<FrontPushrodButtonCommand> event;
    if (extend_pressed && !last_extend_pressed_) {
      event = FrontPushrodButtonCommand::kExtend;
    }
    if (retract_pressed && !last_retract_pressed_) {
      event = FrontPushrodButtonCommand::kRetract;
    }

    last_extend_pressed_ = extend_pressed;
    last_retract_pressed_ = retract_pressed;
    return event;
  }

private:
  bool last_extend_pressed_{false};
  bool last_retract_pressed_{false};
};

inline std::optional<uint8_t> commandIdForFrontPushrodCommand(
  FrontPushrodButtonCommand command) noexcept
{
  using CommandID = rc26_serial::CommandID;
  switch (command) {
    case FrontPushrodButtonCommand::kExtend:
      return static_cast<uint8_t>(CommandID::FRONT_PUSHROD_EXTEND);
    case FrontPushrodButtonCommand::kRetract:
      return static_cast<uint8_t>(CommandID::FRONT_PUSHROD_RETRACT);
    default:
      return std::nullopt;
  }
}

inline const char * frontPushrodCommandName(FrontPushrodButtonCommand command) noexcept
{
  switch (command) {
    case FrontPushrodButtonCommand::kExtend:
      return "extend";
    case FrontPushrodButtonCommand::kRetract:
      return "retract";
    default:
      return "unknown";
  }
}

}  // namespace rc26_telecontrol
