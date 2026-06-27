#pragma once

#include <cstdint>
#include <optional>

#include "rc26_serial/protocol.hpp"

namespace rc26_telecontrol {

enum class RearPushrodButtonCommand : uint8_t {
  kExtend = 0,
  kRetract,
};

class RearPushrodButtonLogic
{
public:
  std::optional<RearPushrodButtonCommand> update(
    bool select_pressed, bool start_pressed) noexcept
  {
    if (select_pressed && start_pressed) {
      last_select_pressed_ = true;
      last_start_pressed_ = true;
      return std::nullopt;
    }

    std::optional<RearPushrodButtonCommand> event;
    if (select_pressed && !last_select_pressed_) {
      event = RearPushrodButtonCommand::kExtend;
    }
    if (start_pressed && !last_start_pressed_) {
      event = RearPushrodButtonCommand::kRetract;
    }

    last_select_pressed_ = select_pressed;
    last_start_pressed_ = start_pressed;
    return event;
  }

private:
  bool last_select_pressed_{false};
  bool last_start_pressed_{false};
};

inline uint8_t commandIdForRearPushrodCommand(RearPushrodButtonCommand command) noexcept
{
  using CommandID = rc26_serial::CommandID;
  switch (command) {
    case RearPushrodButtonCommand::kExtend:
      return static_cast<uint8_t>(CommandID::REAR_PUSHROD_EXTEND);
    case RearPushrodButtonCommand::kRetract:
    default:
      return static_cast<uint8_t>(CommandID::REAR_PUSHROD_RETRACT);
  }
}

inline const char * rearPushrodCommandName(RearPushrodButtonCommand command) noexcept
{
  switch (command) {
    case RearPushrodButtonCommand::kExtend:
      return "extend";
    case RearPushrodButtonCommand::kRetract:
    default:
      return "retract";
  }
}

}  // namespace rc26_telecontrol
