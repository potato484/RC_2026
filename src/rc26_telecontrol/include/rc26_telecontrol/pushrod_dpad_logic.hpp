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
  std::optional<PushrodCommand> update(bool back_pressed, bool start_pressed) noexcept
  {
    if (back_pressed && start_pressed) {
      last_back_pressed_ = true;
      last_start_pressed_ = true;
      return std::nullopt;
    }

    std::optional<PushrodCommand> event;
    if (back_pressed && !last_back_pressed_) {
      event = PushrodCommand::kExtend;
    }
    if (start_pressed && !last_start_pressed_) {
      event = PushrodCommand::kRetract;
    }

    last_back_pressed_ = back_pressed;
    last_start_pressed_ = start_pressed;
    return event;
  }

private:
  bool last_back_pressed_{false};
  bool last_start_pressed_{false};
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
