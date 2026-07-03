#pragma once

#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

struct MechanismErrorDiagnostic {
  uint8_t seq{0};
  uint8_t failed_cmd{0};
  uint8_t error_code{0};
  bool busy{false};
  std::string failed_cmd_name;
  std::string error_code_name;
  std::string meaning;
  std::string recommendation;
};

inline std::string byteHexText(uint8_t value) {
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02X",
                static_cast<unsigned int>(value));
  return std::string(buf);
}

inline std::optional<MechanismErrorDiagnostic>
parseMechanismErrorDiagnostic(
    const rc26_interfaces::msg::MechanismTransportFeedback &msg) {
  if (msg.feedback_id !=
          static_cast<uint8_t>(rc26_serial::FeedbackID::MCU_ERROR) ||
      !rc26_serial::isPlanarArmErrorPayloadSize(msg.payload.size())) {
    return std::nullopt;
  }

  MechanismErrorDiagnostic diagnostic;
  diagnostic.seq = msg.seq;
  diagnostic.failed_cmd = msg.payload[0];
  diagnostic.error_code = msg.payload[1];
  diagnostic.busy = rc26_serial::isPlanarArmBusy(diagnostic.error_code);
  diagnostic.failed_cmd_name = rc26_serial::commandName(diagnostic.failed_cmd);
  diagnostic.error_code_name =
      rc26_serial::planarArmFailCodeName(diagnostic.error_code);
  diagnostic.meaning =
      rc26_serial::planarArmFailCodeMeaning(diagnostic.error_code);
  diagnostic.recommendation =
      rc26_serial::planarArmFailCodeRecommendation(diagnostic.error_code);
  return diagnostic;
}

inline std::string mechanismErrorDiagnosticText(
    const MechanismErrorDiagnostic &diagnostic) {
  return "MCU机械臂反馈0xFE：seq=" + std::to_string(diagnostic.seq) +
         " failed_cmd=" + byteHexText(diagnostic.failed_cmd) + "(" +
         diagnostic.failed_cmd_name + ") error_code=" +
         byteHexText(diagnostic.error_code) + "(" +
         diagnostic.error_code_name + ")，含义=" + diagnostic.meaning +
         "，建议=" + diagnostic.recommendation;
}

inline bool isSameSeqMechanismError(
    const rc26_interfaces::msg::MechanismTransportFeedback &msg, int seq,
    std::optional<MechanismErrorDiagnostic> &diagnostic) {
  diagnostic = parseMechanismErrorDiagnostic(msg);
  return diagnostic.has_value() && seq >= 0 &&
         msg.seq == static_cast<uint8_t>(seq & 0xFF);
}

} // namespace rc26_decision
