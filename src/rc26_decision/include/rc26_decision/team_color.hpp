#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace rc26_decision {

struct TeamColorRuntime {
  std::string requested;
  std::string normalized;
  int mirror_sign{1};
  bool used_fallback{false};
};

inline std::string normalizeTeamToken(std::string value) {
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return value;
}

inline TeamColorRuntime resolveTeamColorRuntime(const std::string &team) {
  TeamColorRuntime runtime;
  runtime.requested = team;
  runtime.normalized = normalizeTeamToken(team);
  if (runtime.normalized == "blue") {
    runtime.mirror_sign = -1;
    return runtime;
  }
  if (runtime.normalized == "red") {
    runtime.mirror_sign = 1;
    return runtime;
  }
  runtime.normalized = "red";
  runtime.mirror_sign = 1;
  runtime.used_fallback = true;
  return runtime;
}

inline int normalizedMirrorSign(int mirror_sign) {
  return mirror_sign < 0 ? -1 : 1;
}

} // namespace rc26_decision
