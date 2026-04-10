#include "launch_mode.hpp"

#include <string>
#include <vector>

namespace rviz2
{
namespace
{

std::string consumeOption(
  const std::vector<std::string> & input_args,
  std::vector<std::string> & filtered_args,
  const std::string & option_name,
  const std::string & fallback_value)
{
  std::string value = fallback_value;
  for (size_t index = 1; index < input_args.size(); ++index) {
    const std::string & arg = input_args[index];
    if (arg == option_name) {
      if (index + 1 < input_args.size()) {
        value = input_args[index + 1];
        ++index;
      }
      continue;
    }
    if (arg.rfind(option_name + "=", 0) == 0) {
      value = arg.substr(option_name.size() + 1);
      continue;
    }
    filtered_args.push_back(arg);
  }
  return value;
}

bool consumeFlag(
  const std::vector<std::string> & input_args,
  std::vector<std::string> & filtered_args,
  const std::string & option_name)
{
  bool enabled = false;
  for (size_t index = 1; index < input_args.size(); ++index) {
    const std::string & arg = input_args[index];
    if (arg == option_name) {
      enabled = true;
      continue;
    }
    filtered_args.push_back(arg);
  }
  return enabled;
}

}  // namespace

bool hasDisplayConfig(const std::vector<std::string> & args)
{
  for (size_t index = 1; index < args.size(); ++index) {
    const std::string & arg = args[index];
    if (arg == "-d" || arg == "--display-config" || arg.rfind("--display-config=", 0) == 0) {
      return true;
    }
  }
  return false;
}

LaunchMode parseLaunchMode(int argc, char ** argv)
{
  LaunchMode mode;
  std::vector<std::string> original_args;
  original_args.reserve(static_cast<size_t>(argc));
  for (int index = 0; index < argc; ++index) {
    original_args.emplace_back(argv[index]);
  }

  std::vector<std::string> without_classic;
  without_classic.push_back(original_args.front());
  mode.classic = consumeFlag(original_args, without_classic, "--classic");

  std::vector<std::string> without_layout;
  without_layout.push_back(without_classic.front());
  mode.rc26_layout = consumeOption(
    without_classic, without_layout, "--rc26-layout", "operator");

  std::vector<std::string> without_legacy_layout;
  without_legacy_layout.push_back(without_layout.front());
  mode.rc26_layout = consumeOption(
    without_layout, without_legacy_layout, "--layout", mode.rc26_layout);

  mode.filtered_args.push_back(without_legacy_layout.front());
  mode.rc26_mode = consumeOption(
    without_legacy_layout, mode.filtered_args, "--rc26-mode", "navigation");

  std::vector<std::string> without_legacy_mode;
  without_legacy_mode.push_back(mode.filtered_args.front());
  mode.rc26_mode = consumeOption(
    mode.filtered_args, without_legacy_mode, "--mode", mode.rc26_mode);
  mode.filtered_args = std::move(without_legacy_mode);

  return mode;
}

}  // namespace rviz2
