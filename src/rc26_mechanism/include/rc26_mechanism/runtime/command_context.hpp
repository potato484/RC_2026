#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>

namespace rc26_mechanism {

struct CommandResult {
    bool success{false};
    uint16_t error_code{0};
    bool timed_out{false};
    bool canceled{false};
};

struct CommandContext {
    uint8_t seq{0};
    uint8_t cmd_id{0};
    std::chrono::steady_clock::time_point start;
    std::chrono::milliseconds timeout{0};
    std::promise<CommandResult> result_promise;
    std::atomic<bool> fulfilled{false};
    std::atomic<bool> cancel_requested{false};
};

}  // namespace rc26_mechanism
