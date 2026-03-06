#include "rc26_mechanism/hal/fault_injecting_hal.hpp"

#include <optional>
#include <thread>
#include <utility>

#include "rc26_serial/protocol.hpp"

namespace rc26_mechanism {

namespace {

std::optional<uint8_t> doneFeedbackForCommand(uint8_t cmd_id) {
    using CID = rc26_serial::CommandID;
    using FID = rc26_serial::FeedbackID;
    switch (static_cast<CID>(cmd_id)) {
    case CID::GRAB_TIP:
        return static_cast<uint8_t>(FID::GRAB_TIP_DONE);
    case CID::ASSEMBLE_WEAPON:
        return static_cast<uint8_t>(FID::ASSEMBLE_DONE);
    case CID::PLACE_KFS_GRID:
        return static_cast<uint8_t>(FID::PLACE_KFS_GRID_DONE);
    case CID::GRAB_KFS:
        return static_cast<uint8_t>(FID::GRAB_KFS_DONE);
    case CID::MECH_UP_MERLIN:
        return static_cast<uint8_t>(FID::MECH_UP_MERLIN_DONE);
    case CID::MECH_DOWN_MERLIN:
        return static_cast<uint8_t>(FID::MECH_DOWN_MERLIN_DONE);
    case CID::MECH_UP_DUEL:
        return static_cast<uint8_t>(FID::MECH_UP_DUEL_DONE);
    case CID::PLACE_KFS_GROUND:
        return static_cast<uint8_t>(FID::PLACE_KFS_GROUND_DONE);
    case CID::ROTATE_POS_90:
        return static_cast<uint8_t>(FID::ROTATE_POS_90_DONE);
    case CID::ROTATE_NEG_90:
        return static_cast<uint8_t>(FID::ROTATE_NEG_90_DONE);
    case CID::ROTATE_POS_180:
        return static_cast<uint8_t>(FID::ROTATE_POS_180_DONE);
    case CID::ROTATE_NEG_180:
        return static_cast<uint8_t>(FID::ROTATE_NEG_180_DONE);
    default:
        return std::nullopt;
    }
}

}  // namespace

FaultInjectingHAL::FaultInjectingHAL() : FaultInjectingHAL(Config{}) {}

FaultInjectingHAL::FaultInjectingHAL(const Config& config) : config_(config) {}

bool FaultInjectingHAL::open() {
    alive_->store(true, std::memory_order_release);
    open_.store(true, std::memory_order_relaxed);
    return true;
}

void FaultInjectingHAL::close() {
    open_.store(false, std::memory_order_relaxed);
    alive_->store(false, std::memory_order_release);
}

bool FaultInjectingHAL::isOpen() const {
    return open_.load(std::memory_order_relaxed);
}

uint8_t FaultInjectingHAL::nextSeq() {
    return seq_.fetch_add(1, std::memory_order_relaxed);
}

bool FaultInjectingHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload) {
    uint8_t seq = 0;
    return sendCommand(cmd_id, payload, seq);
}

bool FaultInjectingHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
    (void)payload;
    if (!isOpen()) {
        return false;
    }

    out_seq = nextSeq();
    if (cmd_id == static_cast<uint8_t>(rc26_serial::CommandID::STOP)) {
        return true;
    }

    FeedbackCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = callback_;
    }

    const auto done_fb = doneFeedbackForCommand(cmd_id);
    const auto latency = config_.action_latency;
    const auto mode = config_.mode;
    const auto fault_error_code = config_.fault_error_code;
    if (mode != "none") {
        injected_count_.fetch_add(1, std::memory_order_relaxed);
    }
    auto alive = alive_;
    std::thread([cb, out_seq, cmd_id, done_fb, latency, mode, fault_error_code, alive]() {
        std::this_thread::sleep_for(latency);
        if (!alive->load(std::memory_order_acquire) || !cb) {
            return;
        }

        if (mode == "none") {
            if (done_fb.has_value()) {
                cb(out_seq, *done_fb, {});
            }
            return;
        }

        if (mode == "action_fail_payload") {
            cb(out_seq, static_cast<uint8_t>(rc26_serial::FeedbackID::ACTION_FAIL),
               {cmd_id, fault_error_code});
            return;
        }

        if (mode == "action_fail_empty") {
            cb(out_seq, static_cast<uint8_t>(rc26_serial::FeedbackID::ACTION_FAIL), {});
            return;
        }

        if (mode == "no_feedback") {
            return;
        }

        if (mode == "out_of_order") {
            if (done_fb.has_value()) {
                cb(static_cast<uint8_t>(out_seq + 1), *done_fb, {});
            }
            return;
        }

        if (mode == "duplicate_done") {
            if (done_fb.has_value()) {
                cb(out_seq, *done_fb, {});
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                if (!alive->load(std::memory_order_acquire)) {
                    return;
                }
                cb(out_seq, *done_fb, {});
            }
            return;
        }

        if (mode == "late_success") {
            std::this_thread::sleep_for(latency * 2);
            if (!alive->load(std::memory_order_acquire)) {
                return;
            }
            if (done_fb.has_value()) {
                cb(out_seq, *done_fb, {});
            }
            return;
        }

        cb(out_seq, static_cast<uint8_t>(rc26_serial::FeedbackID::ACTION_FAIL), {cmd_id, fault_error_code});
    }).detach();

    return true;
}

void FaultInjectingHAL::setFeedbackCallback(FeedbackCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(cb);
}

CommHealthSnapshot FaultInjectingHAL::commHealthSnapshot() const {
    CommHealthSnapshot snapshot{};
    snapshot.avg_rtt_ms = static_cast<float>(config_.action_latency.count());
    snapshot.parse_error_count = injected_count_.load(std::memory_order_relaxed);
    if (config_.mode == "none") {
        snapshot.comm_health_level = 0U;
    } else if (config_.mode == "no_feedback") {
        snapshot.comm_health_level = 2U;
    } else {
        snapshot.comm_health_level = 1U;
    }
    return snapshot;
}

}  // namespace rc26_mechanism
