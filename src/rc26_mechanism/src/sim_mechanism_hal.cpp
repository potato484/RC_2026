#include "rc26_mechanism/hal/sim_mechanism_hal.hpp"

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

SimMechanismHAL::SimMechanismHAL() : SimMechanismHAL(Config{}) {}

SimMechanismHAL::SimMechanismHAL(const Config& config) : config_(config), rng_(config_.random_seed) {}

bool SimMechanismHAL::open() {
    alive_->store(true, std::memory_order_release);
    open_.store(true, std::memory_order_relaxed);
    return true;
}

void SimMechanismHAL::close() {
    open_.store(false, std::memory_order_relaxed);
    alive_->store(false, std::memory_order_release);
}

bool SimMechanismHAL::isOpen() const {
    return open_.load(std::memory_order_relaxed);
}

uint8_t SimMechanismHAL::nextSeq() {
    return seq_.fetch_add(1, std::memory_order_relaxed);
}

bool SimMechanismHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload) {
    uint8_t seq = 0;
    return sendCommand(cmd_id, payload, seq);
}

bool SimMechanismHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
    (void)payload;
    if (!isOpen()) {
        return false;
    }

    out_seq = nextSeq();
    if (cmd_id == static_cast<uint8_t>(rc26_serial::CommandID::STOP)) {
        return true;
    }

    const auto done_fb = doneFeedbackForCommand(cmd_id);

    bool should_fail = false;
    if (config_.fail_rate > 0.0F) {
        std::lock_guard<std::mutex> lock(rng_mutex_);
        should_fail = fail_dist_(rng_) < config_.fail_rate;
    }

    FeedbackCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = callback_;
    }

    const auto latency = config_.action_latency;
    const auto fail_code = config_.fail_error_code;
    if (should_fail) {
        fail_count_.fetch_add(1, std::memory_order_relaxed);
    }
    auto alive = alive_;
    std::thread([cb, out_seq, cmd_id, done_fb, should_fail, latency, fail_code, alive]() {
        std::this_thread::sleep_for(latency);
        if (!alive->load(std::memory_order_acquire) || !cb) {
            return;
        }
        if (should_fail) {
            cb(out_seq, static_cast<uint8_t>(rc26_serial::FeedbackID::ACTION_FAIL), {cmd_id, fail_code});
            return;
        }
        if (done_fb.has_value()) {
            cb(out_seq, *done_fb, {});
        }
    }).detach();

    return true;
}

void SimMechanismHAL::setFeedbackCallback(FeedbackCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(cb);
}

CommHealthSnapshot SimMechanismHAL::commHealthSnapshot() const {
    CommHealthSnapshot snapshot{};
    snapshot.avg_rtt_ms = static_cast<float>(config_.action_latency.count());
    snapshot.parse_error_count = fail_count_.load(std::memory_order_relaxed);
    snapshot.comm_health_level = (snapshot.parse_error_count > 0U) ? 1U : 0U;
    return snapshot;
}

}  // namespace rc26_mechanism
