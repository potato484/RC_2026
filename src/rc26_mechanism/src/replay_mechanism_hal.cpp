#include "rc26_mechanism/hal/replay_mechanism_hal.hpp"

#include <charconv>
#include <fstream>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>

#include "rc26_serial/protocol.hpp"

namespace rc26_mechanism {

ReplayMechanismHAL::ReplayMechanismHAL() : ReplayMechanismHAL(Config{}) {}

ReplayMechanismHAL::ReplayMechanismHAL(const Config& config) : config_(config) {}

bool ReplayMechanismHAL::open() {
    alive_->store(true, std::memory_order_release);
    open_.store(true, std::memory_order_relaxed);
    if (!config_.replay_file.empty()) {
        (void)loadReplayFile();
    }
    return true;
}

void ReplayMechanismHAL::close() {
    open_.store(false, std::memory_order_relaxed);
    alive_->store(false, std::memory_order_release);
}

bool ReplayMechanismHAL::isOpen() const {
    return open_.load(std::memory_order_relaxed);
}

uint8_t ReplayMechanismHAL::nextSeq() {
    return seq_.fetch_add(1, std::memory_order_relaxed);
}

bool ReplayMechanismHAL::parseUint8Token(const std::string& token, uint8_t& out) {
    if (token.empty()) {
        return false;
    }
    unsigned int value = 0;
    const int base = (token.size() > 2 && token[0] == '0' && (token[1] == 'x' || token[1] == 'X')) ? 16 : 10;
    auto begin = token.data();
    auto end = token.data() + token.size();
    if (base == 16) {
        begin += 2;
    }
    auto [ptr, ec] = std::from_chars(begin, end, value, base);
    if (ec != std::errc{} || ptr != end || value > 0xFFU) {
        return false;
    }
    out = static_cast<uint8_t>(value);
    return true;
}

std::vector<uint8_t> ReplayMechanismHAL::parsePayloadHex(const std::string& token) {
    std::vector<uint8_t> payload;
    if (token.empty()) {
        return payload;
    }
    std::string hex = token;
    if (hex.rfind("0x", 0) == 0 || hex.rfind("0X", 0) == 0) {
        hex = hex.substr(2);
    }
    if (hex.empty() || (hex.size() % 2U != 0U)) {
        return {};
    }
    payload.reserve(hex.size() / 2U);
    for (size_t i = 0; i < hex.size(); i += 2U) {
        const auto byte_str = hex.substr(i, 2U);
        uint8_t value = 0;
        if (!parseUint8Token("0x" + byte_str, value)) {
            return {};
        }
        payload.push_back(value);
    }
    return payload;
}

bool ReplayMechanismHAL::loadReplayFile() {
    std::ifstream file(config_.replay_file);
    if (!file.is_open()) {
        parse_error_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    std::vector<ReplayEvent> loaded_events;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream ss(line);
        std::string seq_token;
        std::string fb_token;
        std::string payload_token;
        if (!std::getline(ss, seq_token, ',') || !std::getline(ss, fb_token, ',')) {
            parse_error_count_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        (void)std::getline(ss, payload_token);

        ReplayEvent event{};
        if (!parseUint8Token(seq_token, event.seq) || !parseUint8Token(fb_token, event.fb_id)) {
            parse_error_count_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        event.payload = parsePayloadHex(payload_token);
        if (!payload_token.empty() && event.payload.empty()) {
            parse_error_count_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        loaded_events.push_back(std::move(event));
    }

    std::lock_guard<std::mutex> lock(events_mutex_);
    events_ = std::move(loaded_events);
    event_index_ = 0;
    return true;
}

std::optional<uint8_t> ReplayMechanismHAL::doneFeedbackForCommand(uint8_t cmd_id) {
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

bool ReplayMechanismHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload) {
    uint8_t seq = 0;
    return sendCommand(cmd_id, payload, seq);
}

bool ReplayMechanismHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
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

    ReplayEvent replay_event{};
    bool has_replay_event = false;
    {
        std::lock_guard<std::mutex> lock(events_mutex_);
        if (event_index_ < events_.size()) {
            replay_event = events_[event_index_];
            has_replay_event = true;
            ++event_index_;
            if (config_.loop && event_index_ >= events_.size()) {
                event_index_ = 0U;
            }
        }
    }

    const auto done_fb = doneFeedbackForCommand(cmd_id);
    const auto latency = config_.action_latency;
    auto alive = alive_;
    std::thread([cb, out_seq, has_replay_event, replay_event, done_fb, latency, alive]() {
        std::this_thread::sleep_for(latency);
        if (!alive->load(std::memory_order_acquire) || !cb) {
            return;
        }

        if (has_replay_event) {
            const uint8_t event_seq = (replay_event.seq == 0xFFU) ? out_seq : replay_event.seq;
            cb(event_seq, replay_event.fb_id, replay_event.payload);
            return;
        }

        if (done_fb.has_value()) {
            cb(out_seq, *done_fb, {});
        }
    }).detach();

    return true;
}

void ReplayMechanismHAL::setFeedbackCallback(FeedbackCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(cb);
}

CommHealthSnapshot ReplayMechanismHAL::commHealthSnapshot() const {
    CommHealthSnapshot snapshot{};
    snapshot.parse_error_count = parse_error_count_.load(std::memory_order_relaxed);
    snapshot.avg_rtt_ms = static_cast<float>(config_.action_latency.count());
    snapshot.comm_health_level = (snapshot.parse_error_count > 0U) ? 1U : 0U;
    return snapshot;
}

}  // namespace rc26_mechanism
