#pragma once

#include <cstdint>
#include <optional>

namespace rc26_mechanism {

enum class TipState : uint8_t {
    IDLE = 0,
    APPROACHING = 1,
    HOLDING = 2,
    ASSEMBLING = 3,
    ASSEMBLED = 4
};

class TipStateMachine {
public:
    TipState current() const { return state_; }

    bool canTransition(TipState next) const {
        switch (state_) {
        case TipState::IDLE:
            return next == TipState::APPROACHING;
        case TipState::APPROACHING:
            return next == TipState::HOLDING || next == TipState::IDLE;
        case TipState::HOLDING:
            return next == TipState::ASSEMBLING || next == TipState::IDLE;
        case TipState::ASSEMBLING:
            return next == TipState::ASSEMBLED || next == TipState::HOLDING;
        case TipState::ASSEMBLED:
            return next == TipState::IDLE;
        default:
            return false;
        }
    }

    bool transition(TipState next) {
        if (!canTransition(next)) {
            return false;
        }
        state_ = next;
        return true;
    }

    bool isHoldingTip() const {
        return state_ == TipState::HOLDING || state_ == TipState::ASSEMBLING;
    }

    std::optional<uint8_t> lockedTipSlot;

private:
    TipState state_{TipState::IDLE};
};

}  // namespace rc26_mechanism
