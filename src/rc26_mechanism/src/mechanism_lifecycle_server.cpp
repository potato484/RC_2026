#include "rc26_mechanism/mechanism_lifecycle_server.hpp"

#include <cmath>
#include <functional>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

#include "rc26_mechanism/hal/serial_mechanism_hal.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_mechanism {

namespace {

std::chrono::milliseconds timeoutFromGoal(float timeout_sec, std::chrono::milliseconds fallback) {
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0F) {
        return fallback;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(static_cast<double>(timeout_sec)));
}

}  // namespace

MechanismLifecycleServer::MechanismLifecycleServer(const rclcpp::NodeOptions& opts)
    : rclcpp_lifecycle::LifecycleNode("mechanism_server", opts) {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    this->declare_parameter<int>("serial_baud", 1000000);
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_configure(const rclcpp_lifecycle::State&) {
    const auto port = this->get_parameter("serial_port").as_string();
    const auto baud = this->get_parameter("serial_baud").as_int();

    hal_ = std::make_unique<SerialMechanismHAL>(port, baud);
    if (!hal_->open()) {
        RCLCPP_ERROR(this->get_logger(), "failed to open mechanism serial: %s", port.c_str());
        return CallbackReturn::FAILURE;
    }

    hal_->setFeedbackCallback(
        [this](uint8_t fb_id, const std::vector<uint8_t>& payload) {
            onSerialFeedback(fb_id, payload);
        });

    state_pub_ = this->create_publisher<rc26_interfaces::msg::MechanismState>(
        "/mechanism/state", rclcpp::QoS(1).reliable());

    grab_tip_srv_ = rclcpp_action::create_server<GrabTip>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/mechanism/grab_tip",
        std::bind(&MechanismLifecycleServer::handleGrabGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MechanismLifecycleServer::handleGrabCancel, this, std::placeholders::_1),
        std::bind(&MechanismLifecycleServer::handleGrabAccepted, this, std::placeholders::_1));

    assemble_srv_ = rclcpp_action::create_server<AssembleWeapon>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/mechanism/assemble_weapon",
        std::bind(&MechanismLifecycleServer::handleAssembleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MechanismLifecycleServer::handleAssembleCancel, this, std::placeholders::_1),
        std::bind(&MechanismLifecycleServer::handleAssembleAccepted, this, std::placeholders::_1));

    place_grid_srv_ = rclcpp_action::create_server<PlaceKFSGrid>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/mechanism/place_kfs_grid",
        std::bind(&MechanismLifecycleServer::handlePlaceGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MechanismLifecycleServer::handlePlaceCancel, this, std::placeholders::_1),
        std::bind(&MechanismLifecycleServer::handlePlaceAccepted, this, std::placeholders::_1));

    execute_srv_ = rclcpp_action::create_server<ExecuteMechanism>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/mechanism/execute",
        std::bind(&MechanismLifecycleServer::handleExecuteGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MechanismLifecycleServer::handleExecuteCancel, this, std::placeholders::_1),
        std::bind(&MechanismLifecycleServer::handleExecuteAccepted, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_activate(const rclcpp_lifecycle::State&) {
    if (state_pub_) {
        state_pub_->on_activate();
    }

    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MechanismLifecycleServer::publishMechanismState, this));
    publishMechanismState();
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_deactivate(const rclcpp_lifecycle::State&) {
    if (state_timer_) {
        state_timer_->cancel();
        state_timer_.reset();
    }
    if (state_pub_) {
        state_pub_->on_deactivate();
    }
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_cleanup(const rclcpp_lifecycle::State&) {
    if (state_timer_) {
        state_timer_->cancel();
        state_timer_.reset();
    }
    if (state_pub_) {
        state_pub_.reset();
    }
    grab_tip_srv_.reset();
    assemble_srv_.reset();
    place_grid_srv_.reset();
    execute_srv_.reset();

    if (hal_) {
        hal_->close();
        hal_.reset();
    }
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_error(const rclcpp_lifecycle::State&) {
    if (hal_) {
        hal_->close();
    }
    return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse MechanismLifecycleServer::handleGrabGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GrabTip::Goal> goal) {
    (void)uuid;
    if (!hal_ || !hal_->isOpen() || !goal || goal->tip_index > 5) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handleGrabCancel(
    const std::shared_ptr<GoalHandleGrabTip>& /*goal_handle*/) {
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handleGrabAccepted(const std::shared_ptr<GoalHandleGrabTip>& goal_handle) {
    std::thread([this, goal_handle]() { executeGrab(goal_handle); }).detach();
}

void MechanismLifecycleServer::executeGrab(const std::shared_ptr<GoalHandleGrabTip>& goal_handle) {
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<GrabTip::Result>();
    result->success = false;
    result->error_code = 0;

    const auto goal = goal_handle->get_goal();
    if (!hal_ || !hal_->isOpen() || !goal) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        if (!tip_sm_.transition(TipState::APPROACHING)) {
            result->error_code = 2;
            goal_handle->abort(result);
            return;
        }
        tip_sm_.lockedTipSlot = goal->tip_index;
    }

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        resetCommandFlags(static_cast<uint8_t>(rc26_serial::CommandID::GRAB_TIP));
        last_error_code_ = 0;
    }

    if (!hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::GRAB_TIP),
                           {goal->tip_index})) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::IDLE);
        tip_sm_.lockedTipSlot.reset();
        result->error_code = 3;
        goal_handle->abort(result);
        return;
    }

    const bool signaled = waitUntilDoneOrFailed(grab_done_, grab_failed_, std::chrono::seconds(8));
    if (goal_handle->is_canceling()) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::IDLE);
        tip_sm_.lockedTipSlot.reset();
        goal_handle->canceled(result);
        return;
    }
    if (!signaled || grab_failed_) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::IDLE);
        tip_sm_.lockedTipSlot.reset();
        result->error_code = last_error_code_;
        goal_handle->abort(result);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        if (!tip_sm_.transition(TipState::HOLDING)) {
            (void)tip_sm_.transition(TipState::IDLE);
            tip_sm_.lockedTipSlot.reset();
            result->error_code = 4;
            goal_handle->abort(result);
            return;
        }
    }

    result->success = true;
    goal_handle->succeed(result);
    publishMechanismState();
}

rclcpp_action::GoalResponse MechanismLifecycleServer::handleAssembleGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const AssembleWeapon::Goal> goal) {
    (void)uuid;
    (void)goal;
    if (!hal_ || !hal_->isOpen()) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handleAssembleCancel(
    const std::shared_ptr<GoalHandleAssemble>& /*goal_handle*/) {
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handleAssembleAccepted(const std::shared_ptr<GoalHandleAssemble>& goal_handle) {
    std::thread([this, goal_handle]() { executeAssemble(goal_handle); }).detach();
}

void MechanismLifecycleServer::executeAssemble(const std::shared_ptr<GoalHandleAssemble>& goal_handle) {
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<AssembleWeapon::Result>();
    result->success = false;
    result->error_code = 0;

    if (!hal_ || !hal_->isOpen()) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        if (!tip_sm_.isHoldingTip() || !tip_sm_.transition(TipState::ASSEMBLING)) {
            result->error_code = 2;
            goal_handle->abort(result);
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        resetCommandFlags(static_cast<uint8_t>(rc26_serial::CommandID::ASSEMBLE_WEAPON));
        last_error_code_ = 0;
    }

    if (!hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::ASSEMBLE_WEAPON))) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::HOLDING);
        result->error_code = 3;
        goal_handle->abort(result);
        return;
    }

    const bool signaled = waitUntilDoneOrFailed(assemble_done_, assemble_failed_, std::chrono::seconds(30));
    if (goal_handle->is_canceling()) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::HOLDING);
        goal_handle->canceled(result);
        return;
    }
    if (!signaled || assemble_failed_) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::HOLDING);
        result->error_code = last_error_code_;
        goal_handle->abort(result);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::ASSEMBLED);
        (void)tip_sm_.transition(TipState::IDLE);
        tip_sm_.lockedTipSlot.reset();
        if (assembled_count_ < 255) {
            ++assembled_count_;
        }
    }

    result->success = true;
    goal_handle->succeed(result);
    publishMechanismState();
}

rclcpp_action::GoalResponse MechanismLifecycleServer::handlePlaceGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PlaceKFSGrid::Goal> goal) {
    (void)uuid;
    if (!hal_ || !hal_->isOpen() || !goal || goal->layer == 0) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handlePlaceCancel(
    const std::shared_ptr<GoalHandlePlace>& /*goal_handle*/) {
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handlePlaceAccepted(const std::shared_ptr<GoalHandlePlace>& goal_handle) {
    std::thread([this, goal_handle]() { executePlace(goal_handle); }).detach();
}

void MechanismLifecycleServer::executePlace(const std::shared_ptr<GoalHandlePlace>& goal_handle) {
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<PlaceKFSGrid::Result>();
    result->success = false;
    result->error_code = 0;

    const auto goal = goal_handle->get_goal();
    if (!hal_ || !hal_->isOpen() || !goal || goal->layer == 0) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        resetCommandFlags(static_cast<uint8_t>(rc26_serial::CommandID::PLACE_KFS_GRID));
        last_error_code_ = 0;
    }

    if (!hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::PLACE_KFS_GRID),
                           {goal->grid_position, goal->layer})) {
        result->error_code = 2;
        goal_handle->abort(result);
        return;
    }

    const bool signaled = waitUntilDoneOrFailed(place_done_, place_failed_, std::chrono::seconds(8));
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
    }
    if (!signaled || place_failed_) {
        result->error_code = last_error_code_;
        goal_handle->abort(result);
        return;
    }

    result->success = true;
    goal_handle->succeed(result);
    publishMechanismState();
}

rclcpp_action::GoalResponse MechanismLifecycleServer::handleExecuteGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExecuteMechanism::Goal> goal) {
    (void)uuid;
    if (!hal_ || !hal_->isOpen() || !goal || !isCommandSupported(goal->command_id)) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handleExecuteCancel(
    const std::shared_ptr<GoalHandleExecute>& /*goal_handle*/) {
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handleExecuteAccepted(const std::shared_ptr<GoalHandleExecute>& goal_handle) {
    std::thread([this, goal_handle]() { executeCommand(goal_handle); }).detach();
}

void MechanismLifecycleServer::executeCommand(const std::shared_ptr<GoalHandleExecute>& goal_handle) {
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<ExecuteMechanism::Result>();
    result->success = false;
    result->error_code = 0;

    const auto goal = goal_handle->get_goal();
    if (!hal_ || !hal_->isOpen() || !goal || !isCommandSupported(goal->command_id)) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    bool* done = nullptr;
    bool* failed = nullptr;
    if (!commandFlagsFor(goal->command_id, done, failed) || !done || !failed) {
        result->error_code = 2;
        goal_handle->abort(result);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        resetCommandFlags(goal->command_id);
        last_error_code_ = 0;
    }

    if (!hal_->sendCommand(goal->command_id, goal->payload)) {
        result->error_code = 3;
        goal_handle->abort(result);
        return;
    }

    const auto timeout = timeoutFromGoal(goal->timeout_sec, defaultTimeoutForCommand(goal->command_id));
    const bool signaled = waitUntilDoneOrFailed(*done, *failed, timeout);
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
    }
    if (!signaled || *failed) {
        result->error_code = last_error_code_;
        goal_handle->abort(result);
        return;
    }

    result->success = true;
    goal_handle->succeed(result);
    publishMechanismState();
}

bool MechanismLifecycleServer::isCommandSupported(uint8_t cmd_id) const {
    using CID = rc26_serial::CommandID;
    switch (static_cast<CID>(cmd_id)) {
    case CID::GRAB_KFS:
    case CID::MECH_UP_MERLIN:
    case CID::MECH_DOWN_MERLIN:
    case CID::MECH_UP_DUEL:
    case CID::PLACE_KFS_GROUND:
    case CID::ROTATE_POS_90:
    case CID::ROTATE_NEG_90:
    case CID::ROTATE_POS_180:
    case CID::ROTATE_NEG_180:
        return true;
    default:
        return false;
    }
}

std::chrono::milliseconds MechanismLifecycleServer::defaultTimeoutForCommand(uint8_t cmd_id) const {
    using CID = rc26_serial::CommandID;
    switch (static_cast<CID>(cmd_id)) {
    case CID::GRAB_KFS:
    case CID::MECH_UP_MERLIN:
    case CID::MECH_DOWN_MERLIN:
    case CID::MECH_UP_DUEL:
    case CID::PLACE_KFS_GROUND:
    case CID::ROTATE_POS_90:
    case CID::ROTATE_NEG_90:
    case CID::ROTATE_POS_180:
    case CID::ROTATE_NEG_180:
        return std::chrono::seconds(8);
    default:
        return std::chrono::seconds(8);
    }
}

bool MechanismLifecycleServer::commandFlagsFor(uint8_t cmd_id, bool*& done, bool*& failed) {
    using CID = rc26_serial::CommandID;
    switch (static_cast<CID>(cmd_id)) {
    case CID::GRAB_TIP:
        done = &grab_done_;
        failed = &grab_failed_;
        return true;
    case CID::ASSEMBLE_WEAPON:
        done = &assemble_done_;
        failed = &assemble_failed_;
        return true;
    case CID::PLACE_KFS_GRID:
        done = &place_done_;
        failed = &place_failed_;
        return true;
    case CID::GRAB_KFS:
        done = &grab_kfs_done_;
        failed = &grab_kfs_failed_;
        return true;
    case CID::MECH_UP_MERLIN:
        done = &mech_up_merlin_done_;
        failed = &mech_up_merlin_failed_;
        return true;
    case CID::MECH_DOWN_MERLIN:
        done = &mech_down_merlin_done_;
        failed = &mech_down_merlin_failed_;
        return true;
    case CID::MECH_UP_DUEL:
        done = &mech_up_duel_done_;
        failed = &mech_up_duel_failed_;
        return true;
    case CID::PLACE_KFS_GROUND:
        done = &place_ground_done_;
        failed = &place_ground_failed_;
        return true;
    case CID::ROTATE_POS_90:
    case CID::ROTATE_NEG_90:
    case CID::ROTATE_POS_180:
    case CID::ROTATE_NEG_180:
        done = &rotate_done_;
        failed = &rotate_failed_;
        return true;
    default:
        done = nullptr;
        failed = nullptr;
        return false;
    }
}

void MechanismLifecycleServer::resetCommandFlags(uint8_t cmd_id) {
    bool* done = nullptr;
    bool* failed = nullptr;
    if (commandFlagsFor(cmd_id, done, failed) && done && failed) {
        *done = false;
        *failed = false;
    }
}

void MechanismLifecycleServer::onSerialFeedback(uint8_t fb_id, const std::vector<uint8_t>& payload) {
    using FID = rc26_serial::FeedbackID;

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    switch (static_cast<FID>(fb_id)) {
    case FID::GRAB_TIP_DONE:
        grab_done_ = true;
        break;
    case FID::ASSEMBLE_DONE:
        assemble_done_ = true;
        break;
    case FID::PLACE_KFS_GRID_DONE:
        place_done_ = true;
        break;
    case FID::GRAB_KFS_DONE:
        grab_kfs_done_ = true;
        break;
    case FID::MECH_UP_MERLIN_DONE:
        mech_up_merlin_done_ = true;
        break;
    case FID::MECH_DOWN_MERLIN_DONE:
        mech_down_merlin_done_ = true;
        break;
    case FID::MECH_UP_DUEL_DONE:
        mech_up_duel_done_ = true;
        break;
    case FID::PLACE_KFS_GROUND_DONE:
        place_ground_done_ = true;
        break;
    case FID::ROTATE_POS_90_DONE:
    case FID::ROTATE_NEG_90_DONE:
    case FID::ROTATE_POS_180_DONE:
    case FID::ROTATE_NEG_180_DONE:
        rotate_done_ = true;
        break;
    case FID::ACTION_FAIL:
        if (!payload.empty()) {
            bool* done = nullptr;
            bool* failed = nullptr;
            if (commandFlagsFor(payload[0], done, failed) && failed) {
                *failed = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "ACTION_FAIL unknown cmd_id=0x%02X", payload[0]);
            }
            last_error_code_ = (payload.size() >= 2) ? payload[1] : 0;
        } else {
            RCLCPP_WARN(this->get_logger(), "ACTION_FAIL payload empty, mark all as failed");
            grab_failed_ = true;
            assemble_failed_ = true;
            place_failed_ = true;
            grab_kfs_failed_ = true;
            mech_up_merlin_failed_ = true;
            mech_down_merlin_failed_ = true;
            mech_up_duel_failed_ = true;
            place_ground_failed_ = true;
            rotate_failed_ = true;
            last_error_code_ = 0;
        }
        break;
    default:
        break;
    }
    feedback_cv_.notify_all();
}

void MechanismLifecycleServer::publishMechanismState() {
    if (!state_pub_ || !state_pub_->is_activated()) {
        return;
    }

    rc26_interfaces::msg::MechanismState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.tip_state = static_cast<uint8_t>(tip_sm_.current());
    msg.hal_open = hal_ && hal_->isOpen();
    msg.locked_tip_slot = tip_sm_.lockedTipSlot.value_or(255);
    msg.assembled_count = assembled_count_;
    msg.last_error_code = last_error_code_;
    state_pub_->publish(msg);
}

bool MechanismLifecycleServer::waitUntilDoneOrFailed(
    bool& done, bool& failed, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(feedback_mutex_);
    return feedback_cv_.wait_for(lock, timeout, [&done, &failed]() { return done || failed; });
}

}  // namespace rc26_mechanism

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_mechanism::MechanismLifecycleServer)
