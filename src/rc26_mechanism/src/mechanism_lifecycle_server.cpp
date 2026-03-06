#include "rc26_mechanism/mechanism_lifecycle_server.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

#include "rc26_mechanism/hal/fault_injecting_hal.hpp"
#include "rc26_mechanism/hal/replay_mechanism_hal.hpp"
#include "rc26_mechanism/hal/serial_mechanism_hal.hpp"
#include "rc26_mechanism/hal/sim_mechanism_hal.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_mechanism {

namespace {

template <typename F>
class ScopeExit {
public:
    explicit ScopeExit(F&& fn) : fn_(std::forward<F>(fn)) {}
    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;
    ScopeExit(ScopeExit&& other) noexcept : fn_(std::move(other.fn_)), active_(other.active_) {
        other.active_ = false;
    }
    ~ScopeExit() {
        if (active_) {
            fn_();
        }
    }

private:
    F fn_;
    bool active_{true};
};

constexpr auto kBufferedFeedbackTtl = std::chrono::seconds(2);

std::chrono::milliseconds timeoutFromGoal(float timeout_sec, std::chrono::milliseconds fallback) {
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0F) {
        return fallback;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(static_cast<double>(timeout_sec)));
}

bool isFatalErrorCode(uint16_t error_code) {
    return error_code == 0xFE || error_code == 0xFF;
}

bool isTerminalOrFailureFeedback(uint8_t fb_id) {
    using FID = rc26_serial::FeedbackID;
    switch (static_cast<FID>(fb_id)) {
    case FID::GRAB_TIP_DONE:
    case FID::ASSEMBLE_DONE:
    case FID::PLACE_KFS_GRID_DONE:
    case FID::GRAB_KFS_DONE:
    case FID::MECH_UP_MERLIN_DONE:
    case FID::MECH_DOWN_MERLIN_DONE:
    case FID::MECH_UP_DUEL_DONE:
    case FID::PLACE_KFS_GROUND_DONE:
    case FID::ROTATE_POS_90_DONE:
    case FID::ROTATE_NEG_90_DONE:
    case FID::ROTATE_POS_180_DONE:
    case FID::ROTATE_NEG_180_DONE:
    case FID::ACTION_FAIL:
    case FID::ERROR:
        return true;
    default:
        return false;
    }
}

}  // namespace

MechanismLifecycleServer::MechanismLifecycleServer(const rclcpp::NodeOptions& opts)
    : rclcpp_lifecycle::LifecycleNode("mechanism_server", opts),
      cmd_start_time_(std::chrono::steady_clock::now()) {
    this->declare_parameter<std::string>("hal_type", "serial");
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    this->declare_parameter<int>("serial_baud", 1000000);
    this->declare_parameter<int>("sim_action_latency_ms", 500);
    this->declare_parameter<double>("sim_fail_rate", 0.0);
    this->declare_parameter<int>("sim_fail_error_code", 1);
    this->declare_parameter<std::string>("fault_mode", "action_fail_payload");
    this->declare_parameter<int>("fault_action_latency_ms", 200);
    this->declare_parameter<int>("fault_error_code", 1);
    this->declare_parameter<std::string>("replay_file", "");
    this->declare_parameter<int>("replay_action_latency_ms", 100);
    this->declare_parameter<bool>("replay_loop", false);
}

MechanismLifecycleServer::~MechanismLifecycleServer() {
    active_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
        drainPendingContexts();
        buffered_feedbacks_.clear();
    }
    feedback_cv_.notify_all();
    if (hal_) {
        hal_->close();
    }
    joinExecutionThread();
    execution_in_progress_.store(false, std::memory_order_release);
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_configure(const rclcpp_lifecycle::State&) {
    const auto hal_type = this->get_parameter("hal_type").as_string();
    const auto port = this->get_parameter("serial_port").as_string();
    const auto baud = this->get_parameter("serial_baud").as_int();

    if (hal_type == "serial") {
        hal_ = std::make_unique<SerialMechanismHAL>(port, baud);
        RCLCPP_INFO(this->get_logger(), "configured mechanism HAL: serial (%s @ %ld)", port.c_str(),
                    static_cast<long>(baud));
    } else if (hal_type == "sim") {
        SimMechanismHAL::Config cfg{};
        cfg.action_latency = std::chrono::milliseconds(this->get_parameter("sim_action_latency_ms").as_int());
        cfg.fail_rate = static_cast<float>(this->get_parameter("sim_fail_rate").as_double());
        cfg.fail_error_code = static_cast<uint8_t>(this->get_parameter("sim_fail_error_code").as_int());
        hal_ = std::make_unique<SimMechanismHAL>(cfg);
        RCLCPP_INFO(this->get_logger(), "configured mechanism HAL: sim (latency=%lldms fail_rate=%.3f)",
                    static_cast<long long>(cfg.action_latency.count()), static_cast<double>(cfg.fail_rate));
    } else if (hal_type == "fault") {
        FaultInjectingHAL::Config cfg{};
        cfg.action_latency = std::chrono::milliseconds(this->get_parameter("fault_action_latency_ms").as_int());
        cfg.mode = this->get_parameter("fault_mode").as_string();
        cfg.fault_error_code = static_cast<uint8_t>(this->get_parameter("fault_error_code").as_int());
        hal_ = std::make_unique<FaultInjectingHAL>(cfg);
        RCLCPP_INFO(this->get_logger(), "configured mechanism HAL: fault (mode=%s latency=%lldms)",
                    cfg.mode.c_str(), static_cast<long long>(cfg.action_latency.count()));
    } else if (hal_type == "replay") {
        ReplayMechanismHAL::Config cfg{};
        cfg.replay_file = this->get_parameter("replay_file").as_string();
        cfg.action_latency = std::chrono::milliseconds(this->get_parameter("replay_action_latency_ms").as_int());
        cfg.loop = this->get_parameter("replay_loop").as_bool();
        hal_ = std::make_unique<ReplayMechanismHAL>(cfg);
        RCLCPP_INFO(this->get_logger(), "configured mechanism HAL: replay (file=%s latency=%lldms loop=%s)",
                    cfg.replay_file.c_str(), static_cast<long long>(cfg.action_latency.count()),
                    cfg.loop ? "true" : "false");
    } else {
        RCLCPP_ERROR(this->get_logger(), "unsupported hal_type: %s", hal_type.c_str());
        return CallbackReturn::FAILURE;
    }

    hal_->setFeedbackCallback(
        [this](uint8_t seq, uint8_t fb_id, const std::vector<uint8_t>& payload) {
            onSerialFeedback(seq, fb_id, payload);
        });

    if (!hal_->open()) {
        RCLCPP_ERROR(this->get_logger(), "failed to open mechanism HAL (type=%s)", hal_type.c_str());
        hal_->close();
        hal_.reset();
        return CallbackReturn::FAILURE;
    }

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
    if (hal_ && !hal_->isOpen()) {
        if (!hal_->open()) {
            RCLCPP_ERROR(this->get_logger(), "failed to reopen mechanism HAL on activate");
            return CallbackReturn::FAILURE;
        }
    }
    active_.store(true, std::memory_order_relaxed);
    cancel_requested_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
        pending_contexts_.clear();
        buffered_feedbacks_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
        cmd_start_time_ = std::chrono::steady_clock::now();
    }

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
    active_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
        drainPendingContexts();
        buffered_feedbacks_.clear();
    }
    feedback_cv_.notify_all();

    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }

    if (state_timer_) {
        state_timer_->cancel();
        state_timer_.reset();
    }
    if (state_pub_) {
        state_pub_->on_deactivate();
    }
    if (hal_) {
        hal_->close();
    }
    joinExecutionThread();
    execution_in_progress_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
    }
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_cleanup(const rclcpp_lifecycle::State&) {
    active_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
        drainPendingContexts();
        buffered_feedbacks_.clear();
    }
    feedback_cv_.notify_all();

    if (hal_) {
        hal_->close();
    }
    joinExecutionThread();
    execution_in_progress_.store(false, std::memory_order_release);
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
        hal_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
    }
    return CallbackReturn::SUCCESS;
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_error(const rclcpp_lifecycle::State&) {
    active_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
        drainPendingContexts();
        buffered_feedbacks_.clear();
    }
    feedback_cv_.notify_all();
    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
    }
    if (hal_) {
        hal_->close();
    }
    joinExecutionThread();
    execution_in_progress_.store(false, std::memory_order_release);
    return CallbackReturn::SUCCESS;
}

bool MechanismLifecycleServer::tryReserveExecution() {
    bool expected = false;
    return execution_in_progress_.compare_exchange_strong(
        expected, true, std::memory_order_acq_rel, std::memory_order_acquire);
}

void MechanismLifecycleServer::releaseExecution() {
    execution_in_progress_.store(false, std::memory_order_release);
}

void MechanismLifecycleServer::launchExecutionThread(std::function<void()> task) {
    std::thread previous_thread;
    {
        std::lock_guard<std::mutex> lock(execution_thread_mutex_);
        if (execution_thread_.joinable()) {
            previous_thread = std::move(execution_thread_);
        }
    }
    if (previous_thread.joinable()) {
        previous_thread.join();
    }

    std::thread next_thread(std::move(task));
    {
        std::lock_guard<std::mutex> lock(execution_thread_mutex_);
        execution_thread_ = std::move(next_thread);
    }
}

void MechanismLifecycleServer::joinExecutionThread() {
    std::thread worker;
    {
        std::lock_guard<std::mutex> lock(execution_thread_mutex_);
        if (!execution_thread_.joinable()) {
            return;
        }
        if (execution_thread_.get_id() == std::this_thread::get_id()) {
            return;
        }
        worker = std::move(execution_thread_);
    }
    worker.join();
}

void MechanismLifecycleServer::pruneBufferedFeedbacksLocked(
    const std::chrono::steady_clock::time_point& now) {
    for (auto it = buffered_feedbacks_.begin(); it != buffered_feedbacks_.end();) {
        if (now - it->second.received_at > kBufferedFeedbackTtl) {
            it = buffered_feedbacks_.erase(it);
            continue;
        }
        ++it;
    }
}

std::optional<CommandResult> MechanismLifecycleServer::takeBufferedCommandResultLocked(uint8_t seq,
                                                                                        uint8_t cmd_id) {
    auto it = buffered_feedbacks_.find(seq);
    if (it == buffered_feedbacks_.end()) {
        return std::nullopt;
    }

    const auto buffered = std::move(it->second);
    buffered_feedbacks_.erase(it);

    using FID = rc26_serial::FeedbackID;
    if (buffered.fb_id == static_cast<uint8_t>(FID::ACTION_FAIL)) {
        CommandResult result{};
        result.success = false;
        result.error_code = (buffered.payload.size() >= 2U) ? buffered.payload[1] : 0xFFU;
        return result;
    }
    if (buffered.fb_id == static_cast<uint8_t>(FID::ERROR)) {
        return CommandResult{false, 0xFFU, false, false};
    }
    if (isTerminalFeedbackForCommand(cmd_id, buffered.fb_id)) {
        return CommandResult{true, 0U, false, false};
    }
    return std::nullopt;
}

rclcpp_action::GoalResponse MechanismLifecycleServer::handleGrabGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GrabTip::Goal> goal) {
    (void)uuid;
    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen() || !goal ||
        goal->tip_index > 5) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!tryReserveExecution()) {
        RCLCPP_WARN(this->get_logger(), "reject grab goal while another mechanism action is running");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handleGrabCancel(
    const std::shared_ptr<GoalHandleGrabTip>& /*goal_handle*/) {
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
    }
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    feedback_cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handleGrabAccepted(const std::shared_ptr<GoalHandleGrabTip>& goal_handle) {
    try {
        launchExecutionThread([this, goal_handle]() {
            try {
                executeGrab(goal_handle);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "executeGrab threw: %s", e.what());
                auto result = std::make_shared<GrabTip::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "executeGrab threw unknown exception");
                auto result = std::make_shared<GrabTip::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            }
        });
    } catch (const std::exception& e) {
        releaseExecution();
        RCLCPP_ERROR(this->get_logger(), "failed to start grab execution thread: %s", e.what());
        auto result = std::make_shared<GrabTip::Result>();
        result->success = false;
        result->error_code = 5;
        goal_handle->abort(result);
    }
}

void MechanismLifecycleServer::executeGrab(const std::shared_ptr<GoalHandleGrabTip>& goal_handle) {
    auto execution_guard = ScopeExit([this]() { releaseExecution(); });
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<GrabTip::Result>();
    auto feedback = std::make_shared<GrabTip::Feedback>();
    result->success = false;
    result->error_code = 0;

    const auto goal = goal_handle->get_goal();
    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen() || !goal) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    cancel_requested_.store(false, std::memory_order_relaxed);

    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        if (!tip_sm_.transition(TipState::APPROACHING)) {
            result->error_code = 2;
            goal_handle->abort(result);
            return;
        }
        tip_sm_.lockedTipSlot = goal->tip_index;
    }

    feedback->state = static_cast<uint8_t>(TipState::APPROACHING);
    goal_handle->publish_feedback(feedback);

    const auto cmd_id = static_cast<uint8_t>(rc26_serial::CommandID::GRAB_TIP);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_error_code_ = 0;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(cmd_id, std::memory_order_relaxed);
        cmd_start_time_ = std::chrono::steady_clock::now();
    }

    const auto command_result =
        executeWithContext(cmd_id, {goal->tip_index}, std::chrono::seconds(8),
                           [goal_handle, feedback]() { goal_handle->publish_feedback(feedback); });

    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
    }

    if (goal_handle->is_canceling() || command_result.canceled ||
        cancel_requested_.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::IDLE);
        tip_sm_.lockedTipSlot.reset();
        goal_handle->canceled(result);
        return;
    }
    if (!command_result.success) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::IDLE);
        tip_sm_.lockedTipSlot.reset();
        result->error_code = command_result.error_code;
        if (result->error_code == 0U) {
            result->error_code = last_error_code_;
        }
        last_error_code_ = result->error_code;
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

    feedback->state = static_cast<uint8_t>(TipState::HOLDING);
    goal_handle->publish_feedback(feedback);

    result->success = true;
    goal_handle->succeed(result);
    publishMechanismState();
}

rclcpp_action::GoalResponse MechanismLifecycleServer::handleAssembleGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const AssembleWeapon::Goal> goal) {
    (void)uuid;
    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen() || !goal) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!tryReserveExecution()) {
        RCLCPP_WARN(this->get_logger(), "reject assemble goal while another mechanism action is running");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handleAssembleCancel(
    const std::shared_ptr<GoalHandleAssemble>& /*goal_handle*/) {
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
    }
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    feedback_cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handleAssembleAccepted(const std::shared_ptr<GoalHandleAssemble>& goal_handle) {
    try {
        launchExecutionThread([this, goal_handle]() {
            try {
                executeAssemble(goal_handle);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "executeAssemble threw: %s", e.what());
                auto result = std::make_shared<AssembleWeapon::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "executeAssemble threw unknown exception");
                auto result = std::make_shared<AssembleWeapon::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            }
        });
    } catch (const std::exception& e) {
        releaseExecution();
        RCLCPP_ERROR(this->get_logger(), "failed to start assemble execution thread: %s", e.what());
        auto result = std::make_shared<AssembleWeapon::Result>();
        result->success = false;
        result->error_code = 5;
        goal_handle->abort(result);
    }
}

void MechanismLifecycleServer::executeAssemble(const std::shared_ptr<GoalHandleAssemble>& goal_handle) {
    auto execution_guard = ScopeExit([this]() { releaseExecution(); });
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<AssembleWeapon::Result>();
    auto feedback = std::make_shared<AssembleWeapon::Feedback>();
    result->success = false;
    result->error_code = 0;

    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen()) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    cancel_requested_.store(false, std::memory_order_relaxed);

    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        if (!tip_sm_.isHoldingTip() || !tip_sm_.transition(TipState::ASSEMBLING)) {
            result->error_code = 2;
            goal_handle->abort(result);
            return;
        }
    }

    feedback->state = static_cast<uint8_t>(TipState::ASSEMBLING);
    goal_handle->publish_feedback(feedback);

    const auto cmd_id = static_cast<uint8_t>(rc26_serial::CommandID::ASSEMBLE_WEAPON);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_error_code_ = 0;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(cmd_id, std::memory_order_relaxed);
        cmd_start_time_ = std::chrono::steady_clock::now();
    }

    const auto command_result =
        executeWithContext(cmd_id, {}, std::chrono::seconds(30),
                           [goal_handle, feedback]() { goal_handle->publish_feedback(feedback); });

    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
    }

    if (goal_handle->is_canceling() || command_result.canceled ||
        cancel_requested_.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::HOLDING);
        goal_handle->canceled(result);
        return;
    }
    if (!command_result.success) {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        (void)tip_sm_.transition(TipState::HOLDING);
        result->error_code = command_result.error_code;
        if (result->error_code == 0U) {
            result->error_code = last_error_code_;
        }
        last_error_code_ = result->error_code;
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
    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen() || !goal ||
        goal->layer == 0) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!tryReserveExecution()) {
        RCLCPP_WARN(this->get_logger(), "reject place goal while another mechanism action is running");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handlePlaceCancel(
    const std::shared_ptr<GoalHandlePlace>& /*goal_handle*/) {
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
    }
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    feedback_cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handlePlaceAccepted(const std::shared_ptr<GoalHandlePlace>& goal_handle) {
    try {
        launchExecutionThread([this, goal_handle]() {
            try {
                executePlace(goal_handle);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "executePlace threw: %s", e.what());
                auto result = std::make_shared<PlaceKFSGrid::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "executePlace threw unknown exception");
                auto result = std::make_shared<PlaceKFSGrid::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            }
        });
    } catch (const std::exception& e) {
        releaseExecution();
        RCLCPP_ERROR(this->get_logger(), "failed to start place execution thread: %s", e.what());
        auto result = std::make_shared<PlaceKFSGrid::Result>();
        result->success = false;
        result->error_code = 5;
        goal_handle->abort(result);
    }
}

void MechanismLifecycleServer::executePlace(const std::shared_ptr<GoalHandlePlace>& goal_handle) {
    auto execution_guard = ScopeExit([this]() { releaseExecution(); });
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<PlaceKFSGrid::Result>();
    auto feedback = std::make_shared<PlaceKFSGrid::Feedback>();
    result->success = false;
    result->error_code = 0;

    const auto goal = goal_handle->get_goal();
    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen() || !goal || goal->layer == 0) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    cancel_requested_.store(false, std::memory_order_relaxed);

    feedback->state = 1;
    goal_handle->publish_feedback(feedback);

    const auto cmd_id = static_cast<uint8_t>(rc26_serial::CommandID::PLACE_KFS_GRID);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_error_code_ = 0;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(cmd_id, std::memory_order_relaxed);
        cmd_start_time_ = std::chrono::steady_clock::now();
    }

    const auto command_result =
        executeWithContext(cmd_id, {goal->grid_position, goal->layer}, std::chrono::seconds(8),
                           [goal_handle, feedback]() { goal_handle->publish_feedback(feedback); });

    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
    }

    if (goal_handle->is_canceling() || command_result.canceled ||
        cancel_requested_.load(std::memory_order_relaxed)) {
        goal_handle->canceled(result);
        return;
    }
    if (!command_result.success) {
        result->error_code = command_result.error_code;
        if (result->error_code == 0U) {
            result->error_code = last_error_code_;
        }
        last_error_code_ = result->error_code;
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
    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen() || !goal ||
        !isCommandSupported(goal->command_id)) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!tryReserveExecution()) {
        RCLCPP_WARN(this->get_logger(), "reject execute goal while another mechanism action is running");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MechanismLifecycleServer::handleExecuteCancel(
    const std::shared_ptr<GoalHandleExecute>& /*goal_handle*/) {
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
    }
    if (hal_ && hal_->isOpen()) {
        (void)hal_->sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::STOP));
    }
    feedback_cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MechanismLifecycleServer::handleExecuteAccepted(const std::shared_ptr<GoalHandleExecute>& goal_handle) {
    try {
        launchExecutionThread([this, goal_handle]() {
            try {
                executeCommand(goal_handle);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "executeCommand threw: %s", e.what());
                auto result = std::make_shared<ExecuteMechanism::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "executeCommand threw unknown exception");
                auto result = std::make_shared<ExecuteMechanism::Result>();
                result->success = false;
                result->error_code = 5;
                goal_handle->abort(result);
                releaseExecution();
            }
        });
    } catch (const std::exception& e) {
        releaseExecution();
        RCLCPP_ERROR(this->get_logger(), "failed to start execute thread: %s", e.what());
        auto result = std::make_shared<ExecuteMechanism::Result>();
        result->success = false;
        result->error_code = 5;
        goal_handle->abort(result);
    }
}

void MechanismLifecycleServer::executeCommand(const std::shared_ptr<GoalHandleExecute>& goal_handle) {
    auto execution_guard = ScopeExit([this]() { releaseExecution(); });
    std::unique_lock<std::mutex> exec_lock(execution_mutex_);

    auto result = std::make_shared<ExecuteMechanism::Result>();
    auto feedback = std::make_shared<ExecuteMechanism::Feedback>();
    result->success = false;
    result->error_code = 0;

    const auto goal = goal_handle->get_goal();
    if (!active_.load(std::memory_order_relaxed) || !hal_ || !hal_->isOpen() || !goal ||
        !isCommandSupported(goal->command_id)) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    cancel_requested_.store(false, std::memory_order_relaxed);

    feedback->state = 1;
    goal_handle->publish_feedback(feedback);

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_error_code_ = 0;
    }

    const auto timeout = timeoutFromGoal(goal->timeout_sec, defaultTimeoutForCommand(goal->command_id));
    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(goal->command_id, std::memory_order_relaxed);
        cmd_start_time_ = std::chrono::steady_clock::now();
    }

    const auto command_result =
        executeWithContext(goal->command_id, goal->payload, timeout,
                           [goal_handle, feedback]() { goal_handle->publish_feedback(feedback); });

    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        current_cmd_id_.store(0, std::memory_order_relaxed);
    }

    if (goal_handle->is_canceling() || command_result.canceled ||
        cancel_requested_.load(std::memory_order_relaxed)) {
        goal_handle->canceled(result);
        return;
    }
    if (!command_result.success) {
        result->error_code = command_result.error_code;
        if (result->error_code == 0U) {
            result->error_code = last_error_code_;
        }
        last_error_code_ = result->error_code;
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

bool MechanismLifecycleServer::isTerminalFeedbackForCommand(uint8_t cmd_id, uint8_t fb_id) const {
    using CID = rc26_serial::CommandID;
    using FID = rc26_serial::FeedbackID;
    switch (static_cast<CID>(cmd_id)) {
    case CID::GRAB_TIP:
        return fb_id == static_cast<uint8_t>(FID::GRAB_TIP_DONE);
    case CID::ASSEMBLE_WEAPON:
        return fb_id == static_cast<uint8_t>(FID::ASSEMBLE_DONE);
    case CID::PLACE_KFS_GRID:
        return fb_id == static_cast<uint8_t>(FID::PLACE_KFS_GRID_DONE);
    case CID::GRAB_KFS:
        return fb_id == static_cast<uint8_t>(FID::GRAB_KFS_DONE);
    case CID::MECH_UP_MERLIN:
        return fb_id == static_cast<uint8_t>(FID::MECH_UP_MERLIN_DONE);
    case CID::MECH_DOWN_MERLIN:
        return fb_id == static_cast<uint8_t>(FID::MECH_DOWN_MERLIN_DONE);
    case CID::MECH_UP_DUEL:
        return fb_id == static_cast<uint8_t>(FID::MECH_UP_DUEL_DONE);
    case CID::PLACE_KFS_GROUND:
        return fb_id == static_cast<uint8_t>(FID::PLACE_KFS_GROUND_DONE);
    case CID::ROTATE_POS_90:
        return fb_id == static_cast<uint8_t>(FID::ROTATE_POS_90_DONE);
    case CID::ROTATE_NEG_90:
        return fb_id == static_cast<uint8_t>(FID::ROTATE_NEG_90_DONE);
    case CID::ROTATE_POS_180:
        return fb_id == static_cast<uint8_t>(FID::ROTATE_POS_180_DONE);
    case CID::ROTATE_NEG_180:
        return fb_id == static_cast<uint8_t>(FID::ROTATE_NEG_180_DONE);
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

CommandResult MechanismLifecycleServer::executeWithContext(uint8_t cmd_id, const std::vector<uint8_t>& payload,
                                                           std::chrono::milliseconds timeout,
                                                           const std::function<void()>& keep_alive) {
    auto execute_once = [&]() {
        CommandResult command_result{};
        uint8_t out_seq = 0;
        auto context = std::make_shared<CommandContext>();
        context->cmd_id = cmd_id;
        context->timeout = timeout;
        auto future = context->result_promise.get_future();

        const bool send_ok = hal_->sendCommand(cmd_id, payload, out_seq);
        context->seq = out_seq;
        context->start = std::chrono::steady_clock::now();

        bool result_ready = false;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            pruneBufferedFeedbacksLocked(std::chrono::steady_clock::now());
            if (const auto buffered_result = takeBufferedCommandResultLocked(out_seq, cmd_id)) {
                command_result = *buffered_result;
                result_ready = true;
                if (!context->fulfilled.exchange(true, std::memory_order_relaxed)) {
                    try {
                        context->result_promise.set_value(command_result);
                    } catch (const std::future_error&) {
                    }
                }
            } else if (send_ok) {
                pending_contexts_[out_seq] = context;
            }
        }

        if (!send_ok) {
            if (result_ready) {
                return command_result;
            }
            command_result.error_code = 3;
            return command_result;
        }

        if (result_ready) {
            return command_result;
        }

        auto next_keep_alive = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        const auto deadline = context->start + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cancel_requested_.load(std::memory_order_relaxed) || context->cancel_requested.load()) {
                command_result.canceled = true;
                break;
            }

            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            const auto wait_step =
                std::max(std::chrono::milliseconds(1), std::min(remaining, std::chrono::milliseconds(50)));
            if (future.wait_for(wait_step) == std::future_status::ready) {
                command_result = future.get();
                result_ready = true;
                break;
            }

            if (keep_alive && std::chrono::steady_clock::now() >= next_keep_alive) {
                keep_alive();
                next_keep_alive = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
            }
        }

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            auto it = pending_contexts_.find(out_seq);
            if (it != pending_contexts_.end()) {
                if (command_result.canceled || command_result.timed_out) {
                    it->second->cancel_requested.store(true, std::memory_order_relaxed);
                }
                pending_contexts_.erase(it);
            }
        }

        if (!result_ready && !command_result.canceled) {
            command_result.timed_out = true;
            command_result.error_code = 0xFF;
        }
        return command_result;
    };

    constexpr int max_retries = 2;
    CommandResult command_result{};
    for (int attempt = 0; attempt <= max_retries; ++attempt) {
        command_result = execute_once();
        if (command_result.success || command_result.canceled) {
            return command_result;
        }

        const auto health = hal_ ? hal_->commHealthSnapshot() : CommHealthSnapshot{};
        if (command_result.timed_out) {
            if (health.comm_health_level == 0U) {
                break;
            }
            if (attempt < max_retries) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100 * (attempt + 1)));
                continue;
            }
            break;
        }
        if (isFatalErrorCode(command_result.error_code)) {
            break;
        }
        if (attempt < max_retries && health.comm_health_level >= 1U) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100 * (attempt + 1)));
            continue;
        }
        break;
    }

    return command_result;
}

void MechanismLifecycleServer::onSerialFeedback(uint8_t seq, uint8_t fb_id,
                                                const std::vector<uint8_t>& payload) {
    using FID = rc26_serial::FeedbackID;

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    pruneBufferedFeedbacksLocked(std::chrono::steady_clock::now());
    auto pending_it = pending_contexts_.find(seq);
    if (pending_it != pending_contexts_.end()) {
        auto context = pending_it->second;
        if (fb_id == static_cast<uint8_t>(FID::ACTION_FAIL) || fb_id == static_cast<uint8_t>(FID::ERROR)) {
            CommandResult command_result{};
            command_result.success = false;
            command_result.error_code = (fb_id == static_cast<uint8_t>(FID::ACTION_FAIL) && payload.size() >= 2U)
                                            ? payload[1]
                                            : 0xFFU;
            last_error_code_.store(command_result.error_code, std::memory_order_relaxed);
            if (!context->fulfilled.exchange(true, std::memory_order_relaxed)) {
                try {
                    context->result_promise.set_value(command_result);
                } catch (const std::future_error&) {
                }
            }
            if (fb_id == static_cast<uint8_t>(FID::ACTION_FAIL)) {
                if (payload.size() >= 2U) {
                    RCLCPP_WARN(this->get_logger(), "ACTION_FAIL: seq=0x%02X cmd=0x%02X err=0x%02X", seq,
                                payload[0], payload[1]);
                } else if (payload.size() == 1U) {
                    RCLCPP_WARN(this->get_logger(), "ACTION_FAIL: seq=0x%02X cmd=0x%02X (no error_code)", seq,
                                payload[0]);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "ACTION_FAIL: seq=0x%02X empty payload", seq);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "mechanism ERROR feedback: seq=0x%02X", seq);
            }
            pending_contexts_.erase(pending_it);
            feedback_cv_.notify_all();
            return;
        } else if (isTerminalFeedbackForCommand(context->cmd_id, fb_id)) {
            CommandResult command_result{};
            command_result.success = true;
            if (!context->fulfilled.exchange(true, std::memory_order_relaxed)) {
                try {
                    context->result_promise.set_value(command_result);
                } catch (const std::future_error&) {
                }
            }
            pending_contexts_.erase(pending_it);
            feedback_cv_.notify_all();
            return;
        }
    }

    if (isTerminalOrFailureFeedback(fb_id)) {
        buffered_feedbacks_[seq] = BufferedFeedback{fb_id, payload, std::chrono::steady_clock::now()};
        if (fb_id == static_cast<uint8_t>(FID::ACTION_FAIL)) {
            if (payload.size() >= 2U) {
                last_error_code_.store(payload[1], std::memory_order_relaxed);
                RCLCPP_WARN(this->get_logger(),
                            "buffered early ACTION_FAIL: seq=0x%02X cmd=0x%02X err=0x%02X", seq, payload[0],
                            payload[1]);
            } else {
                last_error_code_.store(0xFFU, std::memory_order_relaxed);
                RCLCPP_WARN(this->get_logger(), "buffered early ACTION_FAIL without full payload: seq=0x%02X", seq);
            }
        } else if (fb_id == static_cast<uint8_t>(FID::ERROR)) {
            last_error_code_.store(0xFFU, std::memory_order_relaxed);
            RCLCPP_ERROR(this->get_logger(), "buffered early ERROR feedback: seq=0x%02X", seq);
        }
        feedback_cv_.notify_all();
        return;
    }

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
        if (payload.size() >= 2) {
            bool* done = nullptr;
            bool* failed = nullptr;
            if (commandFlagsFor(payload[0], done, failed) && failed) {
                *failed = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "ACTION_FAIL unknown cmd_id=0x%02X", payload[0]);
            }
            last_error_code_ = payload[1];
            RCLCPP_WARN(this->get_logger(), "ACTION_FAIL: seq=0x%02X cmd=0x%02X err=0x%02X", seq,
                        payload[0], payload[1]);
        } else if (payload.size() == 1) {
            bool* done = nullptr;
            bool* failed = nullptr;
            if (commandFlagsFor(payload[0], done, failed) && failed) {
                *failed = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "ACTION_FAIL unknown cmd_id=0x%02X", payload[0]);
            }
            last_error_code_ = 0xFF;
            RCLCPP_WARN(this->get_logger(), "ACTION_FAIL: seq=0x%02X cmd=0x%02X (no error_code)",
                        seq, payload[0]);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "ACTION_FAIL: seq=0x%02X empty payload, broadcast all failed", seq);
            grab_failed_ = true;
            assemble_failed_ = true;
            place_failed_ = true;
            grab_kfs_failed_ = true;
            mech_up_merlin_failed_ = true;
            mech_down_merlin_failed_ = true;
            mech_up_duel_failed_ = true;
            place_ground_failed_ = true;
            rotate_failed_ = true;
            last_error_code_ = 0xFF;
        }
        break;
    default:
        break;
    }
    feedback_cv_.notify_all();
}

void MechanismLifecycleServer::drainPendingContexts() {
    for (auto& [seq, ctx] : pending_contexts_) {
        if (!ctx->fulfilled.exchange(true, std::memory_order_relaxed)) {
            try {
                ctx->result_promise.set_value(CommandResult{false, 0, false, true});
            } catch (const std::future_error&) {
            }
        }
    }
    pending_contexts_.clear();
    buffered_feedbacks_.clear();
}

void MechanismLifecycleServer::publishMechanismState() {
    if (!state_pub_ || !state_pub_->is_activated()) {
        return;
    }

    rc26_interfaces::msg::MechanismState msg;
    msg.header.stamp = this->get_clock()->now();
    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        msg.tip_state = static_cast<uint8_t>(tip_sm_.current());
        msg.locked_tip_slot = tip_sm_.lockedTipSlot.value_or(255);
    }
    msg.hal_open = hal_ && hal_->isOpen();
    msg.assembled_count = assembled_count_.load(std::memory_order_relaxed);
    msg.last_error_code = last_error_code_.load(std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(cmd_state_mutex_);
        msg.current_cmd_id = current_cmd_id_.load(std::memory_order_relaxed);
        msg.cmd_elapsed_ms =
            (msg.current_cmd_id != 0U)
                ? static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                            std::chrono::steady_clock::now() - cmd_start_time_)
                                            .count())
                : 0U;
    }
    const auto health = hal_ ? hal_->commHealthSnapshot() : CommHealthSnapshot{};
    msg.ack_timeout_count = health.ack_timeout_count;
    msg.reconnect_count = health.reconnect_count;
    msg.parse_error_count = health.parse_error_count;
    msg.avg_rtt_ms = health.avg_rtt_ms;
    msg.comm_health_level = health.comm_health_level;
    state_pub_->publish(msg);
}

bool MechanismLifecycleServer::waitUntilDoneOrFailed(
    bool& done, bool& failed, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(feedback_mutex_);
    return feedback_cv_.wait_for(lock, timeout, [&done, &failed, this]() {
        return done || failed || cancel_requested_.load(std::memory_order_relaxed);
    });
}

}  // namespace rc26_mechanism

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_mechanism::MechanismLifecycleServer)
