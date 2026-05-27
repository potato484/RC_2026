#include "rc26_mechanism/nodes/mechanism_lifecycle_server.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

#include "rc26_mechanism/catalog/mechanism_command_catalog.hpp"
#include "rc26_mechanism/hal/shared_serial/shared_serial_mechanism_hal.hpp"
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
constexpr uint8_t kRunningFeedbackState = 1U;

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

}  // namespace

MechanismLifecycleServer::MechanismLifecycleServer(const rclcpp::NodeOptions& opts)
    : rclcpp_lifecycle::LifecycleNode("mechanism_server", opts) {
    this->declare_parameter<std::string>("hal_type", "shared_serial");
}

MechanismLifecycleServer::~MechanismLifecycleServer() {
    active_.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        cancel_requested_.store(true, std::memory_order_relaxed);
        drainPendingContexts();
        buffered_feedbacks_.clear();
    }
    if (hal_) {
        hal_->close();
    }
    joinExecutionThread();
    execution_in_progress_.store(false, std::memory_order_release);
}

MechanismLifecycleServer::CallbackReturn
MechanismLifecycleServer::on_configure(const rclcpp_lifecycle::State&) {
    const auto hal_type = this->get_parameter("hal_type").as_string();
    if (hal_type != "shared_serial") {
        RCLCPP_ERROR(this->get_logger(),
                     "unsupported hal_type: %s (shared_serial is the only supported runtime HAL)",
                     hal_type.c_str());
        return CallbackReturn::FAILURE;
    }

    hal_ = std::make_unique<SharedSerialMechanismHAL>(*this);
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
        "/mechanism/status", rclcpp::QoS(1).reliable());

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

    execute_srv_ = rclcpp_action::create_server<ExecuteMechanism>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/mechanism/run_command",
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
    current_cmd_id_.store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
        pending_contexts_.clear();
        buffered_feedbacks_.clear();
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
    current_cmd_id_.store(0, std::memory_order_relaxed);
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
    execute_srv_.reset();
    hal_.reset();
    current_cmd_id_.store(0, std::memory_order_relaxed);
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
    if (hal_) {
        hal_->close();
    }
    joinExecutionThread();
    execution_in_progress_.store(false, std::memory_order_release);
    current_cmd_id_.store(0, std::memory_order_relaxed);
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

    if (isTerminalSuccessFeedbackForMechanismCommand(cmd_id, buffered.fb_id)) {
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
    feedback->state = kRunningFeedbackState;
    goal_handle->publish_feedback(feedback);

    const auto cmd_id = static_cast<uint8_t>(rc26_serial::CommandID::GRAB_TIP);
    last_error_code_.store(0, std::memory_order_relaxed);
    current_cmd_id_.store(cmd_id, std::memory_order_relaxed);

    const auto command_result =
        executeWithContext(cmd_id, {goal->tip_index}, defaultTimeoutForMechanismCommand(cmd_id),
                           [goal_handle, feedback]() { goal_handle->publish_feedback(feedback); });

    current_cmd_id_.store(0, std::memory_order_relaxed);

    if (goal_handle->is_canceling() || command_result.canceled ||
        cancel_requested_.load(std::memory_order_relaxed)) {
        goal_handle->canceled(result);
        publishMechanismState();
        return;
    }
    if (!command_result.success) {
        result->error_code = command_result.error_code;
        if (result->error_code == 0U) {
            result->error_code = last_error_code_.load(std::memory_order_relaxed);
        }
        last_error_code_.store(result->error_code, std::memory_order_relaxed);
        goal_handle->abort(result);
        publishMechanismState();
        return;
    }

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
    feedback->state = kRunningFeedbackState;
    goal_handle->publish_feedback(feedback);

    const auto cmd_id = static_cast<uint8_t>(rc26_serial::CommandID::ASSEMBLE_WEAPON);
    last_error_code_.store(0, std::memory_order_relaxed);
    current_cmd_id_.store(cmd_id, std::memory_order_relaxed);

    const auto command_result =
        executeWithContext(cmd_id, {}, defaultTimeoutForMechanismCommand(cmd_id),
                           [goal_handle, feedback]() { goal_handle->publish_feedback(feedback); });

    current_cmd_id_.store(0, std::memory_order_relaxed);

    if (goal_handle->is_canceling() || command_result.canceled ||
        cancel_requested_.load(std::memory_order_relaxed)) {
        goal_handle->canceled(result);
        publishMechanismState();
        return;
    }
    if (!command_result.success) {
        result->error_code = command_result.error_code;
        if (result->error_code == 0U) {
            result->error_code = last_error_code_.load(std::memory_order_relaxed);
        }
        last_error_code_.store(result->error_code, std::memory_order_relaxed);
        goal_handle->abort(result);
        publishMechanismState();
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
        !isExecuteSupportedMechanismCommand(goal->command_id)) {
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
        !isExecuteSupportedMechanismCommand(goal->command_id)) {
        result->error_code = 1;
        goal_handle->abort(result);
        return;
    }

    cancel_requested_.store(false, std::memory_order_relaxed);
    feedback->state = kRunningFeedbackState;
    goal_handle->publish_feedback(feedback);

    last_error_code_.store(0, std::memory_order_relaxed);
    const auto timeout = timeoutFromGoal(goal->timeout_sec, defaultTimeoutForMechanismCommand(goal->command_id));
    current_cmd_id_.store(goal->command_id, std::memory_order_relaxed);

    const auto command_result =
        executeWithContext(goal->command_id, goal->payload, timeout,
                           [goal_handle, feedback]() { goal_handle->publish_feedback(feedback); });

    current_cmd_id_.store(0, std::memory_order_relaxed);

    if (goal_handle->is_canceling() || command_result.canceled ||
        cancel_requested_.load(std::memory_order_relaxed)) {
        goal_handle->canceled(result);
        publishMechanismState();
        return;
    }
    if (!command_result.success) {
        result->error_code = command_result.error_code;
        if (result->error_code == 0U) {
            result->error_code = last_error_code_.load(std::memory_order_relaxed);
        }
        last_error_code_.store(result->error_code, std::memory_order_relaxed);
        goal_handle->abort(result);
        publishMechanismState();
        return;
    }

    result->success = true;
    goal_handle->succeed(result);
    publishMechanismState();
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
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    pruneBufferedFeedbacksLocked(std::chrono::steady_clock::now());
    auto pending_it = pending_contexts_.find(seq);
    if (pending_it != pending_contexts_.end()) {
        auto context = pending_it->second;
        if (isTerminalSuccessFeedbackForMechanismCommand(context->cmd_id, fb_id)) {
            CommandResult command_result{};
            command_result.success = true;
            if (!context->fulfilled.exchange(true, std::memory_order_relaxed)) {
                try {
                    context->result_promise.set_value(command_result);
                } catch (const std::future_error&) {
                }
            }
            pending_contexts_.erase(pending_it);
            return;
        }
    }

    if (isTerminalMechanismFeedback(fb_id)) {
        buffered_feedbacks_[seq] = BufferedFeedback{fb_id, payload, std::chrono::steady_clock::now()};
    }
}

void MechanismLifecycleServer::drainPendingContexts() {
    for (auto& [seq, ctx] : pending_contexts_) {
        (void)seq;
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
    msg.hal_open = hal_ && hal_->isOpen();
    msg.last_error_code = last_error_code_.load(std::memory_order_relaxed);
    msg.current_cmd_id = current_cmd_id_.load(std::memory_order_relaxed);
    state_pub_->publish(msg);
}

}  // namespace rc26_mechanism

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_mechanism::MechanismLifecycleServer)
