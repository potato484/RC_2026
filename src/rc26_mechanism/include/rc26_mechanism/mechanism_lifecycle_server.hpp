#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rc26_interfaces/action/assemble_weapon.hpp"
#include "rc26_interfaces/action/execute_mechanism.hpp"
#include "rc26_interfaces/action/grab_tip.hpp"
#include "rc26_interfaces/action/place_kfs_grid.hpp"
#include "rc26_interfaces/msg/mechanism_state.hpp"

#include "rc26_mechanism/command_context.hpp"
#include "rc26_mechanism/hal/i_mechanism_hal.hpp"
#include "rc26_mechanism/tip_state_machine.hpp"

namespace rc26_mechanism {

class MechanismLifecycleServer : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MechanismLifecycleServer(const rclcpp::NodeOptions& opts);
    ~MechanismLifecycleServer() override;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

private:
    using GrabTip = rc26_interfaces::action::GrabTip;
    using AssembleWeapon = rc26_interfaces::action::AssembleWeapon;
    using PlaceKFSGrid = rc26_interfaces::action::PlaceKFSGrid;
    using ExecuteMechanism = rc26_interfaces::action::ExecuteMechanism;
    using GoalHandleGrabTip = rclcpp_action::ServerGoalHandle<GrabTip>;
    using GoalHandleAssemble = rclcpp_action::ServerGoalHandle<AssembleWeapon>;
    using GoalHandlePlace = rclcpp_action::ServerGoalHandle<PlaceKFSGrid>;
    using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteMechanism>;

    rclcpp_action::GoalResponse handleGrabGoal(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const GrabTip::Goal> goal);
    rclcpp_action::CancelResponse handleGrabCancel(
        const std::shared_ptr<GoalHandleGrabTip>& goal_handle);
    void handleGrabAccepted(const std::shared_ptr<GoalHandleGrabTip>& goal_handle);
    void executeGrab(const std::shared_ptr<GoalHandleGrabTip>& goal_handle);

    rclcpp_action::GoalResponse handleAssembleGoal(const rclcpp_action::GoalUUID& uuid,
                                                   std::shared_ptr<const AssembleWeapon::Goal> goal);
    rclcpp_action::CancelResponse handleAssembleCancel(
        const std::shared_ptr<GoalHandleAssemble>& goal_handle);
    void handleAssembleAccepted(const std::shared_ptr<GoalHandleAssemble>& goal_handle);
    void executeAssemble(const std::shared_ptr<GoalHandleAssemble>& goal_handle);

    rclcpp_action::GoalResponse handlePlaceGoal(const rclcpp_action::GoalUUID& uuid,
                                                std::shared_ptr<const PlaceKFSGrid::Goal> goal);
    rclcpp_action::CancelResponse handlePlaceCancel(
        const std::shared_ptr<GoalHandlePlace>& goal_handle);
    void handlePlaceAccepted(const std::shared_ptr<GoalHandlePlace>& goal_handle);
    void executePlace(const std::shared_ptr<GoalHandlePlace>& goal_handle);

    rclcpp_action::GoalResponse handleExecuteGoal(const rclcpp_action::GoalUUID& uuid,
                                                  std::shared_ptr<const ExecuteMechanism::Goal> goal);
    rclcpp_action::CancelResponse handleExecuteCancel(
        const std::shared_ptr<GoalHandleExecute>& goal_handle);
    void handleExecuteAccepted(const std::shared_ptr<GoalHandleExecute>& goal_handle);
    void executeCommand(const std::shared_ptr<GoalHandleExecute>& goal_handle);

    bool isCommandSupported(uint8_t cmd_id) const;
    bool isTerminalFeedbackForCommand(uint8_t cmd_id, uint8_t fb_id) const;
    std::chrono::milliseconds defaultTimeoutForCommand(uint8_t cmd_id) const;
    bool commandFlagsFor(uint8_t cmd_id, bool*& done, bool*& failed);
    void resetCommandFlags(uint8_t cmd_id);
    CommandResult executeWithContext(uint8_t cmd_id, const std::vector<uint8_t>& payload,
                                     std::chrono::milliseconds timeout,
                                     const std::function<void()>& keep_alive = {});
    bool tryReserveExecution();
    void releaseExecution();
    void launchExecutionThread(std::function<void()> task);
    void joinExecutionThread();

    struct BufferedFeedback {
        uint8_t fb_id{0};
        std::vector<uint8_t> payload;
        std::chrono::steady_clock::time_point received_at{};
    };
    void pruneBufferedFeedbacksLocked(const std::chrono::steady_clock::time_point& now);
    std::optional<CommandResult> takeBufferedCommandResultLocked(uint8_t seq, uint8_t cmd_id);

    void onSerialFeedback(uint8_t seq, uint8_t fb_id, const std::vector<uint8_t>& payload);
    void publishMechanismState();
    bool waitUntilDoneOrFailed(bool& done, bool& failed, std::chrono::milliseconds timeout);
    void drainPendingContexts();

    std::unique_ptr<IMechanismHAL> hal_;
    TipStateMachine tip_sm_;
    std::mutex sm_mutex_;

    rclcpp_action::Server<GrabTip>::SharedPtr grab_tip_srv_;
    rclcpp_action::Server<AssembleWeapon>::SharedPtr assemble_srv_;
    rclcpp_action::Server<PlaceKFSGrid>::SharedPtr place_grid_srv_;
    rclcpp_action::Server<ExecuteMechanism>::SharedPtr execute_srv_;
    rclcpp_lifecycle::LifecyclePublisher<rc26_interfaces::msg::MechanismState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    std::mutex execution_mutex_;
    std::mutex execution_thread_mutex_;
    std::thread execution_thread_;
    std::mutex cmd_state_mutex_;
    std::atomic<bool> active_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> execution_in_progress_{false};
    std::atomic<uint8_t> current_cmd_id_{0};
    std::chrono::steady_clock::time_point cmd_start_time_;

    std::mutex feedback_mutex_;
    std::condition_variable feedback_cv_;
    bool grab_done_{false};
    bool grab_failed_{false};
    bool assemble_done_{false};
    bool assemble_failed_{false};
    bool place_done_{false};
    bool place_failed_{false};
    bool grab_kfs_done_{false};
    bool grab_kfs_failed_{false};
    bool mech_up_merlin_done_{false};
    bool mech_up_merlin_failed_{false};
    bool mech_down_merlin_done_{false};
    bool mech_down_merlin_failed_{false};
    bool mech_up_duel_done_{false};
    bool mech_up_duel_failed_{false};
    bool place_ground_done_{false};
    bool place_ground_failed_{false};
    bool rotate_done_{false};
    bool rotate_failed_{false};
    std::unordered_map<uint8_t, std::shared_ptr<CommandContext>> pending_contexts_;
    std::unordered_map<uint8_t, BufferedFeedback> buffered_feedbacks_;
    std::atomic<uint16_t> last_error_code_{0};
    std::atomic<uint8_t> assembled_count_{0};
};

}  // namespace rc26_mechanism
