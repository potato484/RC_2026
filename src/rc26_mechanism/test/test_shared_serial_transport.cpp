#include <chrono>
#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rc26_interfaces/action/execute_mechanism.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_mechanism/mechanism_lifecycle_server.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_mechanism
{
namespace
{

using ExecuteMechanism = rc26_interfaces::action::ExecuteMechanism;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteMechanism>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FakeMechanismTransportProvider : public rclcpp::Node
{
public:
  enum class Mode
  {
    kFrontTrackUpSuccess,
    kFrontTrackDownSuccess,
    kRejectSend,
    kNoFeedback,
  };

  struct RequestRecord
  {
    uint8_t command_id{0};
    std::vector<uint8_t> payload;
  };

  FakeMechanismTransportProvider()
  : Node("fake_mechanism_transport_provider")
  {
    feedback_pub_ = create_publisher<rc26_interfaces::msg::MechanismTransportFeedback>(
      "/mechanism/transport/feedback", rclcpp::QoS(32).reliable());
    send_command_srv_ = create_service<rc26_interfaces::srv::SendMechanismTransportCommand>(
      "/mechanism/transport/send_command",
        std::bind(&FakeMechanismTransportProvider::handleSendCommand, this, std::placeholders::_1,
        std::placeholders::_2));
  }

  ~FakeMechanismTransportProvider() override
  {
    alive_->store(false, std::memory_order_release);
  }

  void setMode(Mode mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
    requests_.clear();
    next_seq_ = 1;
  }

  std::vector<RequestRecord> requests() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }

private:
  void handleSendCommand(
    const std::shared_ptr<rc26_interfaces::srv::SendMechanismTransportCommand::Request> request,
    std::shared_ptr<rc26_interfaces::srv::SendMechanismTransportCommand::Response> response)
  {
    Mode mode;
    uint8_t seq = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mode = mode_;
      requests_.push_back(RequestRecord{request->command_id, request->payload});
      seq = next_seq_++;
    }

    if (mode == Mode::kRejectSend) {
      response->accepted = false;
      response->seq = 0;
      return;
    }

    response->accepted = true;
    response->seq = seq;

    if (mode == Mode::kNoFeedback) {
      return;
    }

    const uint8_t feedback_id =
      mode == Mode::kFrontTrackDownSuccess ?
      static_cast<uint8_t>(rc26_serial::FeedbackID::FRONT_TRACK_DOWN_DONE) :
      static_cast<uint8_t>(rc26_serial::FeedbackID::FRONT_TRACK_UP_DONE);

    auto feedback_pub = feedback_pub_;
    auto alive = alive_;
    std::thread([feedback_pub, alive, seq, feedback_id]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (!alive->load(std::memory_order_acquire)) {
        return;
      }
      rc26_interfaces::msg::MechanismTransportFeedback feedback;
      feedback.seq = seq;
      feedback.feedback_id = feedback_id;
      feedback_pub->publish(feedback);
    }).detach();
  }

  mutable std::mutex mutex_;
  Mode mode_{Mode::kFrontTrackUpSuccess};
  uint8_t next_seq_{1};
  std::vector<RequestRecord> requests_;
  std::shared_ptr<std::atomic<bool>> alive_{std::make_shared<std::atomic<bool>>(true)};
  rclcpp::Publisher<rc26_interfaces::msg::MechanismTransportFeedback>::SharedPtr feedback_pub_;
  rclcpp::Service<rc26_interfaces::srv::SendMechanismTransportCommand>::SharedPtr send_command_srv_;
};

class SharedSerialTransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    transport_provider_ = std::make_shared<FakeMechanismTransportProvider>();

    rclcpp::NodeOptions options;
    options.append_parameter_override("hal_type", "shared_serial");
    mechanism_server_ = std::make_shared<MechanismLifecycleServer>(options);
    action_client_node_ = std::make_shared<rclcpp::Node>("shared_serial_transport_test_client");
    action_client_ =
      rclcpp_action::create_client<ExecuteMechanism>(action_client_node_, "/mechanism/execute");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2U);
    executor_->add_node(transport_provider_);
    executor_->add_node(action_client_node_);
    executor_->add_node(mechanism_server_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    ASSERT_EQ(mechanism_server_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(mechanism_server_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_TRUE(action_client_->wait_for_action_server(std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    if (mechanism_server_) {
      (void)mechanism_server_->on_deactivate(rclcpp_lifecycle::State());
      (void)mechanism_server_->on_cleanup(rclcpp_lifecycle::State());
    }

    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    action_client_.reset();
    action_client_node_.reset();
    mechanism_server_.reset();
    transport_provider_.reset();
    executor_.reset();
  }

  GoalHandle::WrappedResult sendGoal(uint8_t command_id, float timeout_sec)
  {
    ExecuteMechanism::Goal goal;
    goal.command_id = command_id;
    goal.timeout_sec = timeout_sec;

    std::promise<GoalHandle::WrappedResult> result_promise;
    auto result_future = result_promise.get_future();

    rclcpp_action::Client<ExecuteMechanism>::SendGoalOptions options;
    options.result_callback =
      [&result_promise](const GoalHandle::WrappedResult & result) mutable {
        result_promise.set_value(result);
      };

    auto goal_handle_future = action_client_->async_send_goal(goal, options);
    EXPECT_EQ(goal_handle_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto goal_handle = goal_handle_future.get();
    EXPECT_NE(goal_handle, nullptr);

    EXPECT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    return result_future.get();
  }

  std::shared_ptr<FakeMechanismTransportProvider> transport_provider_;
  std::shared_ptr<MechanismLifecycleServer> mechanism_server_;
  std::shared_ptr<rclcpp::Node> action_client_node_;
  rclcpp_action::Client<ExecuteMechanism>::SharedPtr action_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(SharedSerialTransportTest, FrontTrackUpSucceedsViaTransportBridge)
{
  transport_provider_->setMode(FakeMechanismTransportProvider::Mode::kFrontTrackUpSuccess);

  const auto result = sendGoal(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_UP), 1.0F);

  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->error_code, 0U);

  const auto requests = transport_provider_->requests();
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().command_id, static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_UP));
}

TEST_F(SharedSerialTransportTest, FrontTrackDownSucceedsViaTransportBridge)
{
  transport_provider_->setMode(FakeMechanismTransportProvider::Mode::kFrontTrackDownSuccess);

  const auto result = sendGoal(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_DOWN), 1.0F);

  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->error_code, 0U);

  const auto requests = transport_provider_->requests();
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().command_id, static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_DOWN));
}

TEST_F(SharedSerialTransportTest, RejectedTransportSendAbortsGoal)
{
  transport_provider_->setMode(FakeMechanismTransportProvider::Mode::kRejectSend);

  const auto result = sendGoal(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_UP), 1.0F);

  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(result.result->success);
  EXPECT_EQ(result.result->error_code, 3U);
}

TEST_F(SharedSerialTransportTest, MissingTransportFeedbackTimesOut)
{
  transport_provider_->setMode(FakeMechanismTransportProvider::Mode::kNoFeedback);

  const auto result = sendGoal(static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_DOWN), 0.2F);

  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(result.result->success);
  EXPECT_EQ(result.result->error_code, 0xFFU);
}

}  // namespace
}  // namespace rc26_mechanism
