#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rc26_interfaces/srv/set_keepout_runtime.hpp"

namespace rc26_kfs_keepout {

namespace {

using LoadNode = composition_interfaces::srv::LoadNode;
using UnloadNode = composition_interfaces::srv::UnloadNode;
using SetKeepoutRuntime = rc26_interfaces::srv::SetKeepoutRuntime;

constexpr std::chrono::milliseconds kServiceWaitTimeout{3000};

std::string joinServiceName(const std::string& left, const std::string& right) {
    if (left.empty()) {
        return right;
    }
    if (left.back() == '/') {
        return left + right;
    }
    return left + "/" + right;
}

}  // namespace

class KfsKeepoutRuntimeManager : public rclcpp::Node {
public:
    KfsKeepoutRuntimeManager()
        : rclcpp::Node("kfs_keepout_runtime_manager") {
        this->declare_parameter<std::string>("runtime_service_name", "/kfs_keepout/set_runtime");
        this->declare_parameter<std::string>("component_container_name", "kfs_keepout_container");
        this->declare_parameter<std::string>("component_node_name", "kfs_block_fuser");
        this->declare_parameter<std::string>("component_runtime_control_service", "set_runtime");
        this->declare_parameter<std::string>("kfs_state_topic", "mf_kfs_state");
        this->declare_parameter<std::string>("mask_topic", "/kfs_filter_mask");
        this->declare_parameter<std::string>("heartbeat_topic", "/kfs_keepout_heartbeat");
        this->declare_parameter<std::string>("grid_layout_file", "");
        this->declare_parameter<std::string>("diagnostics_topic", "diagnostics");
        this->declare_parameter<double>("min_confidence", 0.60);
        this->declare_parameter<double>("inflate_radius_m", 0.60);
        this->declare_parameter<double>("map_resolution", 0.10);
        this->declare_parameter<std::string>("keepout_shape", "square");
        this->declare_parameter<double>("block_half_size_m", 0.60);
        this->declare_parameter<double>("keepout_margin_m", 0.03);
        this->declare_parameter<double>("block_thresh", 0.70);
        this->declare_parameter<double>("free_thresh", 0.35);
        this->declare_parameter<double>("lo_hit", 1.099);
        this->declare_parameter<double>("lo_hit_block", 1.099);
        this->declare_parameter<double>("lo_hit_fake", 0.693);
        this->declare_parameter<double>("lo_miss", -0.693);
        this->declare_parameter<double>("decay_rate", 2.0);
        this->declare_parameter<double>("decay_target_prob", 0.05);
        this->declare_parameter<double>("ttl_sec", 10.0);
        this->declare_parameter<std::string>("ttl_mode", "hard");
        this->declare_parameter<int>("dwell_cycles", 3);
        this->declare_parameter<double>("grid_spacing_tolerance_m", 0.05);
        this->declare_parameter<std::vector<int64_t>>("slow_grid_ids", std::vector<int64_t>{});
        this->declare_parameter<bool>("use_sim_time", false);

        runtime_service_name_ = this->get_parameter("runtime_service_name").as_string();
        container_name_ = this->get_parameter("component_container_name").as_string();
        component_node_name_ = this->get_parameter("component_node_name").as_string();
        component_runtime_control_service_ =
            this->get_parameter("component_runtime_control_service").as_string();

        service_callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        const auto load_service = joinServiceName(container_name_, "_container/load_node");
        const auto unload_service = joinServiceName(container_name_, "_container/unload_node");
        const auto component_control_service =
            joinServiceName(component_node_name_, component_runtime_control_service_);

        load_client_ = this->create_client<LoadNode>(
            load_service, rmw_qos_profile_services_default, client_callback_group_);
        unload_client_ = this->create_client<UnloadNode>(
            unload_service, rmw_qos_profile_services_default, client_callback_group_);
        component_runtime_client_ = this->create_client<SetKeepoutRuntime>(
            component_control_service, rmw_qos_profile_services_default, client_callback_group_);

        runtime_service_ = this->create_service<SetKeepoutRuntime>(
            runtime_service_name_,
            std::bind(&KfsKeepoutRuntimeManager::handleRuntimeRequest, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_callback_group_);

        RCLCPP_INFO(this->get_logger(),
                    "kfs keepout runtime manager ready: service=%s container=%s component=%s",
                    runtime_service_name_.c_str(),
                    container_name_.c_str(),
                    component_node_name_.c_str());
    }

private:
    enum class RuntimeState : uint8_t {
        UNLOADED = 0,
        LOADING = 1,
        ACTIVE = 2,
        CLEARING = 3,
        ERROR = 4,
    };

    void handleRuntimeRequest(
        const SetKeepoutRuntime::Request::SharedPtr request,
        SetKeepoutRuntime::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mu_);
        if (!request || !response) {
            return;
        }

        if (request->activate) {
            handleActivate(*request, *response);
        } else {
            handleDeactivate(*request, *response);
        }
    }

    void handleActivate(
        const SetKeepoutRuntime::Request& request,
        SetKeepoutRuntime::Response& response) {
        response.outputs_cleared = false;

        if (component_loaded_ && active_) {
            response.ok = true;
            response.active = true;
            response.component_loaded = true;
            response.status = stateLabel(state_);
            response.message = "keepout already active";
            return;
        }

        if (!component_loaded_) {
            state_ = RuntimeState::LOADING;
            auto load_response = loadComponent();
            if (!load_response.first) {
                state_ = RuntimeState::ERROR;
                active_ = false;
                response.ok = false;
                response.active = false;
                response.component_loaded = false;
                response.status = stateLabel(state_);
                response.message = load_response.second;
                return;
            }
            component_loaded_ = true;
        }

        auto activate_response = callComponentRuntime(true, request.reason);
        if (!activate_response.first || !activate_response.second->ok ||
            !activate_response.second->active) {
            state_ = RuntimeState::ERROR;
            active_ = false;
            response.ok = false;
            response.active = false;
            response.component_loaded = component_loaded_;
            response.outputs_cleared =
                activate_response.second ? activate_response.second->outputs_cleared : false;
            response.status = stateLabel(state_);
            response.message = activate_response.first
                                   ? activate_response.second->message
                                   : "keepout component activation service failed";
            return;
        }

        active_ = true;
        state_ = RuntimeState::ACTIVE;
        response.ok = true;
        response.active = true;
        response.component_loaded = component_loaded_;
        response.outputs_cleared = false;
        response.status = stateLabel(state_);
        response.message = activate_response.second->message;
    }

    void handleDeactivate(
        const SetKeepoutRuntime::Request& request,
        SetKeepoutRuntime::Response& response) {
        if (!component_loaded_) {
            active_ = false;
            state_ = RuntimeState::UNLOADED;
            response.ok = true;
            response.active = false;
            response.outputs_cleared = true;
            response.component_loaded = false;
            response.status = stateLabel(state_);
            response.message = "keepout already unloaded";
            return;
        }

        state_ = RuntimeState::CLEARING;
        auto deactivate_response = callComponentRuntime(false, request.reason);
        if (!deactivate_response.first || !deactivate_response.second->outputs_cleared) {
            state_ = RuntimeState::ERROR;
            active_ = false;
            response.ok = false;
            response.active = false;
            response.outputs_cleared = false;
            response.component_loaded = component_loaded_;
            response.status = stateLabel(state_);
            response.message = deactivate_response.first && deactivate_response.second
                                   ? deactivate_response.second->message
                                   : "keepout component clear/deactivate service failed";
            return;
        }

        active_ = false;
        response.outputs_cleared = true;

        auto unload_response = unloadComponent();
        if (!unload_response.first) {
            state_ = RuntimeState::ERROR;
            response.ok = true;
            response.active = false;
            response.component_loaded = true;
            response.status = stateLabel(state_);
            response.message = "keepout outputs cleared but unload failed: " + unload_response.second;
            return;
        }

        component_loaded_ = false;
        loaded_component_id_ = 0;
        state_ = RuntimeState::UNLOADED;
        response.ok = true;
        response.active = false;
        response.component_loaded = false;
        response.status = stateLabel(state_);
        response.message = deactivate_response.second->message;
    }

    std::pair<bool, std::string> loadComponent() {
        if (!waitForService<LoadNode>(load_client_, "container load service")) {
            return {false, "container load service unavailable"};
        }

        auto request = std::make_shared<LoadNode::Request>();
        request->package_name = "rc26_kfs_keepout";
        request->plugin_name = "rc26_kfs_keepout::KfsBlockFuser";
        request->node_name = component_node_name_;
        request->node_namespace = this->get_namespace();
        request->parameters = buildComponentParameters();

        rcl_interfaces::msg::Parameter intra_process;
        intra_process.name = "use_intra_process_comms";
        intra_process.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        intra_process.value.bool_value = true;
        request->extra_arguments.push_back(intra_process);

        auto future = load_client_->async_send_request(request);
        if (future.wait_for(kServiceWaitTimeout) != std::future_status::ready) {
            load_client_->remove_pending_request(future);
            return {false, "timed out waiting for load_node response"};
        }
        const auto response = future.get();
        if (!response || !response->success) {
            return {false, response ? response->error_message : "empty load_node response"};
        }

        loaded_component_id_ = response->unique_id;
        return {true, response->full_node_name};
    }

    std::pair<bool, std::string> unloadComponent() {
        if (loaded_component_id_ == 0) {
            return {false, "loaded component id unknown"};
        }
        if (!waitForService<UnloadNode>(unload_client_, "container unload service")) {
            return {false, "container unload service unavailable"};
        }

        auto request = std::make_shared<UnloadNode::Request>();
        request->unique_id = loaded_component_id_;
        auto future = unload_client_->async_send_request(request);
        if (future.wait_for(kServiceWaitTimeout) != std::future_status::ready) {
            unload_client_->remove_pending_request(future);
            return {false, "timed out waiting for unload_node response"};
        }
        const auto response = future.get();
        if (!response || !response->success) {
            return {false, response ? response->error_message : "empty unload_node response"};
        }

        return {true, "component unloaded"};
    }

    std::pair<bool, SetKeepoutRuntime::Response::SharedPtr> callComponentRuntime(
        const bool activate, const std::string& reason) {
        if (!waitForService<SetKeepoutRuntime>(
                component_runtime_client_, "component runtime control service")) {
            return {false, nullptr};
        }

        auto request = std::make_shared<SetKeepoutRuntime::Request>();
        request->activate = activate;
        request->reason = reason;
        auto future = component_runtime_client_->async_send_request(request);
        if (future.wait_for(kServiceWaitTimeout) != std::future_status::ready) {
            component_runtime_client_->remove_pending_request(future);
            return {false, nullptr};
        }
        return {true, future.get()};
    }

    std::vector<rcl_interfaces::msg::Parameter> buildComponentParameters() {
        std::vector<rcl_interfaces::msg::Parameter> params;
        params.reserve(24);
        const auto push = [&params](const rclcpp::Parameter& param) {
            params.push_back(param.to_parameter_msg());
        };

        push(this->get_parameter("use_sim_time"));
        push(this->get_parameter("kfs_state_topic"));
        push(this->get_parameter("mask_topic"));
        push(this->get_parameter("heartbeat_topic"));
        push(this->get_parameter("grid_layout_file"));
        push(this->get_parameter("diagnostics_topic"));
        push(this->get_parameter("min_confidence"));
        push(this->get_parameter("inflate_radius_m"));
        push(this->get_parameter("map_resolution"));
        push(this->get_parameter("keepout_shape"));
        push(this->get_parameter("block_half_size_m"));
        push(this->get_parameter("keepout_margin_m"));
        push(this->get_parameter("block_thresh"));
        push(this->get_parameter("free_thresh"));
        push(this->get_parameter("lo_hit"));
        push(this->get_parameter("lo_hit_block"));
        push(this->get_parameter("lo_hit_fake"));
        push(this->get_parameter("lo_miss"));
        push(this->get_parameter("decay_rate"));
        push(this->get_parameter("decay_target_prob"));
        push(this->get_parameter("ttl_sec"));
        push(this->get_parameter("ttl_mode"));
        push(this->get_parameter("dwell_cycles"));
        push(this->get_parameter("grid_spacing_tolerance_m"));
        push(this->get_parameter("slow_grid_ids"));
        push(rclcpp::Parameter("runtime_control_service", component_runtime_control_service_));
        return params;
    }

    template <typename ServiceT>
    bool waitForService(
        const typename rclcpp::Client<ServiceT>::SharedPtr& client,
        const std::string& label) {
        if (!client) {
            RCLCPP_ERROR(this->get_logger(), "%s client is null", label.c_str());
            return false;
        }
        if (!client->wait_for_service(kServiceWaitTimeout)) {
            RCLCPP_ERROR(this->get_logger(), "%s unavailable", label.c_str());
            return false;
        }
        return true;
    }

    static std::string stateLabel(const RuntimeState state) {
        switch (state) {
            case RuntimeState::UNLOADED:
                return "UNLOADED";
            case RuntimeState::LOADING:
                return "LOADING";
            case RuntimeState::ACTIVE:
                return "ACTIVE";
            case RuntimeState::CLEARING:
                return "CLEARING";
            case RuntimeState::ERROR:
                return "ERROR";
        }
        return "ERROR";
    }

    std::mutex mu_;
    RuntimeState state_{RuntimeState::UNLOADED};
    bool component_loaded_{false};
    bool active_{false};
    uint64_t loaded_component_id_{0};

    std::string runtime_service_name_;
    std::string container_name_;
    std::string component_node_name_;
    std::string component_runtime_control_service_;

    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    rclcpp::Service<SetKeepoutRuntime>::SharedPtr runtime_service_;
    rclcpp::Client<LoadNode>::SharedPtr load_client_;
    rclcpp::Client<UnloadNode>::SharedPtr unload_client_;
    rclcpp::Client<SetKeepoutRuntime>::SharedPtr component_runtime_client_;
};

}  // namespace rc26_kfs_keepout

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_kfs_keepout::KfsKeepoutRuntimeManager>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
