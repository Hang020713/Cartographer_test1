/* rs485_coordinator.cpp - Coordinates sequential polling of RS-485 devices
 *
 * This node orchestrates polling of BMS and level transmitter in sequence
 * to prevent bus conflicts.
 *
 * Parameters:
 *   bms_poll_interval_ms    (int) default 2000
 *   level_poll_interval_ms  (int) default 2000
 *   bus_acquire_timeout_ms  (int) default 500
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class Rs485Coordinator : public rclcpp::Node
{
public:
    Rs485Coordinator() : Node("rs485_coordinator")
    {
        bms_interval_ms_   = declare_parameter<int>("bms_poll_interval_ms", 2000);
        level_interval_ms_ = declare_parameter<int>("level_poll_interval_ms", 2000);
        acquire_timeout_ms_ = declare_parameter<int>("bus_acquire_timeout_ms", 500);

        // Service clients to control bus access
        request_client_ = create_client<std_srvs::srv::Trigger>(
            "/rs485_bus_manager/request_bus");
        release_client_ = create_client<std_srvs::srv::Trigger>(
            "/rs485_bus_manager/release_bus");

        // Service clients to trigger device reads
        bms_read_client_ = create_client<std_srvs::srv::Trigger>(
            "/bms485_node/trigger_read");
        level_read_client_ = create_client<std_srvs::srv::Trigger>(
            "/level_transmitter_node/trigger_read");

        // Wait for services
        RCLCPP_INFO(get_logger(), "Waiting for bus manager and device services...");
        if (!request_client_->wait_for_service(5s) ||
            !release_client_->wait_for_service(5s)) {
            RCLCPP_FATAL(get_logger(), "Bus manager services not available");
            rclcpp::shutdown();
            return;
        }

        if (!bms_read_client_->wait_for_service(5s) ||
            !level_read_client_->wait_for_service(5s)) {
            RCLCPP_FATAL(get_logger(), "Device trigger services not available");
            rclcpp::shutdown();
            return;
        }

        // Start coordination loop
        timer_ = create_wall_timer(
            100ms, std::bind(&Rs485Coordinator::coordination_loop, this));

        RCLCPP_INFO(get_logger(),
                    "Coordinator started (BMS: %d ms, Level: %d ms intervals)",
                    bms_interval_ms_, level_interval_ms_);
    }

private:
    enum class State { IDLE, BMS_ACQUIRE, BMS_WAIT_ACQUIRE, BMS_READ, BMS_WAIT_READ,
                       BMS_RELEASE, LEVEL_ACQUIRE, LEVEL_WAIT_ACQUIRE, LEVEL_READ,
                       LEVEL_WAIT_READ, LEVEL_RELEASE };

    void coordination_loop()
    {
        auto now = std::chrono::steady_clock::now();

        switch (state_) {
            case State::IDLE:
                // Check if it's time to poll BMS
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_bms_read_).count() >= bms_interval_ms_) {
                    state_ = State::BMS_ACQUIRE;
                    RCLCPP_DEBUG(get_logger(), "Starting BMS poll cycle");
                }
                // Check if it's time to poll level transmitter
                else if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_level_read_).count() >= level_interval_ms_) {
                    state_ = State::LEVEL_ACQUIRE;
                    RCLCPP_DEBUG(get_logger(), "Starting level poll cycle");
                }
                break;

            case State::BMS_ACQUIRE:
                acquire_bus_async();
                state_ = State::BMS_WAIT_ACQUIRE;
                operation_start_time_ = now;
                break;

            case State::BMS_WAIT_ACQUIRE:
                if (pending_future_.valid() &&
                    pending_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    auto response = pending_future_.get();
                    if (response->success) {
                        state_ = State::BMS_READ;
                    } else {
                        RCLCPP_WARN(get_logger(), "Failed to acquire bus for BMS");
                        state_ = State::IDLE;
                    }
                } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - operation_start_time_).count() > acquire_timeout_ms_) {
                    RCLCPP_WARN(get_logger(), "Bus acquire timeout for BMS");
                    state_ = State::IDLE;
                }
                break;

            case State::BMS_READ:
                trigger_read_async(bms_read_client_);
                state_ = State::BMS_WAIT_READ;
                operation_start_time_ = now;
                break;

            case State::BMS_WAIT_READ:
                if (pending_future_.valid() &&
                    pending_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    auto response = pending_future_.get();
                    if (response->success) {
                        last_bms_read_ = now;
                    } else {
                        RCLCPP_WARN(get_logger(), "BMS read failed");
                    }
                    state_ = State::BMS_RELEASE;
                } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - operation_start_time_).count() > 2000) {
                    RCLCPP_WARN(get_logger(), "BMS read timeout");
                    state_ = State::BMS_RELEASE;
                }
                break;

            case State::BMS_RELEASE:
                release_bus_async();
                state_ = State::IDLE;
                break;

            case State::LEVEL_ACQUIRE:
                acquire_bus_async();
                state_ = State::LEVEL_WAIT_ACQUIRE;
                operation_start_time_ = now;
                break;

            case State::LEVEL_WAIT_ACQUIRE:
                if (pending_future_.valid() &&
                    pending_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    auto response = pending_future_.get();
                    if (response->success) {
                        state_ = State::LEVEL_READ;
                    } else {
                        RCLCPP_WARN(get_logger(), "Failed to acquire bus for level transmitter");
                        state_ = State::IDLE;
                    }
                } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - operation_start_time_).count() > acquire_timeout_ms_) {
                    RCLCPP_WARN(get_logger(), "Bus acquire timeout for level transmitter");
                    state_ = State::IDLE;
                }
                break;

            case State::LEVEL_READ:
                trigger_read_async(level_read_client_);
                state_ = State::LEVEL_WAIT_READ;
                operation_start_time_ = now;
                break;

            case State::LEVEL_WAIT_READ:
                if (pending_future_.valid() &&
                    pending_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    auto response = pending_future_.get();
                    if (response->success) {
                        last_level_read_ = now;
                    } else {
                        RCLCPP_WARN(get_logger(), "Level transmitter read failed");
                    }
                    state_ = State::LEVEL_RELEASE;
                } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - operation_start_time_).count() > 2000) {
                    RCLCPP_WARN(get_logger(), "Level read timeout");
                    state_ = State::LEVEL_RELEASE;
                }
                break;

            case State::LEVEL_RELEASE:
                release_bus_async();
                state_ = State::IDLE;
                break;
        }
    }

    void acquire_bus_async()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        pending_future_ = request_client_->async_send_request(request);
    }

    void release_bus_async()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        release_client_->async_send_request(request);
    }

    void trigger_read_async(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client)
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        pending_future_ = client->async_send_request(request);
    }

    State state_ = State::IDLE;
    int bms_interval_ms_;
    int level_interval_ms_;
    int acquire_timeout_ms_;

    std::chrono::steady_clock::time_point last_bms_read_;
    std::chrono::steady_clock::time_point last_level_read_;
    std::chrono::steady_clock::time_point operation_start_time_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr request_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bms_read_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr level_read_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> pending_future_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rs485Coordinator>());
    rclcpp::shutdown();
    return 0;
}