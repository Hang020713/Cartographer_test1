/* rs485_bus_manager.cpp - Manages exclusive access to a shared RS-485 bus
 *
 * Services:
 *   ~/request_bus (std_srvs/srv/Trigger) - Request exclusive bus access
 *   ~/release_bus (std_srvs/srv/Trigger) - Release bus access
 *
 * Parameters:
 *   max_hold_time_ms (int) default 1000 - Maximum time a client can hold the bus
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class Rs485BusManager : public rclcpp::Node
{
public:
    Rs485BusManager() : Node("rs485_bus_manager")
    {
        max_hold_time_ms_ = declare_parameter<int>("max_hold_time_ms", 1000);

        request_service_ = create_service<std_srvs::srv::Trigger>(
            "~/request_bus",
            std::bind(&Rs485BusManager::handle_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        release_service_ = create_service<std_srvs::srv::Trigger>(
            "~/release_bus",
            std::bind(&Rs485BusManager::handle_release, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Watchdog timer to auto-release stale locks
        watchdog_timer_ = create_wall_timer(
            100ms, std::bind(&Rs485BusManager::watchdog_callback, this));

        RCLCPP_INFO(get_logger(), "RS-485 bus manager ready (max_hold=%d ms)",
                    max_hold_time_ms_);
    }

private:
    void handle_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (bus_locked_) {
            response->success = false;
            response->message = "Bus is busy";
            return;
        }

        bus_locked_ = true;
        lock_time_ = std::chrono::steady_clock::now();

        response->success = true;
        response->message = "Bus access granted";
        RCLCPP_DEBUG(get_logger(), "Bus access granted");
    }

    void handle_release(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!bus_locked_) {
            response->success = false;
            response->message = "Bus was not locked";
            return;
        }

        bus_locked_ = false;
        response->success = true;
        response->message = "Bus released";
        RCLCPP_DEBUG(get_logger(), "Bus released");
    }

    void watchdog_callback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!bus_locked_) return;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lock_time_).count();

        if (elapsed > max_hold_time_ms_) {
            RCLCPP_WARN(get_logger(),
                        "Force-releasing bus after %ld ms (exceeded max %d ms)",
                        elapsed, max_hold_time_ms_);
            bus_locked_ = false;
        }
    }

    std::mutex mutex_;
    bool bus_locked_ = false;
    std::chrono::steady_clock::time_point lock_time_;
    int max_hold_time_ms_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rs485BusManager>());
    rclcpp::shutdown();
    return 0;
}