#include "g1_bridge/managed_bridge.hpp"

#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;

void request_shutdown(int) noexcept
{
  // Signal handlers only set this flag. ROS and Gazebo teardown stays on the
  // executor thread where resource destruction is deterministic.
  shutdown_requested = 1;
}

}  // namespace

int main(int argc, char * argv[])
{
  std::shared_ptr<g1_bridge::ManagedBridge> bridge;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  bool ros_initialized = false;
  bool node_added = false;
  int return_code = 1;

  try {
    rclcpp::init(
      argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    ros_initialized = true;

    std::signal(SIGINT, request_shutdown);
#ifdef SIGTERM
    std::signal(SIGTERM, request_shutdown);
#endif

    // Construct the executor only after rclcpp::init: its guard condition
    // requires an initialized ROS context.
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    bridge = std::make_shared<g1_bridge::ManagedBridge>(rclcpp::NodeOptions());
    bridge->load_config();

    executor->add_node(bridge);
    node_added = true;
    while (rclcpp::ok() && shutdown_requested == 0) {
      executor->spin_some(std::chrono::milliseconds(100));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return_code = 0;
  } catch (const std::exception & error) {
    std::cerr << "managed_bridge failed: " << error.what() << std::endl;
  } catch (...) {
    std::cerr << "managed_bridge failed with an unknown exception" << std::endl;
  }

  // Best-effort cleanup also runs for constructor/configuration failures.
  if (node_added && bridge && executor) {
    try {
      executor->remove_node(bridge);
    } catch (const std::exception & error) {
      std::cerr << "managed_bridge remove_node failed: " << error.what() << std::endl;
      return_code = 1;
    } catch (...) {
      std::cerr << "managed_bridge remove_node failed with an unknown exception"
                << std::endl;
      return_code = 1;
    }
  }
  if (bridge) {
    if (!bridge->stop()) {
      std::cerr << "managed_bridge cleanup failed" << std::endl;
      return_code = 1;
    }
    bridge.reset();
  }
  executor.reset();
  if (ros_initialized) {
    try {
      rclcpp::shutdown();
    } catch (const std::exception & error) {
      std::cerr << "managed_bridge ROS shutdown failed: " << error.what()
                << std::endl;
      return_code = 1;
    } catch (...) {
      std::cerr << "managed_bridge ROS shutdown failed" << std::endl;
      return_code = 1;
    }
  }

  return return_code;
}
