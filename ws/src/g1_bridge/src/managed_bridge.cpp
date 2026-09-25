#include "g1_bridge/managed_bridge.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace g1_bridge
{

ManagedBridge::ManagedBridge(const rclcpp::NodeOptions & options)
: ros_gz_bridge::RosGzBridge(options)
{
  // Keep the upstream heartbeat alive so lazy bridge handles continue to be
  // maintained by the single-threaded executor. It is stopped explicitly in
  // stop(), after the executor has removed this node.
  validate_config_file();
}

void ManagedBridge::validate_config_file() const
{
  std::string config_file;
  if (!get_parameter("config_file", config_file) || config_file.empty()) {
    throw std::runtime_error(
      "g1_bridge requires a non-empty ROS parameter: config_file");
  }

  std::error_code error;
  const std::filesystem::path path(config_file);
  if (!std::filesystem::is_regular_file(path, error) || error) {
    throw std::runtime_error(
      "g1_bridge config_file is not a readable regular file: " + config_file);
  }

  std::ifstream stream(path);
  if (!stream.good()) {
    throw std::runtime_error(
      "g1_bridge cannot read config_file: " + config_file);
  }

}

void ManagedBridge::load_config()
{
  if (stopped_) {
    throw std::runtime_error("g1_bridge cannot load config after stop()");
  }
  if (config_loaded_) {
    return;
  }

  // Validate again in case the path was removed or replaced between node
  // construction and the post-make_shared() load point.
  validate_config_file();

  // RosGzBridge::spin() calls shared_from_this() while creating handles, so
  // this must run after std::make_shared<ManagedBridge>() returns.
  spin();
  if (handles_.empty() && services_.empty()) {
    throw std::runtime_error(
      "g1_bridge config_file did not create any bridge or service");
  }
  config_loaded_ = true;
}

bool ManagedBridge::stop() noexcept
{
  if (stopped_) {
    return true;
  }

  bool cleanup_ok = true;
  heartbeat_timer_.reset();

  // BridgeHandleGzToRos has a default destructor and does not expose an
  // explicit subscriber stop. Remove every Gazebo subscription while the
  // transport node and ROS context are still valid, before destroying the
  // opaque BridgeHandle objects.
  if (gz_node_) {
    try {
      const auto topics = gz_node_->SubscribedTopics();
      for (const auto & topic : topics) {
        try {
          if (!gz_node_->Unsubscribe(topic)) {
            std::cerr << "managed_bridge could not unsubscribe Gazebo topic: "
                      << topic << std::endl;
            cleanup_ok = false;
          }
        } catch (const std::exception & error) {
          std::cerr << "managed_bridge unsubscribe failed for Gazebo topic "
                    << topic << ": " << error.what() << std::endl;
          cleanup_ok = false;
        } catch (...) {
          std::cerr << "managed_bridge unsubscribe failed for Gazebo topic "
                    << topic << std::endl;
          cleanup_ok = false;
        }
      }
    } catch (const std::exception & error) {
      std::cerr << "managed_bridge could not enumerate Gazebo subscriptions: "
                << error.what() << std::endl;
      cleanup_ok = false;
    } catch (...) {
      std::cerr << "managed_bridge could not enumerate Gazebo subscriptions"
                << std::endl;
      cleanup_ok = false;
    }
  }

  // Keep this order explicit: release bridge handles/services while the
  // Gazebo node is still available, and finally release the Gazebo node.
  handles_.clear();
  services_.clear();
  gz_node_.reset();
  stopped_ = true;
  return cleanup_ok;
}

}  // namespace g1_bridge
