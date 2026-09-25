#ifndef G1_BRIDGE__MANAGED_BRIDGE_HPP_
#define G1_BRIDGE__MANAGED_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

namespace g1_bridge
{

/// RosGzBridge with an explicit, deterministic shutdown sequence.
///
/// The upstream heartbeat timer remains active for lazy bridges. This wrapper
/// uses the upstream bridge implementation, loads the project's YAML once
/// after make_shared() has completed, and owns the shutdown order.
class ManagedBridge final : public ros_gz_bridge::RosGzBridge
{
public:
  explicit ManagedBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Load the required config_file once, after the object is shared-owned.
  void load_config();

  /// Stop all ROS/Gazebo bridge resources before the node is destroyed.
  /// Returns false when a Gazebo subscription could not be removed.
  bool stop() noexcept;

private:
  void validate_config_file() const;

  bool config_loaded_{false};
  bool stopped_{false};
};

}  // namespace g1_bridge

#endif  // G1_BRIDGE__MANAGED_BRIDGE_HPP_
