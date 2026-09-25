# g1_bridge

`managed_bridge` is a small `ament_cmake` ROS 2 C++ package for the local G1
Gazebo simulation. It derives from the installed `ros_gz_bridge::RosGzBridge`
and reuses its YAML conversion and bridge-handle implementation; it does not
copy or replace message conversion code.

The node keeps the upstream node name `ros_gz_bridge` and accepts the standard
ROS parameter:

```text
--ros-args -p config_file:=/absolute/path/to/bridge.yaml
```

The file is required and must be readable. The upstream heartbeat timer stays
active, so the normal `lazy` parameter and per-entry YAML lazy settings remain
supported. The configuration is loaded once after `std::make_shared` has
completed. On SIGINT or SIGTERM the handler only sets a
`sig_atomic_t` flag; the single-threaded executor notices it and performs
`remove_node`, then `stop` cancels the heartbeat and releases bridge resources,
followed by node reset and ROS shutdown.

This is only a local simulation lifecycle wrapper. It has no real-robot
connection and sends no physical motion commands.
