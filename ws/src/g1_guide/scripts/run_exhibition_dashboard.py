#!/usr/bin/env python3
"""Start the Gazebo exhibition and live ROS sensor dashboard as one session.

Run inside sourced WSL ROS environment. Ctrl-C in this terminal, or q in the
dashboard window, stops only the two process groups created by this script.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time

from g1_guide.lowlevel_metrics import live_group_members


def _stop_owned(process: subprocess.Popen[bytes], label: str) -> bool:
    if process.poll() is None:
        try:
            # ROS launch/run wrappers forward SIGINT to their children. Sending
            # it to the whole group as well delivers a duplicate interrupt and
            # can abort rclpy while it is destroying a node.
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            print(f"{label} did not exit on SIGINT; sending SIGTERM to its own group", file=sys.stderr)
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                print(f"{label} needs manual inspection (PGID {process.pid}); not using SIGKILL", file=sys.stderr)
                return False
    remaining = live_group_members(process.pid)
    if remaining:
        print(f"{label} left owned group members {remaining}; sending SIGTERM", file=sys.stderr)
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        until = time.monotonic() + 5
        while remaining and time.monotonic() < until:
            time.sleep(0.1)
            remaining = live_group_members(process.pid)
    if remaining:
        print(f"{label} still has owned PIDs {remaining}; inspect manually", file=sys.stderr)
        return False
    return process.returncode in (0, -signal.SIGINT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gazebo-gui", action="store_true", help="also open the Gazebo 3D UI")
    parser.add_argument("--scene-config", help="optional exhibition JSON path")
    parser.add_argument("--duration", type=float,
                        help="automatically stop this session after N wall-clock seconds (for demos/tests)")
    args = parser.parse_args()
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be finite and positive")
    if not os.environ.get("AMENT_PREFIX_PATH"):
        parser.error("Source /opt/ros/lyrical/setup.bash and the built G1 workspace first")

    command = ["ros2", "launch", "g1_guide", "exhibition.launch.py",
               f"headless:={'false' if args.gazebo_gui else 'true'}"]
    if args.scene_config:
        command.append(f"scene_config:={args.scene_config}")

    launch = None
    dashboard = None
    interrupted = False
    exit_code = 0
    shutdown_ok = True
    started = time.monotonic()
    try:
        launch = subprocess.Popen(command, start_new_session=True)
        dashboard = subprocess.Popen(["ros2", "run", "g1_guide", "sensor_dashboard"],
                                     start_new_session=True)
        print("Gazebo + sensor dashboard starting. Press q in the dashboard or Ctrl-C here to stop.", flush=True)
        while True:
            if args.duration is not None and time.monotonic() - started >= args.duration:
                print("Configured session duration reached; stopping.", flush=True)
                break
            if launch.poll() is not None:
                print(f"Gazebo launch exited with code {launch.returncode}", file=sys.stderr)
                exit_code = 1 if launch.returncode else 0
                break
            if dashboard.poll() is not None:
                print(f"Dashboard exited with code {dashboard.returncode}", flush=True)
                exit_code = 1 if dashboard.returncode else 0
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if dashboard is not None:
            shutdown_ok = _stop_owned(dashboard, "dashboard") and shutdown_ok
        if launch is not None:
            shutdown_ok = _stop_owned(launch, "Gazebo launch") and shutdown_ok
        if not shutdown_ok:
            print("One or more owned processes did not shut down cleanly; inspect the PIDs above.", file=sys.stderr)
        if interrupted:
            print("Session stopped.", flush=True)
    return exit_code if shutdown_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
