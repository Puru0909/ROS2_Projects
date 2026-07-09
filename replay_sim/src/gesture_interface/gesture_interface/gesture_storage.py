# gesture_storage.py

import os
import yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

class GestureStorage:
    """Handles saving and loading robot gestures inside the gesture_interface package folder in the workspace."""

    def __init__(self, node):
      self.node = node

      from ament_index_python.packages import get_package_share_directory
      package_share = get_package_share_directory('gesture_interface')
      self.base_path = os.path.join(package_share, "gestures")

      os.makedirs(self.base_path, exist_ok=True)
      self.node.get_logger().info(f"Gesture storage directory: {self.base_path}")

    def save(self, name: str, trajectory: JointTrajectory):
        """Save a JointTrajectory as a YAML file with the given name."""
        path = os.path.join(self.base_path, f"{name}.yaml")

        data = {
            "joint_names": trajectory.joint_names,
            "points": [
                {
                    "positions": list(p.positions),  # convert to list for YAML
                    "time_from_start": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                }
                for p in trajectory.points
            ],
        }

        with open(path, "w") as f:
            yaml.dump(data, f)

        self.node.get_logger().info(f"Gesture '{name}' saved to {path}")

    def load(self, name: str) -> JointTrajectory | None:
        """Load a JointTrajectory by name. Returns None if not found."""
        path = os.path.join(self.base_path, f"{name}.yaml")

        if not os.path.exists(path):
            self.node.get_logger().warn(f"Gesture '{name}' not found at {path}")
            return None

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        traj = JointTrajectory()
        traj.joint_names = data["joint_names"]

        for p in data["points"]:
            point = JointTrajectoryPoint()
            point.positions = p["positions"]

            sec = int(p["time_from_start"])
            nanosec = int((p["time_from_start"] % 1.0) * 1e9)

            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec

            traj.points.append(point)

        self.node.get_logger().info(f"Gesture '{name}' loaded from {path}")
        return traj
