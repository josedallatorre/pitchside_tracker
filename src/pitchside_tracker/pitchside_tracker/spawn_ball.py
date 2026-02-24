#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from pitchside_tracker.utils.yaml_loader import load_yaml

class BallSpawner(Node):
    def __init__(self):
        super().__init__('ball_spawner')
        # --- Load YAML config ---
        self.config = load_yaml()

        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/spawn_entity...')

        self.spawn_ball()

    def spawn_ball(self):
        req = SpawnEntity.Request()
        req.name = "football"
        req.xml = open('/ros2_ws/src/sjtu_drone_description/models/MSL_models15/RoboCup15_MSL_Football/model.sdf').read()
        drone_config = self.config.get("drone")
        pos = drone_config["position"]
        req.robot_namespace = ""
        req.initial_pose.position.x = pos[0] 
        req.initial_pose.position.y = pos[1]
        req.initial_pose.position.z = pos[2]

        self.cli.call_async(req)
        self.get_logger().info("Football spawned in Gazebo")


def main(args=None):
    rclpy.init(args=args)
    node = BallSpawner()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()
