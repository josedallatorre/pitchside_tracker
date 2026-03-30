#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from pitchside_tracker.utils.yaml_loader import load_yaml
from pitchside_tracker.utils.quaternion import quaternion_from_euler


class PlayerSpawner(Node):
    def __init__(self):
        super().__init__('player_spawner')
        # Load YAML config
        self.config = load_yaml()

        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/spawn_entity...')

        self.spawn_players()

    def spawn_players(self):
        if not self.config.get("spawn_players", False):
            self.get_logger().info("Player spawning disabled.")
            return
        
        # spawning every player in yaml
        for player in self.config.get("players", []):
            name = player["name"]
            pos = player["position"]

            req = SpawnEntity.Request()
            req.name = name

            # sdf model of the player
            req.xml = open('/ros2_ws/src/sjtu_drone_description/models/standing_person/model.sdf').read()

            req.robot_namespace = ""
            req.initial_pose.position.x = pos[0]
            req.initial_pose.position.y = pos[1]
            req.initial_pose.position.z = pos[2]
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, pos[3])
            self.get_logger().info(f"Quaternion: {qx, qy, qz, qw}")

            req.initial_pose.orientation.x = qx
            req.initial_pose.orientation.y = qy
            req.initial_pose.orientation.z = qz
            req.initial_pose.orientation.w = qw

            self.cli.call_async(req)
            self.get_logger().info(f"Spawned {name} at {pos}")


def main(args=None):
    rclpy.init(args=args)
    node = PlayerSpawner()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()
