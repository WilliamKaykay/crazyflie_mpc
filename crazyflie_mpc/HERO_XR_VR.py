#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose,PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from enum import Enum
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import argparse
from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder
import open3d as o3d
from mpl_toolkits.mplot3d import Axes3D
import time
from crazyflie_mpc.PathFinder import PathFinder

class HeroXR(Node):
    def __init__(self, drone_name):
        super().__init__('hero_xr')
        self.get_logger().info('Initializing HeroXR node')
        # Initialize clients and subscriptions for each drone
        self.drone_name = drone_name
        self.drone_position = [0,0,0]
        self.get_logger().info(f'Drone position: {self.drone_position}')

        
        self.drone_clients = {
            'takeoff_client': self.create_client(Takeoff, f'/{drone_name}/takeoff'),
            'land_client': self.create_client(Land, f'/{drone_name}/land'),
            'goto_client': self.create_client(GoTo, f'/{drone_name}/go_to')
        }
        self.create_subscription(Empty, f'/{drone_name}/takeoff', self.takeoff_callback, 10)
        self.create_subscription(Empty, f'/{drone_name}/land', self.land_callback, 10)
        self.create_subscription(Pose, f'/{drone_name}/go_to', self.goto_callback, 10)
        self.create_subscription(PoseStamped, f'/{drone_name}/pose', self.dronePose_callback, 10)
        self.get_logger().info('HeroXR node initialized and ready')


        self.grid_unit = 0.1
        self.resolution = None
        x_size = int(6/self.grid_unit)
        y_size = int(9/self.grid_unit)
        z_size = int(3/self.grid_unit)
        self.grid_size = (x_size, y_size, z_size) 
        self.x_range = [-3, 3]
        self.y_range = [-4.5, 4.5]
        self.z_range = [0, 3]
        self.pathfinder = PathFinder(self.grid_unit, resolution=None, grid_size=self.grid_size)
        self.get_logger().info('Path finder initialized and ready')

         # Load the point cloud
        pcd = self.pathfinder.load_point_cloud("/home/cpsl/CrazySim/ros2_ws/src/crazyflie_mpc/point cloud/Vicon1.ply")
        # Filter out points out of range
        filtered_pcd = self.pathfinder.filter_point_cloud(pcd)
        # Translate to occupancy map
        self.occupancy_map = self.pathfinder.point_cloud_to_numpy(filtered_pcd, x_min=-2.82, y_min=-4.86, z_min=0.0)

    def takeoff_callback(self, msg):
        self.get_logger().info(f'Received takeoff command for {self.drone_name}')
        self.takeoff(self.drone_name, 0.5, 3.0)

    def land_callback(self, msg):
        self.get_logger().info(f'Received land command for {self.drone_name}')
        self.land(self.drone_name, 0.0, 3.0)

    def dronePose_callback(self, msg: PoseStamped):
        #Get drone's pose from Vicon system message 
        current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z]
        # Update pose of the specific drone
        self.drone_position = current_position


    def goto_callback(self, msg: Pose):
        # Print log
        self.get_logger().info(f'Received go_to command for {self.drone_name}')
    
        # Handle Pose message
        goal = [msg.position.x, msg.position.y, msg.position.z]
        self.get_logger().info(f'Read current pose: {self.drone_position}')
        self.get_logger().info(f'goal: {goal}')

        # Find the path and send waypoints 1 by 1
        waypoints = self.pathfinder.find_path(self.occupancy_map, self.drone_position, goal)
        self.get_logger().info(f'Waypoints: {waypoints}')

        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f'Sending waypoint {i + 1}/{len(waypoints)} to {self.drone_name}: {waypoint}')
                
                
            # Wait until drone get to waypoint
            if not self.is_at_position(self.drone_position, waypoint):
                if self.is_at_position(self.drone_position, fwaypoint):
                    break
                self.go_to(self.drone_name, waypoint, yaw=0.0, duration=1.0, relative=False)
                time.sleep(0.5)


    def takeoff(self, drone_name, target_height, duration):
        req = Takeoff.Request()
        req.height = target_height
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.group_mask = 0
        self.get_logger().info(f'Sending takeoff request to {drone_name}')
        self.drone_clients['takeoff_client'].call_async(req)

    def land(self, drone_name, target_height, duration):
        req = Land.Request()
        req.height = target_height
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.group_mask = 0
        self.get_logger().info(f'Sending land request to {drone_name}')
        self.drone_clients['land_client'].call_async(req)

    def go_to(self, drone_name, goal, yaw, duration,relative):
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = goal
        req.yaw = yaw
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = relative
        req.group_mask = 0
        self.get_logger().info(f'Sending go_to request to {drone_name}')
        self.drone_clients['goto_client'].call_async(req)

    def is_at_position(self, current_position, position, tolerance=0.05):
        return np.linalg.norm(np.array(current_position) - np.array(position)) < tolerance


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--n_agents", help="Number of agents", default=2, type=int)
    parsed_args = parser.parse_args(args)

    N_AGENTS = parsed_args.n_agents

    rclpy.init(args=args)

    drone_nodes = [HeroXR(f'cf_{i}') for i in range(1, N_AGENTS + 1)]
    executor = MultiThreadedExecutor()
    for node in drone_nodes:        
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in drone_nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
