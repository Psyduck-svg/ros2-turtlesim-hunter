#!/usr/bin/env python3

import rclpy
import random
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill



class TurtleHunter(Node):
    def __init__(self):
        super().__init__('turtle_hunter')

        self.turtle_pose = None
        self.target_pose = None

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self.turtle_pose_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Spawn service
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        self.spawn_timer = self.create_timer(1.0, self.spawn_once)

        self.kill_client = self.create_client(Kill, '/kill')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')


        # Control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Turtle Hunter started')

    def turtle_pose_callback(self, msg: Pose):
        self.turtle_pose = msg

    def target_pose_callback(self, msg: Pose):
        self.target_pose = msg

    def spawn_once(self):
        self.spawn_timer.cancel()

        req = Spawn.Request()
        req.x = random.uniform(1.0, 10.0)
        req.y = random.uniform(1.0, 10.0)
        req.theta = random.uniform(-3.14, 3.14)
        req.name = 'target'

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_response)

    def spawn_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Spawned target: {response.name}')

            self.create_subscription(
                Pose,
                f'/{response.name}/pose',
                self.target_pose_callback,
                10
            )
        except Exception as e:
            self.get_logger().error(str(e))

    def control_loop(self):
        if self.turtle_pose is None or self.target_pose is None:
            return

        # Compute angle to target
        dx = self.target_pose.x - self.turtle_pose.x
        dy = self.target_pose.y - self.turtle_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        if distance < 0.5:
            self.get_logger().info('🍽️ Target eaten!')
            self.kill_target('target')
            self.target_pose = None
            self.cmd_pub.publish(Twist())
            self.create_timer(1.0, self.spawn_once)
            return

    
        target_angle = math.atan2(dy, dx)

        angle_error = target_angle - self.turtle_pose.theta

        # Normalize angle to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        twist = Twist()
        twist.angular.z = 4.0 * angle_error 
        if abs(angle_error) < 0.3:
            twist.linear.x = 1.5 * distance
        else:  # proportional controller
            twist.linear.x = 0.0                  # NO forward motion yet

        self.cmd_pub.publish(twist)

    def kill_target(self, name):
        request = Kill.Request()
        request.name = name
        self.kill_client.call_async(request)



def main(args=None):
    rclpy.init(args=args)
    node = TurtleHunter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
