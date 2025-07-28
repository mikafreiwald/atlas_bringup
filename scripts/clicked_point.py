#! /usr/bin/env python3

import rclpy
import rclpy.publisher

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose

from frontier_interfaces.srv import FrontierGoal
from frontiers_msgs.srv import FrontierClosestToGoal, PoseIsUnknown

class ClickedPoint(Node):

    def __init__(self, node_name='clicked_point', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        self.clicked_point = None

        self.point_sub = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.point_callback,
            10
        )
        self.frontier_exploration_client = self.create_client(FrontierGoal, '/atlas/frontier_pose')
        self.frontier_closest_to_goal_client = self.create_client(FrontierClosestToGoal, '/frontier_closest_to_goal')
        self.pose_is_unknown_client = self.create_client(PoseIsUnknown, '/pose_is_unknown')

    def point_callback(self, msg):
        self.get_logger().info('Point Clicked: [{}] [{:.2f}, {:.2f}, {:.2f}]'.format(
            msg.header.frame_id, msg.point.x, msg.point.y, msg.point.z))
        self.clicked_point = msg

    def frontier_closest_to_goal(self):
        if not self.frontier_closest_to_goal_client.service_is_ready():
            self.get_logger().error(f'Service {self.frontier_closest_to_goal_client.srv_name} not ready')
            return
    
        req = FrontierClosestToGoal.Request()
        req.goal.header.frame_id = self.clicked_point.header.frame_id
        req.goal.header.stamp = self.get_clock().now().to_msg()
        req.goal.point.x = self.clicked_point.point.x
        req.goal.point.y = self.clicked_point.point.y

        future = self.frontier_closest_to_goal_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f'Closest to goal: {response}')

    def pose_is_unknown(self):
        if not self.pose_is_unknown_client.service_is_ready():
            self.get_logger().error(f'Service {self.pose_is_unknown_client.srv_name} not ready')
            return
        
        pose = PoseStamped()
        pose.header.frame_id = self.clicked_point.header.frame_id
        pose.pose.position = self.clicked_point.point

        req = PoseIsUnknown.Request()
        req.pose = pose

        future = self.pose_is_unknown_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f'Pose is unknown: {response}')

    def frontier_exploration(self):
        if not self.frontier_exploration_client.service_is_ready():
            self.get_logger().error(f'Service {self.frontier_exploration_client.srv_name} not ready')
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.clicked_point.header.frame_id
        goal_pose.pose.position = self.clicked_point.point

        req = FrontierGoal.Request()
        req.goal = goal_pose
        req.goal.header.stamp = self.get_clock().now().to_msg()

        future = self.frontier_exploration_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(f'{response}')

def main(args=None):
    rclpy.init()
    node = ClickedPoint()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)

            if node.clicked_point != None:
                node.frontier_closest_to_goal()
                node.pose_is_unknown()
                node.clicked_point = None

    except KeyboardInterrupt:
        print('')

    node.destroy_node()


if __name__ == '__main__':
    main()
