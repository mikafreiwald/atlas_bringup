#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

"""
Basic navigation demo to go to pose.
"""

class Navigator(Node):

    def __init__(self, node_name='py_navigator', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info(
            'Navigating to goal: '
            + str(pose.pose.position.x)
            + ' '
            + str(pose.pose.position.y)
            + '...'
        )
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle or not self.goal_handle.accepted:
            self.error(
                'Goal to '
                + str(pose.pose.position.x)
                + ' '
                + str(pose.pose.position.y)
                + ' was rejected!'
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.error(f'Task failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        # self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getStatus(self):
        return self.status

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

def main() -> None:
    rclpy.init()

    navigator = Navigator()

    while True:

        command_line = input('Command: ').strip()
        parts = command_line.split()
        command = parts[0].lower()
        args = parts[1:]

        if command == 'exit':
            break

        elif command == 'nav':
            # Go to our demos first goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'atlas/map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(args[0])
            goal_pose.pose.position.y = float(args[1])
            goal_pose.pose.orientation.w = 1.0
            goal_pose.pose.orientation.z = 0.0

            navigator.goToPose(goal_pose)

            i = 0
            try:
                while not navigator.isTaskComplete():
                    i = i + 1
                    feedback = navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        print(
                            'Estimated time of arrival: '
                            + '{:.1f}'.format(
                                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                                / 1e9
                            )
                            + ' seconds.'
                        )
                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=2.0):
                        navigator.cancelTask()

            except KeyboardInterrupt:
                print('Interrupted')
                navigator.cancelTask()
                continue

            # Do something depending on the return code
            result = navigator.getStatus()
            if result == GoalStatus.STATUS_SUCCEEDED:
                print('Goal succeeded!')
            elif result == GoalStatus.STATUS_ABORTED:
                print('Goal was canceled!')
            elif result == GoalStatus.STATUS_CANCELED:
                print('Goal failed!{error_code}:{error_msg}')
            else:
                print('Goal has an invalid return status!')

        else:
            print(f'Unknown command {command}')

    exit(0)


if __name__ == '__main__':
    main()
