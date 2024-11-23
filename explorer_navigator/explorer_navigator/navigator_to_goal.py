#! /usr/bin/env python3

# Code by Ricardo Berumen

from geometry_msgs.msg import PoseStamped
from nav2_connection_class import NavigatorUtils, TaskResult
import rclpy
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus

"""
Navigation to go to a specified pose
"""

def main():
    rclpy.init()

    navigator = NavigatorUtils()

    # Set Initial Pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # wait for nav2 to fully activate
    navigator.waitUntilNav2Active()

    # Go to the goal pose

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0
    navigator.goToPose(goal_pose)


    i = 0

    while not navigator.isNavComplete():
        
        i = i+1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds')
            
            # Nav timeout if taken too long
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()
    
    # Return code
    result = navigator.getResult()
    if result == GoalStatus.STATUS_SUCCEEDED:
        print("Goal suceeded")
    elif result == GoalStatus.STATUS_CANCELED:
        print("Goal canceled")
    elif result == GoalStatus.STATUS_ABORTED:
        print("Goal failed (aborted)")
    
    else:
        print("Goal has an invalid return status")
    
    exit(0)


if __name__ == '__main__':
    main()


