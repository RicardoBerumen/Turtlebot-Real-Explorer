#! /usr/bin/env python3

# Code by Ricardo Berumen

from geometry_msgs.msg import PoseStamped
from explorer_navigator.nav2_connection_class import NavigatorUtils, TaskResult
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
    initial_pose.pose.position.x = 0.035
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # wait for nav2 to fully activate
    navigator.waitUntilNav2Active()

    # Go to the goal pose

    hazmat1 = "oxygen"
    hazmat2 = "organic peroxide"
    hazmat3 = "flammable gas"
    hazmat4 = "spontaneously combustible"
    hazmat5 = "dangerous"
    select = 0
    select = input("Select goal: \n    1)"+hazmat1+"\n    2)"+hazmat2+"\n    3)"+hazmat3+"\n    4)"+hazmat4+"\n    5)"+hazmat5)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    match select:
        case "1":
            goal_pose.pose.position.x = 0.42
            goal_pose.pose.position.y = 0.484
            goal_pose.pose.orientation.w = 0.0603
        case "2":
            goal_pose.pose.position.x = 1.0
            goal_pose.pose.position.y = -0.0
            goal_pose.pose.orientation.w = 0.0603
        case "3":
            goal_pose.pose.position.x = 1.0
            goal_pose.pose.position.y = -0.0
            goal_pose.pose.orientation.w = 0.0603
        case "4":
            goal_pose.pose.position.x = 1.0
            goal_pose.pose.position.y = -0.0
            goal_pose.pose.orientation.w = 0.0603
        case "5":
            goal_pose.pose.position.x = 1.0
            goal_pose.pose.position.y = -0.0
            goal_pose.pose.orientation.w = 0.0603
        case _:
            goal_pose.pose.position.x = 0.0
            goal_pose.pose.position.y = -0.0
            goal_pose.pose.orientation.w = 0.00
            navigator.info("Failed to receive valid objective")
    
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
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1500.0):
                navigator.cancelNav()
                navigator.info("Navigation Canceled")
    
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


