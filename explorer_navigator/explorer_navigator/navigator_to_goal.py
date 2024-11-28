#! /usr/bin/env python3

# Code by Ricardo Berumen

from geometry_msgs.msg import PoseStamped
from explorer_navigator.nav2_connection_class import NavigatorUtils, TaskResult
import rclpy
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus

import pandas as pd

"""
Navigation to go to a specified pose
"""

def main():
    rclpy.init()

    navigator = NavigatorUtils()

    # Read csv file for positions
    dtype = {"Hazmat": str, "X":float, "Y":float, "Z":float, "Distance":float}
    hazmat_d = []
    x_d = []
    y_d = []
    z_d = []
    df = pd.read_csv("/home/student/ros0_ws/src/ros2_explorer/explorer_navigator/explorer_navigator/data/Detected_Labels_20241126_235738.csv", dtype=dtype)
    print(df)
# Go to the goal pose
    hazmat1 = df.iloc[0,0] #"oxygen"
    hazmat2 = df.iloc[1,0] #"organic peroxide"
    hazmat3 = df.iloc[2,0] #"flammable gas"
    hazmat4 = df.iloc[3,0] #"spontaneously combustible"
    hazmat5 = "dangerous" #df.iloc[4,0]
    select = 0
    select = input("Select goal: \n    1)"+hazmat1+"\n    2)"+hazmat2+"\n    3)"+hazmat3+"\n    4)"+hazmat4+"\n    5)"+hazmat5+"\n")
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

    

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    select_num = int(select)
    """match select:
        case "1":
            goal_pose.pose.position.x = 0.377
            goal_pose.pose.position.y = 0.408
            goal_pose.pose.orientation.w = 0.43
        case "2":
            goal_pose.pose.position.x = 0.4628
            goal_pose.pose.position.y = 0.38983
            goal_pose.pose.orientation.w = 0.4703
        case "3":
            goal_pose.pose.position.x = 1.2603
            goal_pose.pose.position.y = 1.4216
            goal_pose.pose.orientation.w = 1.426
        case "4":
            goal_pose.pose.position.x = 2.632
            goal_pose.pose.position.y = 2.48
            goal_pose.pose.orientation.w = 2.61
        case "5":
            goal_pose.pose.position.x = 1.0
            goal_pose.pose.position.y = -0.0
            goal_pose.pose.orientation.w = 0.0603
        case _:
            goal_pose.pose.position.x = 0.0
            goal_pose.pose.position.y = -0.0
            goal_pose.pose.orientation.w = 0.00
            navigator.info("Failed to receive valid objective")"""
    goal_pose.pose.position.x = df.iloc[select_num,1]
    goal_pose.pose.position.y = df.iloc[select_num,2]
    goal_pose.pose.orientation.w = df.iloc[select_num,3]
    
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


