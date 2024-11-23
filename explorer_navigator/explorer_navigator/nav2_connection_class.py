#!/usr/bin/env python3

# Written by Ricardo Berumen

# Imports
from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import AssistedTeleop, BackUp, DriveOnHeading, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import (
    FollowPath,
    FollowWaypoints,
    NavigateThroughPoses,
    NavigateToPose,
)
from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Description of goal results in nav2
class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

# Main class for utils
class NavigatorUtils(Node):
    def __init__(self, node_name='navigator_utils', namespace='') -> None:
        super().__init__(node_name=node_name, namespace=namespace)
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.drive_on_heading_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
        self.model_pose_sub = self.create_subscription(
                                        PoseWithCovarianceStamped,
                                        'amcl_pose',
                                        self._amclPoseCallback,
                                        amcl_pose_qos,
                                    )
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        
        self.clear_costmap_global_srv = self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self.clear_costmap_local_srv = self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(GetCostmap, 'global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(GetCostmap, 'local_costmap/get_costmap')

    def destroyNode(self):
        self.destroy_node()

    def destroy_node(self):
        self.nav_through_poses_client.destroy()
        self.nav_to_pose_client.destroy()
        self.follow_waypoints_client.destroy()
        self.follow_path_client.destroy()
        self.compute_path_to_pose_client.destroy()
        self.smoother_client.destroy()
        self.spin_client.destroy()
        self.backup_client.destroy()
        self.drive_on_heading_client.destroy()
        self.assisted_teleop_client.destroy()
        super().destroy_node()
    
    def setInitialPose(self, initial_pose):
        # Setting of initial pose of the system
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()
    
    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return
    
    def goThroughPoses(self, poses):
        # Sending navtopose action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg, self._feedbackCallback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False
        
        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def goToPose(self, pose):
        # Sends a "NavToPose" action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting ..")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.info("Navigating to goal: " + str(pose.pose.position.x) + " " + 
                                            str(pose.pose.position.y) + " ..")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error("Goal to" + str(pose.pose.position.x) + " " + str(pose.pose.position.y) + " was rejected!")
            return False
        
        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def cancelNav(self):
        self.info("Canceling current goal ")
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future=future)
        return
    
    def isNavComplete(self):
        if not self.result_future:
            # task cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info("Goal with failed status code: {0}".format(self.status))
                return True
        else:
            # Timed out, strill processing
            return False
        
        self.info("Goal suceeded!")
        return True
    
    def getFeedback(self):
        return self.feedback
    
    def getResult(self):
        return self.status
    
    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate("bt_navigator")
        self.info("Nav2 is ready to use! ")
        return
    
    def _waitForNodeToActivate(self, node_name):
        # Wait for the node to become active
        self.debug('Waiting for ' + node_name + 'to become active...')
        node_service = node_name + "/get_state"
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + " service not available, waiting...")

        req = GetState.Request()
        state = 'unkown'
        while (state != 'active'):
            self.debug("Getting " + node_name + "state...")
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug("Result of get_state: %s" % state)
            time.sleep(2)
        return
    
    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    
    # Info functions
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