#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose, PoseStamped
from ros2ai_msgs.srv import Nav2Gpt
from tf_transformations import quaternion_from_euler

import numpy as np

class Nav2ApiServer(Node):
    def __init__(self):
        super().__init__("nav2_api_server")
        self.server = self.create_service(Nav2Gpt, "goToPose", self.service_clbk)
        self.nav2_client = BasicNavigator()

    
    def service_clbk(self, req, res):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = req.x
        pose.pose.position.y = req.y
        quat = quaternion_from_euler(0, 0, np.deg2rad(req.theta))
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.nav2_client.goToPose(pose)
        while not self.nav2_client.isTaskComplete():
            # feedback = self.nav2_client.getFeedback()
            # if feedback.navigation_duration > 600:
            #     self.nav2_client.cancelTask()
            pass
        
        result = self.nav2_client.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        pass
        res.status = True
        return res

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Nav2ApiServer()
        rclpy.spin(node)
    except Exception as e:
        print(f"Exception: {e}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()