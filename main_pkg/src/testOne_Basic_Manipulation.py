#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from tutorial_interfaces.msg import Atworkobjects, Atworkobjectsarray
from geometry_msgs.msg import PoseStamped
from tutorial_interfaces.srv import Detectionsorganized#, Targetgoal
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped

class TestOne_BasicManipulate(Node):

    def __init__(self):
        super().__init__('TestOne_BasicManipulate')

        #SUBSCRIBER's

        #PUBLISHER's

        #SERVICES
        """self.target_goal = self.create_client(Targetgoal, 'target_goal')
        self.target_goal_req = Targetgoal.Request()"""

        self.list_organizer = self.create_client(Detectionsorganized, 'get_all_detections_organized')
        self.list_organizer_req = Detectionsorganized.Request()

        self.list_cleanner= self.create_client(Empty, 'list_cleanner')
        self.list_cleanner_req = Empty.Request()


        self.main_structure()

    def main_structure(self):

        #Até a mesa 2
        """self.target_goal_req.x, self.target_goal_req.y, self.target_goal_req.theta = 1.365, 0.789, 0.0
        self.target_goal.call_async(self.target_goal_req)
        self.target_goal_req.x, self.target_goal_req.y, self.target_goal_req.theta = 1.365, 1.833, 0.0
        self.target_goal.call_async(self.target_goal_req)
        self.target_goal_req.x, self.target_goal_req.y, self.target_goal_req.theta = 2.225, 1.833, 90.0
        self.target_goal.call_async(self.target_goal_req)"""
    
        self.list_cleanner.call_async(self.list_cleanner_req)
        time.sleep(2)
        detection_list = self.list_organizer.call_async(self.list_organizer_req)
        self.get_logger().info(str(detection_list))
        
        #Até a mesa 1
        """self.target_goal_req.x, self.target_goal_req.y, self.target_goal_req.theta = 1.365, 1.833, 0.0
        self.target_goal.call_async(self.target_goal_req)
        self.target_goal_req.x, self.target_goal_req.y, self.target_goal_req.theta = 1.365, 3.177, 0.0
        self.target_goal.call_async(self.target_goal_req)
        self.target_goal_req.x, self.target_goal_req.y, self.target_goal_req.theta = 2.6, 3.287, 0.0
        self.target_goal.call_async(self.target_goal_req)


        self.target_goal_req.x, self.target_goal_req.y, self.target_goal_req.theta = 2.6, 4.6, 0.0
        self.target_goal.call(self.target_goal_req)"""



def main(args=None):
    rclpy.init(args=args)

    teste_one = TestOne_BasicManipulate()

    rclpy.spin(teste_one)

    teste_one.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()