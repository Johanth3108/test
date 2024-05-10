#!/usr/bin/env python3

import rospy 
import math
from std_msgs.msg import Float64, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class MovementHandler(): 

    def angle_callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        orientation = pose.orientation 
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')
        self.ori_z = yaw

    def dist_callback(self, topic_data: LaserScan): 
        dist = topic_data.ranges
        dist = list(dist)
        for i in dist:
            if i == 0:
                dist[dist.index(i)] = 2
        front_dist = dist[0:45]+dist[315:359]
        left_dist = dist[45:135]
        right_dist = dist[225:315]
        self.min_front_dist = min(front_dist)
        self.min_left_dist = min(left_dist)
        self.min_right_dist = min(right_dist)

    def move_callback(self, topic_data: Float64):
        self.wanted_z = topic_data.data

    def __init__(self): 
        self.node_name = "movement_handler" 
        pub_topic_name = "cmd_vel" 
        sub_topic_name1 = "odom"
        sub_topic_name2 = "scan"
        sub_topic_name3 = "move_cmd"

        self.ctrl_c = False
        self.ori_z = 0
        self.pos_x = 0
        self.pos_y = 0
        self.min_front_dist = 0
        self.min_right_dist = 0
        self.min_left_dist = 0
        self.stop_dist = 0.37
        self.wanted_z = 0

        rospy.init_node(self.node_name, anonymous=True) 

        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10) 
        self.sub1 = rospy.Subscriber(sub_topic_name1, Odometry, self.angle_callback)
        self.sub2 = rospy.Subscriber(sub_topic_name2, LaserScan, self.dist_callback)
        self.sub3 = rospy.Subscriber(sub_topic_name3, Float64, self.move_callback)
        self.rate = rospy.Rate(10) 

        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: 

            vel_cmd = Twist()

            angle_dif = self.wanted_z - self.ori_z
            linear_speed_dist_modif = 0.2 * math.exp((self.stop_dist - self.min_front_dist)*10)

            vel_cmd.linear.x = 0.2 - linear_speed_dist_modif
            vel_cmd.angular.z = angle_dif * 2

            self.pub.publish(vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':  
    movement_handler_instance = MovementHandler() 
    try:
        movement_handler_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
