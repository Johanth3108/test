#!/usr/bin/env python3

import rospy 
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class control(): 

    def angle_callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        orientation = pose.orientation 
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')
        self.ori_z = yaw
        #print(f"ori_z = {self.ori_z:.3f}, wanted_z = {self.wanted_z:.3f}")   

    def dist_callback(self, topic_data: LaserScan): 
        dist = topic_data.ranges
        dist = list(dist)

        # Zero remover (for the actual thing)
        for i in dist:
            if i == 0:
                dist[dist.index(i)] = 2
        front_dist = dist[0:35]+dist[325:359]
        back_dist = dist[135:225]
        tilt_dist_left = dist[20:90]
        tilt_dist_right = dist[270:340]
        self.min_front_dist = min(front_dist)
        self.min_back_dist = min(back_dist)
        self.min_tilt_dist_left = min(tilt_dist_left)
        self.min_tilt_dist_right = min(tilt_dist_right)
        #print(f"min_dist = {self.min_front_dist:.3f}")

    def order_callback(self, topic_data: Float64):
        self.wanted_z = topic_data.data
        #print(f"wanted_z = {self.wanted_z:.3f}")  
        #print(f"ori_z = {self.ori_z:.3f}, wanted_z = {self.wanted_z:.3f}") 

    def __init__(self): 
        self.node_name = "Controller" 
        pub_topic_name = "cmd_vel" 
        sub_topic_name1 = "odom"
        sub_topic_name2 = "scan"
        sub_topic_name3 = "move_cmd"

        self.ctrl_c = False 
        self.wanted_z = 0
        self.stop_dist = 0.32
        self.adj_side_dist = 0.2
        self.normal_speed = 0.2
        self.tilt_speed = 1
        self.ori_z = 0
        self.min_front_dist = 0
        self.min_back_dist = 0
        self.min_tilt_dist_left = 0
        self.min_tilt_dist_right = 0

        rospy.init_node(self.node_name, anonymous=True) 

        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10) 
        self.sub1 = rospy.Subscriber(sub_topic_name1, Odometry, self.angle_callback)
        self.sub2 = rospy.Subscriber(sub_topic_name2, LaserScan, self.dist_callback)
        self.sub3 = rospy.Subscriber(sub_topic_name3, Float64, self.order_callback)
        self.rate = rospy.Rate(10) 

        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: 

            vel_cmd = Twist()

            angle_dif1 = self.wanted_z - self.ori_z
            angle_dif2 = (0-math.pi-self.ori_z) - (math.pi - self.wanted_z)

            if abs(angle_dif1) <= abs(angle_dif2):
                anglelar_speed = angle_dif1 # rad/s
            else :
                anglelar_speed = angle_dif2 # rad/s

            linear_speed_dist_front_modif = self.normal_speed*math.exp((self.stop_dist - self.min_front_dist)*10)
            if (abs(linear_speed_dist_front_modif) > (self.normal_speed*0.8)) and (abs(linear_speed_dist_front_modif) < (self.normal_speed*1.2)):
                linear_speed_dist_front_modif = self.normal_speed

            linear_speed_dist_back_modif = self.normal_speed*math.exp((self.stop_dist - self.min_back_dist)*10)
            if (abs(linear_speed_dist_back_modif) > (self.normal_speed*0.8)) and (abs(linear_speed_dist_back_modif) < (self.normal_speed*1.2)):
                linear_speed_dist_back_modif = self.normal_speed

            anglelar_speed_left_modif = 0
            if self.min_tilt_dist_left < self.adj_side_dist:
                anglelar_speed_left_modif = self.tilt_speed*math.exp((self.adj_side_dist*0.75 - self.min_tilt_dist_left)*5)
            if abs(anglelar_speed_left_modif) < (self.tilt_speed*0.2) :
                anglelar_speed_left_modif = 0
            anglelar_speed_right_modif = 0
            if self.min_tilt_dist_right < self.adj_side_dist:
                anglelar_speed_right_modif = self.tilt_speed*math.exp((self.adj_side_dist*0.75 - self.min_tilt_dist_right)*5)
            if abs(anglelar_speed_right_modif) < (self.tilt_speed*0.2) :
                anglelar_speed_right_modif = 0

            vel_cmd.linear.x = self.normal_speed - linear_speed_dist_front_modif + linear_speed_dist_back_modif - (anglelar_speed/math.pi)*self.normal_speed # m/s
            vel_cmd.angular.z = anglelar_speed*2 + anglelar_speed_right_modif - anglelar_speed_left_modif
            vel_cmd.angular.z = vel_cmd.angular.z

            self.pub.publish(vel_cmd)
            self.rate.sleep()

if __name__ == '__main__': 
    controller_instance = control() 
    try:
        controller_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass