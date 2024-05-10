#!/usr/bin/env python3

import rospy 
import math
from std_msgs.msg import Float64 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import random

class movement_handler(): 

    # Gather data from Odom
    def angle_callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        orientation = pose.orientation 
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')
        self.ori_z = yaw

        position = pose.position
        self.pos_x = position.x
        self.pos_y = position.y
        #print(f"ori_z = {self.ori_z:.3f}, wanted_z = {self.wanted_z:.3f}")   

    # Gather data from Lidar
    def dist_callback(self, topic_data: LaserScan): 

        dist = topic_data.ranges
        dist = list(dist)
        # Zero remover (for the actual thing)
        for i in dist:
            if i == 0:
                dist[dist.index(i)] = 2
                
        self.lidar_data = dist
        front_dist = dist[0:45]+dist[315:359]
        left_dist = dist[45:135]
        right_dist = dist[225:315]
        self.min_front_dist = min(front_dist)
        self.min_left_dist = min(left_dist)
        self.min_right_dist = min(right_dist)

        #print(f"min_dist = {self.min_front_dist:.3f}")

    def __init__(self): 
        self.node_name = "movement_handler" 
        pub_topic_name = "move_cmd" 
        sub_topic_name1 = "odom"
        sub_topic_name2 = "scan"

        # Variable Defenition
        self.ctrl_c = False
        self.ori_z = 0
        self.pos_x = 0
        self.pos_y = 0
        self.min_front_dist = 0
        self.min_right_dist = 0
        self.min_left_dist = 0
        self.stop_dist = 0.37
        self.obst_start_left_angle = 400
        self.obst_end_left_angle = 400
        self.obst_start_right_angle = 400
        self.obst_end_right_angle = 400
        self.left_obst_flag = False
        self.right_obst_flag = False
        self.left_angle_misplacement = 0
        self.right_angle_misplacement = 0
        
        self.lidar_data = []
        self.revered_lidar_data = []
        self.obj_loc = 0                # 0: No Object , 1: Front , 2: Right , 3: Left , 4: Front Right , 5: Front Left , 6 : Right Left     
        self.prev_obj_loc = 0                # 0: No Object , 1: Front , 2: Right , 3: Left , 4: Front Right , 5: Front Left , 6 : Right Left    
        self.prev_angle = 0  
        self.RL_dif = 0 

        rospy.init_node(self.node_name, anonymous=True) 

        self.pub = rospy.Publisher(pub_topic_name, Float64, queue_size=10) 
        self.sub1 = rospy.Subscriber(sub_topic_name1, Odometry, self.angle_callback)
        self.sub2 = rospy.Subscriber(sub_topic_name2, LaserScan, self.dist_callback)
        self.rate = rospy.Rate(10) 

        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: 

            out_angle = Float64()
            self.prev_obj_loc = self.obj_loc
            #### Calculates what angle the object is from right and left
            # Left front obstacle angle detection 
            for i in self.lidar_data[0:45]:
                if self.left_obst_flag == False: 
                    if i < self.stop_dist:
                        self.obst_start_left_angle = self.lidar_data[0:45].index(i)
                        i = i + 1
                        self.left_obst_flag = True
                    else: 
                        self.obst_start_left_angle = 400
                        self.obst_end_left_angle = 400

                else:
                    if i > self.stop_dist:
                        self.obst_end_left_angle = self.lidar_data[0:45].index(i)
                        if self.obst_end_left_angle == 0:
                            self.obst_end_left_angle = 400
                        self.left_obst_flag = False
                        break

            # print(f"LSA = {self.obst_start_left_angle:.3f}")
            # print(f"LEA= {self.obst_end_left_angle:.3f}")

            # Right front obstacle angle detection 
            self.revered_lidar_data = self.lidar_data[::-1]
            for i in self.revered_lidar_data[0:45]:
                if self.right_obst_flag == False: 
                    if i < self.stop_dist:
                        self.obst_start_right_angle = self.revered_lidar_data[0:45].index(i)
                        i = i - 1
                        self.right_obst_flag = True
                    else: 
                        self.obst_start_right_angle = 400
                        self.obst_end_right_angle = 400

                else:
                    if i > self.stop_dist:
                        self.obst_end_right_angle = self.revered_lidar_data[0:45].index(i)
                        if self.obst_end_right_angle == 0:
                            self.obst_end_right_angle = 400
                        self.right_obst_flag = False
                        break
            
            # print("========================================")
            # print(f"RSA = {self.obst_start_right_angle:.3f}")
            # print(f"REA= {self.obst_end_right_angle:.3f}")
            # print("========================================")

            self.right_angle_misplacement = self.obst_end_right_angle - self.obst_start_right_angle
            self.left_angle_misplacement = self.obst_end_left_angle - self.obst_start_left_angle
            self.RL_dif = abs(self.right_angle_misplacement - self.left_angle_misplacement)

            # Object Detection
            if self.min_front_dist <= self.stop_dist:
                if self.min_right_dist <= self.stop_dist:
                    self.obj_loc = 4
                elif self.min_left_dist <= self.stop_dist:
                    self.obj_loc = 5
                else: 
                    self.obj_loc = 1
            elif self.min_right_dist <= self.stop_dist:
                    self.obj_loc = 2
            elif self.min_left_dist <= self.stop_dist:
                    self.obj_loc = 3
            else:
                self.obj_loc = 0

            # print(f"obj loc = {self.obj_loc:.3f}")

            # Motion Handler
            if self.obj_loc == 0:
                out_angle = self.ori_z
            elif self.obj_loc == 1:
                if self.RL_dif > 5:
                    if self.left_angle_misplacement < 80 and self.left_angle_misplacement > 5:
                        out_angle = self.ori_z - math.radians(self.left_angle_misplacement/2)
                    elif self.right_angle_misplacement < 80 and self.right_angle_misplacement > 5:
                        if self.prev_angle < self.ori_z + math.radians(self.right_angle_misplacement/2):
                            out_angle = self.ori_z + math.radians(self.right_angle_misplacement/2)
                    else:
                        if self.prev_angle > self.ori_z:
                            out_angle = self.ori_z + math.pi/2
                        else:
                            out_angle = self.ori_z - math.pi/2
                else:
                    if self.prev_angle > self.ori_z:
                        out_angle = self.ori_z + math.pi/2
                    else:
                        out_angle = self.ori_z - math.pi/2
            elif self.obj_loc == 2:
                # if self.prev_obj_loc != 4:
                out_angle = self.ori_z
            elif self.obj_loc == 3:
                # if self.prev_obj_loc != 5:
                out_angle = self.ori_z
            elif self.obj_loc == 4:
                if self.right_angle_misplacement > 80:
                    out_angle = self.ori_z + math.pi/2
                else:
                    if self.prev_angle < self.ori_z + math.radians(self.right_angle_misplacement/2):
                        out_angle = self.ori_z + math.radians(self.right_angle_misplacement/2)
            elif self.obj_loc == 5:
                if self.left_angle_misplacement > 80:
                    out_angle = self.ori_z - math.pi/2
                else:
                    out_angle = self.ori_z - math.radians(self.left_angle_misplacement/2)
            elif self.obj_loc == 6:
                out_angle = self.ori_z

            print(f"obj loc = {self.obj_loc:.3f}")
            #print(f"angle = {out_angle:.3f}")
            self.pub.publish(out_angle)
            self.rate.sleep()

if __name__ == '__main__':  
    movement_handler_instance = movement_handler() 
    try:
        movement_handler_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass