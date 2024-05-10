#!/usr/bin/env python3

import rospy 
import os
import rospkg
import roslaunch
import subprocess
from nav_msgs.srv import GetMap

class MappingNode:
    def __init__(self):
        rospy.init_node('mapping_node', anonymous=True)

        # Launch SLAM
        self.launch_slam()

        # Register save_map() to be called periodically
        rospy.Timer(rospy.Duration(2), self.save_map)  # Save map every 5 seconds

    def launch_slam(self):
        # Launch SLAM using roslaunch API
        package = 'turtlebot3_slam'
        launch_file = 'turtlebot3_slam.launch'
        roslaunch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_file])
        self.launch.start()

    def save_map(self, event=None):
        # Define the map file name
        map_name = "explore_map"
        
        # Navigate to the maps directory
        maps_dir = os.path.join(rospkg.RosPack().get_path('acs6121_team5'), 'maps')
        print("Maps directory:", maps_dir)
        
        # Create the directory if it doesn't exist
        if not os.path.exists(maps_dir):
            os.makedirs(maps_dir)

        # Change the current working directory to maps_dir
        os.chdir(maps_dir)

        # Define the command to run
        command = f"rosrun map_server map_saver -f {map_name}"

        # Execute the command
        try:
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to save map: {e}")


    def run(self):
        # Main function to run the mapping node
        rospy.spin()  # <- Ensure ROS is spinning to handle events

if __name__ == '__main__':
    mapping_node = MappingNode()
    mapping_node.run()