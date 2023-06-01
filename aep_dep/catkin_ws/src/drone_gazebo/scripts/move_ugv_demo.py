#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose 

x = -1.367482
y = 9.310670
z = 0.449995

current_x = 0.858947
current_y = 5.998969
current_z = 0.449984

target_model_name = "husky"
rospy.init_node("p2p_move", anonymous=True)
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)