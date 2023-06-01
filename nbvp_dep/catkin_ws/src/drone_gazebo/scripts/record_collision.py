#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import message_filters

# This node is used for checking collision for drone with dynamic obstacle for human
# Dynamic Obstacle list
# do_list = ["actor_2", "actor_1", "actor", "actor_0", "actor_3"] #Tunnel
do_list = ["actor1", "actor2"]
idx_list = [0] * len(do_list)
target_name = "drone"
first_time = True
last_one = None

# Assume that human is a box (1m x 1m x 1.8 m)
length = 1.0 #x
width = 1.0  #y
height = 1.8 #z

# Drone (0.4m x 0.4m x 0.1m)
drone_l = 0.4
drone_w = 0.4
drone_h = 0.1



collision_counter = 0
def callback(all_states, odom):
	# rospy.loginfo("test")
	global do_list, idx_list, collision_counter, last_one, first_time
	# Get index for each dynamic obstacle in the world
	if (first_time == True):
		first_time = False
		world_idx = 0
		for name in all_states.name:
			list_idx = 0
			for do in do_list:
				if (name == do):
					idx_list[list_idx] = world_idx
					rospy.loginfo("%s", world_idx)
					break
				list_idx += 1	
			world_idx += 1

	# Drone Position
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	z = odom.pose.pose.position.z
	for idx in idx_list:
		pose = all_states.pose[idx]
		dx = abs(x - pose.position.x)
		dy = abs(y - pose.position.y)
		dz = abs(z - pose.position.z)
		rospy.loginfo("%s distance: %s", all_states.name[idx], (dx**2+dy**2+dz**2)**0.5)
		collision = (dx < (length+drone_l)/2 and dy < (width+drone_w)/2 and dz < (height+drone_h)/2)
		if (collision):
			if (last_one != all_states.name[idx]):
				rospy.loginfo("Collision!!!!!!!!!!!!!! Count: %s", collision_counter)
				rospy.loginfo("With %s", all_states.name[idx])
				last_one = all_states.name[idx]
				collision_counter += 1
	rospy.loginfo("Collision Number: %s",collision_counter)

rospy.init_node("record_collision", anonymous=True)
state_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates)
odom_sub = message_filters.Subscriber("/odom", Odometry)
ts = message_filters.ApproximateTimeSynchronizer([state_sub, odom_sub], 10, 0.1, allow_headerless=True)
ts.registerCallback(callback)
# rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
rospy.spin()