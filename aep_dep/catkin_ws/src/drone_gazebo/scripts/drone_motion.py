#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
import message_filters 
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import ast

linear_velocity = 0.3 
angular_velocity = 0.8

waypoints_idx = 0;
target_state = ModelState()
target_twist = Twist()
target_pose = Pose()
once_rotation = False
once_motion = False
first_time = True
first_time_meet = True

def callback(all_states):
	global waypoints_idx, once_rotation, once_motion, first_time, correct_angle_once, first_time_meet
	idx = 0
	drone_idx = 0
	for name in all_states.name:
		if (name == target_model_name):
			break
		idx += 1


	current_pose = all_states.pose[idx]
	cx, cy, cz, crx, cry, crz, crw = current_pose.position.x, current_pose.position.y, current_pose.position.z,\
			 current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w
	(croll, cpitch, cyaw) = euler_from_quaternion([crx, cry, crz, crw])
	# if (target_model_name[0]=="p"):
	# 	if (cyaw > -math.pi/2):
	# 		cyaw -= math.pi/2
	# 	else:
	# 		cyaw = (2*math.pi + (cyaw-math.pi/2))

	if (waypoints_idx == 0 and first_time):
		if (target_model_name[0]=="p"):
			current_pose.position.x, current_pose.position.y, current_pose.position.z = waypoints[0][0], waypoints[0][1], 2
		else:
			current_pose.position.x, current_pose.position.y = waypoints[0][0], waypoints[0][1]
		current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w = 0, 0, 0, 1
		target_pose = current_pose
		target_twist.linear.x = 0
		target_twist.linear.y = 0
		target_twist.linear.z = 0
		target_twist.angular.z = 0
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		first_time = False
	
	# First move the orientation:
	goal_waypoint = waypoints[waypoints_idx]
	gx, gy = goal_waypoint[0], goal_waypoint[1]
	dx, dy = gx-cx, gy-cy

	gyaw = math.atan2(dy, dx)
	# gyaw += math.pi/2
	# cyaw -=  math.pi/2
	dyaw = gyaw - cyaw
	angular_velocity_direction = (dyaw/(abs(dyaw)+1e-9))
	# if (gyaw > -math.pi/2):
	# 	gyaw -= math.pi/2
	# else:
	# 	gyaw = (2*math.pi + (gyaw-math.pi/2))

	if (abs(dyaw) > math.pi):
		dyaw = (2*math.pi - abs(dyaw)) * -angular_velocity_direction
		angular_velocity_direction = -angular_velocity_direction

	complete_rotation = abs(dyaw) < 0.05
	delta = math.sqrt((dx**2 + dy**2))
	# complete_motion = delta < 0.1
	complete_motion = delta < 0.05
	# rospy.loginfo("current pose/dyaw: %s, %s, %s, %s, %s", cx, cy, cz, cyaw*180/math.pi, dyaw*180/math.pi)
	# rospy.loginfo("goal: %s, %s, %s, %s", gx, gy, gyaw, cyaw)


	

	if (not complete_rotation):
		# rotate
		if (once_rotation == False):
			target_twist.linear.x = 0	
			target_twist.linear.y = 0	
			target_twist.angular.z = angular_velocity_direction * angular_velocity
			q = quaternion_from_euler(0, 0, cyaw)
			target_pose = current_pose
			target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = q[0], q[1], q[2], q[3]
			target_state.pose = target_pose
			target_state.twist = target_twist
			target_state.model_name = target_model_name
			once_rotation = True
			while not rospy.is_shutdown():
				connections = pub.get_num_connections()
				if (connections > 0):
					pub.publish(target_state)
					break
				rospy.Rate(10).sleep()
		return 


	# rospy.loginfo("complete rotation")
	if (not complete_motion):
		if (once_motion == False):
			target_twist.linear.x = dx/delta * linear_velocity	
			target_twist.linear.y = dy/delta *linear_velocity
			target_twist.angular.z = 0
			target_pose = current_pose
			target_state.pose = target_pose
			target_state.twist = target_twist
			target_state.model_name = target_model_name
			once_motion = True
			while not rospy.is_shutdown():
				connections = pub.get_num_connections()
				if (connections > 0):
					pub.publish(target_state)
					break
				rospy.Rate(10).sleep()
		return
	# rospy.loginfo(waypoints_idx)
	# rospy.loginfo(cyaw)
	# rospy.loginfo(gyaw)
	first_time_meet = True
	target_pose = Pose()
	target_pose.position.x, target_pose.position.y, target_pose.position.z = gx, gy, 2
	q = quaternion_from_euler(0, 0, gyaw)
	target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = q[0], q[1], q[2], q[3]
	target_state.pose = target_pose
	target_twist.linear.x = 0	
	target_twist.linear.y = 0
	target_twist.angular.z = 0
	target_state.twist = target_twist
	target_state.model_name = target_model_name
	pub.publish(target_state)


	waypoints_idx += 1
	if (waypoints_idx == len(waypoints)):
		waypoints_idx = 0
	once_motion = False
	once_rotation = False



















rospy.init_node("field_motion", anonymous=True)
args = rospy.myargv(argv=sys.argv)
if len(args) != 3:
	print("Please enter correct model name and waypoints")
target_model_name = args[1]
waypoints = ast.literal_eval(args[2])
# waypoints = [n.strip() for n in waypoints]
print(waypoints)
# target_model_name = "person_walking"
# waypoints = [(5, 5), (5, -6), (-2, -8), (-6, 6)]
rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
rospy.spin()