#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
import message_filters 
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import ast

angular_velocity = 0.5
waypoints_idx = 0;
target_state = ModelState()
target_twist = Twist()
target_pose = Pose()
once_rotation = False
once_motion = False
first_time = True
def callback(all_states):
	global waypoints_idx, once_rotation, once_motion, first_time
	idx = 0
	drone_idx = 0
	for name in all_states.name:
		if (name == target_model_name):
			break
		idx += 1

	for name in all_states.name:
		if (name == "drone"):
			break
		drone_idx += 1
	current_pose = all_states.pose[idx]
	cx, cy, cz, crx, cry, crz, crw = current_pose.position.x, current_pose.position.y, current_pose.position.z,\
			 current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w
	(croll, cpitch, cyaw) = euler_from_quaternion([crx, cry, crz, crw])
	if (cyaw > -math.pi/2):
		cyaw -= math.pi/2
	else:
		cyaw = (2*math.pi + (cyaw-math.pi/2))

	if (waypoints_idx == 0 and first_time):
		q = quaternion_from_euler(0, 0, waypoints[0]*math.pi/180)
		target_pose = current_pose
		target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = q[0], q[1], q[2], q[3]
		target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		first_time = False
	
	# First move the orientation:
	drone_x, drone_y = all_states.pose[drone_idx].position.x, all_states.pose[drone_idx].position.y
	gyaw = waypoints[waypoints_idx]*math.pi/180
	dyaw = gyaw - cyaw
	angular_velocity_direction = (dyaw/(abs(dyaw)+1e-9))
	if (abs(dyaw) > math.pi):
		dyaw = (2*math.pi - abs(dyaw)) * -angular_velocity_direction
		angular_velocity_direction = -angular_velocity_direction

	complete_rotation = abs(dyaw) < 0.15
	rospy.loginfo("current pose/dyaw: %s, %s, %s, %s, %s", cx, cy, cz, cyaw*180/math.pi, dyaw*180/math.pi)

	distance_to_drone = math.sqrt((drone_x-cx)**2 + (drone_y-cy)**2)
	cos_angle = math.cos(cyaw) * (drone_x-cx) + math.sin(cyaw) * (drone_y-cy)
	if (distance_to_drone < 1 and cos_angle >0):
		target_pose = current_pose
		target_twist.linear.x = 0
		target_twist.linear.y = 0
		target_twist.linear.z = 0
		target_twist.angular.z = 0
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		# while not rospy.is_shutdown():
		# 	connections = pub.get_num_connections()
		# 	if (connections > 0):
		# 		pub.publish(target_state)
		# 		break
		# 	rospy.Rate(10).sleep()
		pub.publish(target_state)
		once_rotation = False
		return
		
	target_pose = current_pose
	q = quaternion_from_euler(0, 0, gyaw)
	target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = q[0], q[1], q[2], q[3]
	target_state.pose = target_pose
	target_twist.linear.x = 0	
	target_twist.linear.y = 0
	target_twist.angular.z = 0
	target_state.twist = target_twist
	target_state.model_name = target_model_name
	pub.publish(target_state)


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



	target_pose = current_pose
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
	once_rotation = False

rospy.init_node("field_rotation", anonymous=True)
args = rospy.myargv(argv=sys.argv)
if len(args) != 3:
	print("Please enter correct model name and rotation angle")
target_model_name = args[1]
waypoints = ast.literal_eval(args[2])
# waypoints = [n.strip() for n in waypoints]
print(waypoints)
# target_model_name = "person_walking"
# waypoints = [(5, 5), (5, -6), (-2, -8), (-6, 6)]
rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
rospy.spin()