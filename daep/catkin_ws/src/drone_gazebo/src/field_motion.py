#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import ast


linear_velocity = rospy.get_param('/human_linear_velocity',  0.35)
angular_velocity = rospy.get_param('/human_angular_velocity',  1)

waypoints_idx = 0
target_state = ModelState()
target_twist = Twist()
target_pose = Pose()
once_rotation = False
once_motion = False
first_time = True
first_time_meet = True

def callback(all_states):

	global waypoints_idx, once_rotation, once_motion, first_time, correct_angle_once, first_time_meet
	
	# Find index in model state of the model that executed the field_motion script.
	idx = all_states.name.index(target_model_name)

	# Current pose
	current_pose = all_states.pose[idx]

	cx, cy, cz = current_pose.position.x, current_pose.position.y, current_pose.position.z
	crx, cry, crz, crw = current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w
	(_, _, cyaw) = euler_from_quaternion([crx, cry, crz, crw])
	
	# Starting waypoint
	if (waypoints_idx == 0 and first_time):
		# Teleports the human to the first waypoint
		current_pose.position.x, current_pose.position.y = waypoints[0][0], waypoints[0][1]
		# Sets the orientation
		current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w = crx, cry, crz, crw

		target_pose = current_pose
		target_twist.linear.x = 0
		target_twist.linear.y = 0
		target_twist.linear.z = 0
		target_twist.angular.z = 0
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		first_time = False
	
	# Goal waypoint position
	goal_waypoint = waypoints[waypoints_idx]
	gx, gy = goal_waypoint[0], goal_waypoint[1]
	dx, dy = gx-cx, gy-cy
	gyaw = math.atan2(dy, dx)
	gyaw += math.pi/2
	
	dyaw = gyaw - cyaw
	#angular_velocity_direction = (dyaw/(abs(dyaw)+1e-9))
	#
	#if (abs(dyaw) > math.pi):
	#	dyaw = (2*math.pi - abs(dyaw)) * -angular_velocity_direction
	#	angular_velocity_direction = -angular_velocity_direction
	# Normalize dyaw to the range [-pi, pi]
	if dyaw > math.pi:
		dyaw -= 2 * math.pi
	elif dyaw < -math.pi:
		dyaw += 2 * math.pi

	# Compute the direction of the angular velocity
	if dyaw > 0:
		angular_velocity_direction = 1
	else:
		angular_velocity_direction = -1

	# Rotate until < 0.1 difference
	complete_rotation = abs(dyaw) < 0.1
	delta = math.sqrt((dx**2 + dy**2))
	complete_motion = delta < 0.1

	if avoidance_mode:
		drone_idx = all_states.name.index("drone")
		drone_x, drone_y = all_states.pose[drone_idx].position.x, all_states.pose[drone_idx].position.y
		
		distance_to_drone = math.sqrt((drone_x-cx)**2 + (drone_y-cy)**2)
		drone_dx_norm = (drone_x-cx)/((drone_x-cx)**2+(drone_y-cy)**2)**0.5
		drone_dy_norm = (drone_y-cy)/((drone_x-cx)**2+(drone_y-cy)**2)**0.5
		cos_angle = math.cos(cyaw) * drone_dx_norm + math.sin(cyaw) * drone_dy_norm

		if (distance_to_drone < 1 and cos_angle >0 and first_time_meet==True):
			target_pose = current_pose
			target_twist.linear.x = 0
			target_twist.linear.y = 0
			target_twist.linear.z = 0
			target_state.pose = target_pose
			target_state.twist = target_twist
			target_state.model_name = target_model_name

			pub.publish(target_state)
			# Move back to previous waypoint
			waypoints_idx -= 1
			# If negative, go to last waypoint in list
			if (waypoints_idx < 0):
				waypoints_idx = len(waypoints)-1
			once_rotation = False
			once_motion = False
			first_time_meet = False
			return

	# First move the orientation:
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
			once_rotation = True # Keep rotating until complete_rotation is True
			while not rospy.is_shutdown():
				connections = pub.get_num_connections()
				if (connections > 0):
					pub.publish(target_state)
					break
				rospy.Rate(10).sleep()
		return 

	if (not complete_motion):
		if (once_motion == False):
			target_twist.linear.x = dx/delta * linear_velocity	
			target_twist.linear.y = dy/delta * linear_velocity
			target_twist.angular.z = 0
			target_pose = current_pose
			target_state.pose = target_pose
			target_state.twist = target_twist
			target_state.model_name = target_model_name
			once_motion = True # Keep moving until complete_motion is True
			while not rospy.is_shutdown():
				connections = pub.get_num_connections()
				if (connections > 0):
					pub.publish(target_state)
					break
				rospy.Rate(10).sleep()
		return
	
	# If rotation and moving done
	first_time_meet = True
	target_pose = Pose()
	target_pose.position.x, target_pose.position.y, target_pose.position.z = gx, gy, cz
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


rospy.sleep(4)
rospy.init_node("field_motion", anonymous=True)
args = rospy.myargv(argv=sys.argv)
target_model_name = args[1]
waypoints = ast.literal_eval(args[2])
avoidance_mode = True if (args[3] == 'true' or args[3] == 'True') else False
if avoidance_mode:
    print("Human avoidance mode is enabled.")
else:
    print("Human avoidance mode is disabled.")
    
rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
rospy.spin()
