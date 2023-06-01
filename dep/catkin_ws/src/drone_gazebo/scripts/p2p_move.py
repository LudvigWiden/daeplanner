#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose 
import message_filters
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# TODO:
# Support yaw angle
# Subscribe two topics

# Subscribe gazebo_msgs/ModleStates: /gazebo/model_states
# Subscribe Target_state
# Publish gazebo_msgs/ModelState: /gazebo/set_model_state

target_model_name = "drone"
# target_model_name = "husky"
current_pose = Pose()
# hardcode final position:
x = 0
y = 0
z = 0
rx = 0
ry = 0
rz = 0
rw = 1
current_pose.position.x = 0
current_pose.position.y = 0
current_pose.position.z = 0
current_pose.orientation.x = 0
current_pose.orientation.y = 0
current_pose.orientation.z = 0
current_pose.orientation.w = 1
eps = 1e-6


def callback(all_states, goal):
	global current_pose, x, y, z, rx, ry, rz, rw
	# Get the index of target model state
	idx = 0
	for name in all_states.name:
		if (name == target_model_name):
			break
		idx+=1
	# Get pose:
	current_pose = all_states.pose[idx]
	x = goal.position.x
	y = goal.position.y
	z = goal.position.z
	rx = goal.orientation.x
	ry = goal.orientation.y
	rz = goal.orientation.z
	rw = goal.orientation.w




rospy.init_node("p2p_move", anonymous=True)
state_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates)
goal_sub = message_filters.Subscriber("/get_goal", Pose)
# goal_sub = message_filters.Subscriber("husky/get_goal", Pose)
ts = message_filters.ApproximateTimeSynchronizer([state_sub, goal_sub], 10, 0.1, allow_headerless=True)
ts.registerCallback(callback)
# rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
target_pose = Pose()
target_state = ModelState()
velocity = 0.1
angular_velocity = 0.2
rospy.loginfo("Ready")
rate = rospy.Rate(100)
while not rospy.is_shutdown():
	dx = (x-current_pose.position.x)
	dy = (y-current_pose.position.y)
	dz = (z-current_pose.position.z)
	[roll, pitch, yaw] = euler_from_quaternion([rx, ry, rz, rw])
	[current_roll, current_pitch, current_yaw] = \
		euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
	dyaw = (yaw - current_yaw)
	reach = abs(dx) <0.05 and abs(dy) < 0.05 and abs(dz) < 0.05 
	reach_orientation = abs(dyaw) < 0.15
	if (reach == False):
		rospy.loginfo("Move to Position: %s, %s, %s", x, y, z)
		target_pose.position.x = current_pose.position.x+velocity*dx/((dx**2+dy**2+dz**2)**0.5+eps) #*dx/abs(dx+eps)
		target_pose.position.y = current_pose.position.y+velocity*dy/((dx**2+dy**2+dz**2)**0.5+eps) #dy/abs(dy+eps)
		target_pose.position.z = current_pose.position.z+velocity*dz/((dx**2+dy**2+dz**2)**0.5+eps) #/abs(dz+eps)
		target_rot = quaternion_from_euler(0, 0, current_yaw)
		target_pose.orientation.x = target_rot[0]
		target_pose.orientation.y = target_rot[1]
		target_pose.orientation.z = target_rot[2]
		target_pose.orientation.w = target_rot[3]
		target_state.model_name = target_model_name
		target_state.pose = target_pose
		pub.publish(target_state)
	elif (reach == True and reach_orientation == False):
		rospy.loginfo("Move to Yaw: %s", abs(dyaw))
		target_pose.position = current_pose.position
		target_yaw = current_yaw + angular_velocity*(dyaw)/abs(dyaw)
		target_rot = quaternion_from_euler(0, 0, target_yaw)
		target_pose.orientation.x = target_rot[0]
		target_pose.orientation.y = target_rot[1]
		target_pose.orientation.z = target_rot[2]
		target_pose.orientation.w = target_rot[3]
		target_state.model_name = target_model_name
		target_state.pose = target_pose
		pub.publish(target_state)
	else:
		rospy.loginfo("finished, wait for new goal. dyaw: %s", abs(dyaw))
	rate.sleep()