#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
import message_filters 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

target_model_name = "drone"
current_pose = Pose()


# Hardcode target position:
x = 0
y = 0
z = 0
rx = 0
ry = 0
rz = 0
rw = 1
eps = 1e-6
current_pose.position.x = x
current_pose.position.y = y
current_pose.position.z = z 
current_pose.orientation.x = rx
current_pose.orientation.y = ry
current_pose.orientation.z = rz
current_pose.orientation.w = rw

msg_receive = False
new_linear_cmd = True
new_angular_cmd = False
def callback(all_states, goal):
	global current_pose, msg_receive, x, y, z, rx, ry, rz, rw, new_linear_cmd, new_angular_cmd, msg_receive
	idx = 0
	for name in all_states.name:
		if (name == target_model_name):
			break
		idx += 1
	current_pose = all_states.pose[idx]
	x = goal.position.x
	y = goal.position.y
	z = goal.position.z
	rx = goal.orientation.x
	ry = goal.orientation.y
	rz = goal.orientation.z
	rw = goal.orientation.w
	# msg_receive = True
	# rospy.loginfo("Receive message")
	dx = (x-current_pose.position.x)
	dy = (y-current_pose.position.y)
	dz = (z-current_pose.position.z)
	# diff in orientation
	[roll, pitch, yaw] = euler_from_quaternion([rx, ry, rz, rw])
	[current_roll, current_pitch, current_yaw] =  \
			euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
	dyaw = (yaw - current_yaw)
	# reach_position = abs(dx) < 0.2 and abs(dy) < 0.2 and abs(dz) < 0.2
	reach_position = abs(dx) < 0.15 and abs(dy) < 0.15 and abs(dz) < 0.15
	# reach_position = (dx**2 + dy**2 + dz**2)**0.5  < 0.1
	reach_orientation = abs(dyaw) < 0.1
	# if (msg_receive == False):
	# 	rospy.loginfo("Message not Received yet")
	# 	continue
	# rospy.loginfo("Move to Position: %s, %s, %s", dx, dy, dz)
	rospy.loginfo("current yaw/goal yaw: %s, %s", current_yaw, yaw)
	# rospy.loginfo("bool: %s, %s, %s", reach_position, reach_orientation, new_linear_cmd)
	msg_receive = (x != 0 or y !=0  or z != 0 or rx!= 0 or ry!=0 or rz!=0 or rw != 0 )
	# rospy.loginfo("message received? %s", msg_receive)
	# rospy.loginfo("data %s,%s,%s,%s,%s,%s,%s",x,y,z,rx,ry,rx,rw)
	if (reach_position == False  and new_linear_cmd == True and msg_receive):
		rospy.loginfo("move to position")
		target_twist.linear.x = linear_velocity * dx/(dx**2+dy**2+dz**2+eps)**0.5
		target_twist.linear.y = linear_velocity * dy/(dx**2+dy**2+dz**2+eps)**0.5
		target_twist.linear.z = linear_velocity * dz/(dx**2+dy**2+dz**2+eps)**0.5
		target_twist.angular.x = 0
		target_twist.angular.y = 0
		target_twist.angular.z = abs(dyaw)/(0.5/linear_velocity) * dyaw/abs(dyaw)
		target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name

		while not rospy.is_shutdown():
			connections = pub.get_num_connections()
			if (connections > 0):
				pub.publish(target_state)
				break
			rospy.Rate(10).sleep()
		new_linear_cmd = False
		# now = rospy.Time.now()
		# duration = rospy.Duration(1)
		# end_time = now + duration
		# while (rospy.Time.now() < end_time):
		# 	rospy.loginfo("now/end: %s, %s", rospy.Time.now(), end_time)
		# 	pub.publish(target_state)
		# 	rospy.Rate(10).sleep()

		# new_angular_cmd = True
	elif (reach_position):
		rospy.loginfo("Reach position")
		target_twist.linear.x = 0
		target_twist.linear.y = 0
		target_twist.linear.z = 0
		target_twist.angular.x = 0
		target_twist.angular.y = 0
		target_twist.angular.z = 0
		target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		pub.publish(target_state)
		new_linear_cmd = True
		new_angular_cmd = True

	if (reach_position == True and reach_orientation == False and new_angular_cmd == True):
		rospy.loginfo("move to orientaion: dyaw: %s", dyaw)
		target_twist.linear.x = 0
		target_twist.linear.y = 0
		target_twist.linear.z = 0
		target_twist.angular.x = 0
		target_twist.angular.y = 0
		target_twist.angular.z = angular_velocity * dyaw/abs(dyaw)
		target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		pub.publish(target_state)
	elif (reach_position and reach_orientation):
		rospy.loginfo("Finish")
		target_twist.linear.x = 0
		target_twist.linear.y = 0
		target_twist.linear.z = 0
		target_twist.angular.x = 0
		target_twist.angular.y = 0
		target_twist.angular.z = 0
		target_pose = current_pose
		target_state.pose = target_pose
		target_state.twist = target_twist
		target_state.model_name = target_model_name
		pub.publish(target_state)
		new_linear_cmd = True
		new_angular_cmd = False


	# if (reach_position == False):
	# 	rospy.loginfo("Move to Position: %s, %s, %s", dx, dy, dz)
	# 	target_twist.linear.x = linear_velocity * dx/(dx**2+dy**2+dz**2+eps)**0.5
	# 	target_twist.linear.y = linear_velocity * dy/(dx**2+dy**2+dz**2+eps)**0.5
	# 	target_twist.linear.z = linear_velocity * dz/(dx**2+dy**2+dz**2+eps)**0.5
	# 	target_twist.angular.x = 0
	# 	target_twist.angular.y = 0
	# 	target_twist.angular.z = 0
	# 	target_pose = current_pose
	# 	target_state.pose = target_pose
	# 	target_state.twist = target_twist
	# 	target_state.model_name = target_model_name
	# 	pub.publish(target_state)
	# elif (reach_position == True and reach_orientation == False):
	# 	rospy.loginfo("Move to Orientation: %s", dyaw)
	# 	target_twist.linear.x = 0
	# 	target_twist.linear.y = 0
	# 	target_twist.linear.z = 0
	# 	target_twist.angular.x = 0
	# 	target_twist.angular.y = 0
	# 	target_twist.angular.z = angular_velocity * dyaw/abs(dyaw)
	# 	target_pose = current_pose
	# 	target_state.pose = target_pose
	# 	target_state.twist = target_twist
	# 	target_state.model_name = target_model_name
	# 	pub.publish(target_state)
	# else:
	# 	target_twist.linear.x = 0
	# 	target_twist.linear.y = 0
	# 	target_twist.linear.z = 0
	# 	target_twist.angular.x = 0
	# 	target_twist.angular.y = 0
	# 	target_twist.angular.z = 0
	# 	target_pose = current_pose
	# 	target_state.pose = target_pose
	# 	target_state.twist = target_twist
	# 	target_state.model_name = target_model_name
	# 	pub.publish(target_state)



rospy.init_node("p2p_velocity_move", anonymous=True)
state_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates)
goal_sub = message_filters.Subscriber("/get_goal", Pose)
ts = message_filters.ApproximateTimeSynchronizer([state_sub, goal_sub], 10, 0.01, allow_headerless=True)
ts.registerCallback(callback)
# rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
target_state = ModelState()
linear_velocity = 0.2
angular_velocity = 0.5
target_twist = Twist()
target_pose = Pose()
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
rospy.spin()
# message to be published
