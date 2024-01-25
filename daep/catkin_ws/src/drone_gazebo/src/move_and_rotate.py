#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist, PoseStamped
import message_filters 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from aeplanner.msg import Goal
import math
import numpy as np
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.msg import Reference


target_model_name = "drone"
eps = 1e-6 # Avoid division by zero
rotate = False # Rotating at the moment



POSITION_DIFF = 0.02 # Position minimum between the drone and a pose
ANGLE_DIFF = 0.06 # Orientation minimum between the drone and a pose
SAFE_DISTANCE = 1.0 # How close we look for danger (dynamic obstacles)
AVOIDANCE_DISTANCE = 0.10 # The length that we move away from danger
MAX_STRENGTH = 10.0 # Maximum function value.
AVOIDANCE_GOAL = False # Currently moving to a avoidance goal.

# New goal pose
AVOIDANCE_GOAL_X = 0
AVOIDANCE_GOAL_Y = 0
AVOIDANCE_GOAL_Z = 0
AVOIDANCE_GOAL_YAW = 0

# Assume that human is a box (1m x 1m x 1.8 m)
human_l, human_w, human_h = 1.0, 1.0, 1.8 
# Drone (0.4m x 0.4m x 0.1m)
drone_l, drone_w, drone_h = 0.4, 0.4, 0.1

# Map boundaries
# Read the boundary parameters
boundary_max = rospy.get_param('/boundary/max')
boundary_min = rospy.get_param('/boundary/min')
MAX_X = boundary_max[0]
MAX_Y = boundary_max[1]
MAX_Z = boundary_max[2]
MIN_X = boundary_min[0]
MIN_Y = boundary_min[1]

def sendMrsTrajectory(pos_x, pos_y, pos_z, yaw, max_speed):
	rospy.wait_for_service('/uav1/trajectory_generation/path')

	try:
		path_service = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)

		request = PathSrvRequest()
		request.path.header.stamp = rospy.Time.now()
		request.path.fly_now = True
		request.path.stop_at_waypoints = False
		request.path.loop = False

		request.path.override_constraints = True
		request.path.override_max_velocity_horizontal = max_speed
		request.path.override_max_acceleration_horizontal = 4.0
		request.path.override_max_jerk_horizontal = 60.0
		request.path.override_max_velocity_vertical = 4.0
		request.path.override_max_acceleration_vertical = 2.0
		request.path.override_max_jerk_vertical = 60.0
		
		request.path.use_heading = True
		request.path.relax_heading = False

		if len(yaw) == 1 and yaw[0] == 0:
			yaw = [0] * len(pos_x) 

		for px, py, pz, yw in zip(pos_x, pos_y, pos_z, yaw):
			wp = Reference()
			wp.position.x = px
			wp.position.y = py
			wp.position.z = pz
			wp.heading = yw
			request.path.points.append(wp)

		response = path_service(request)

		if response.success:
			rospy.loginfo('Path request successful: %s', response.message)
		else:
			rospy.logwarn('Path request failed: %s', response.message)

	except rospy.ServiceException as e:
		rospy.logerr('Service call failed: %s', str(e))

# Send to Gazebo And MRS
def sendDestination(position):
	pub.publish(position)
	pos_x = [position.pose.position.x]
	pos_y = [position.pose.position.y]
	pos_z = [position.pose.position.z]
	yaw = [position.twist.angular.z]
	sendMrsTrajectory(pos_x, pos_y, pos_z, yaw, 8)

# Keep track of human positions
def update_human_positions(model_states):
	global dynamic_objects
	for model in model_states.name:
		if "person_walking" in model:
			person_idx = model_states.name.index(model)
			person_pose = model_states.pose[person_idx]
			person_twist = model_states.twist[person_idx]
			dynamic_objects[model] = [person_pose, person_twist]


# Function that puts a strength value on a distance
def strength_function(distance):
	if distance >= SAFE_DISTANCE:
		return 0
	else:
		strength = MAX_STRENGTH - distance/(SAFE_DISTANCE/MAX_STRENGTH)
		return strength

# Get the bounding box of the drone
def get_drone_bbox(drone_position):
	drone_bounds = np.array([
		[drone_position.x - drone_l/2, drone_position.x + drone_l/2],
		[drone_position.y - drone_w/2, drone_position.y + drone_w/2],
		[drone_position.z, drone_position.z + drone_h]])
	return drone_bounds

# Get the bounding box of a human
def get_human_bbox(human_position):
	human_bounds = np.array([
		[human_position.x - human_l/2, human_position.x + human_l/2],
		[human_position.y - human_w/2, human_position.y + human_w/2],
		[human_position.z, human_position.z + human_h]])
	return human_bounds


# Get the distance between two bounding boxes.
def get_bbox_distance(drone_bbox, human_bbox):
    drone_center = np.array([(drone_bbox[0,0] + drone_bbox[0,1])/2,
							 (drone_bbox[1,0] + drone_bbox[1,1])/2,
							 (drone_bbox[2,0] + drone_bbox[2,1])/2])
    human_center = np.array([(human_bbox[0,0] + human_bbox[0,1])/2,
							 (human_bbox[1,0] + human_bbox[1,1])/2,
							 (human_bbox[2,0] + human_bbox[2,1])/2])
    distance = np.linalg.norm(drone_center - human_center) \
			   - 0.5*(drone_bbox[0,1]-drone_bbox[0,0]) \
			   - 0.5*(drone_bbox[1,1]-drone_bbox[1,0]) \
			   - 0.5*(drone_bbox[2,1]-drone_bbox[2,0])
    return distance


# Find the shortest turn direction
def get_shortest_rotation_direction(drone_yaw, goal_yaw):
	# Calculate the difference between the goal yaw and the current yaw
	dyaw = goal_yaw - drone_yaw

	# Ensure that dyaw is in the range -pi to pi to get the shortest rotation direction
	if dyaw > math.pi:
		dyaw -= 2*math.pi
	elif dyaw < -math.pi:
		dyaw += 2*math.pi

	# Determine the direction of the rotation (clockwise or counterclockwise)
	if dyaw > 0:
		# Rotate counterclockwise
		angular_velocity_direction = 1
	else:
		# Rotate clockwise
		angular_velocity_direction = -1
	return angular_velocity_direction


# Adjust a goal to be inside map boundaries
def adjust_goal_position(AVOIDANCE_GOAL_X, AVOIDANCE_GOAL_Y, AVOIDANCE_GOAL_Z):
    # Check if the goal position is within the map boundary
    if AVOIDANCE_GOAL_X > MAX_X:
        AVOIDANCE_GOAL_X = MAX_X
    elif AVOIDANCE_GOAL_X < MIN_X:
        AVOIDANCE_GOAL_X = MIN_X

    if AVOIDANCE_GOAL_Y > MAX_Y:
        AVOIDANCE_GOAL_Y = MAX_Y
    elif AVOIDANCE_GOAL_Y < MIN_Y:
        AVOIDANCE_GOAL_Y = MIN_Y

    if AVOIDANCE_GOAL_Z > MAX_Z:
        AVOIDANCE_GOAL_Z = MAX_Z

    return AVOIDANCE_GOAL_X, AVOIDANCE_GOAL_Y, AVOIDANCE_GOAL_Z


# Visualize the avoidance goal in RVIZ
def visualize_goal_avoidance_arrow(displacement_vector, drone_position, AVOIDANCE_GOAL_YAW):
	displacement_magnitude = np.linalg.norm(displacement_vector)*5
	marker = Marker()
	marker.header.frame_id = "world"
	marker.type = Marker.ARROW
	marker.action = Marker.ADD
	marker.pose.position.x = drone_position.x
	marker.pose.position.y = drone_position.y
	marker.pose.position.z = drone_position.z
	marker.scale.x = displacement_magnitude
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	qq = quaternion_from_euler(0, 0, AVOIDANCE_GOAL_YAW)
	marker.pose.orientation.x = qq[0]
	marker.pose.orientation.y = qq[1]
	marker.pose.orientation.z = qq[2]
	marker.pose.orientation.w = qq[3]
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.5
	avoidance_goal_publisher.publish(marker)



def callback(all_states, goal):
	global rotate, AVOIDANCE_GOAL, AVOIDANCE_GOAL_X, AVOIDANCE_GOAL_Y, AVOIDANCE_GOAL_Z, AVOIDANCE_GOAL_YAW, DRONEPOS
	# Wait until drone is present
	if target_model_name not in all_states.name:
		return
	
	# Boolean publisher message
	GOAL_REACHED = Bool()
	GOAL_REACHED.data = False

	# Find drone in model states
	idx = all_states.name.index(target_model_name)
	current_pose = all_states.pose[idx]
	drone_position = current_pose.position
	drone_orientation = current_pose.orientation
	# Current x,y,z, rx, ry, rz, rw
	cx = drone_position.x
	cy = drone_position.y
	cz = drone_position.z
	crx = drone_orientation.x
	cry = drone_orientation.y
	crz = drone_orientation.z
	crw = drone_orientation.w
	(_, _, current_yaw) = euler_from_quaternion([crx, cry, crz, crw])
	# Yaw between [0, 2pi]
	current_yaw = (current_yaw + 2*math.pi) % (2*math.pi)
	#if current_yaw < 0:
	#	current_yaw = 2*math.pi - (-current_yaw)

	distances_to_humans_too_close = []
	if AVOIDANCE_ACTIVATED:
		if not AVOIDANCE_GOAL:
			drone_bbx = get_drone_bbox(drone_position)
			human_positions = []
			avoidance = Bool()
			# Find if any human is too close
			for model, pose_twist in dynamic_objects.items():
				human_position = pose_twist[0].position
				human_position_array = np.array([human_position.x, human_position.y, human_position.z])
				human_bbx = get_human_bbox(human_position)
				distance = get_bbox_distance(drone_bbx, human_bbx)
				if distance < SAFE_DISTANCE:
					distances_to_humans_too_close.append(distance)
					human_positions.append(human_position_array)

		
			# If there exists human too close
			if distances_to_humans_too_close:

				# Calculate the weights for each human based on the strength function
				weights = np.array([strength_function(d) for d in distances_to_humans_too_close])
				
				# Calculate the unit vectors pointing from the drone to each human
				drone_position_array = np.array([drone_position.x, drone_position.y, drone_position.z])
				human_vectors = np.array(human_positions) - drone_position_array
				
				# Reduce Z-axis influence by 50%
				human_vectors[:, 2] *= 0.5

				human_unit_vectors = human_vectors / np.linalg.norm(human_vectors, axis=1, keepdims=True)

				# Calculate the weighted average of the human unit vectors
				new_direction = np.average(human_unit_vectors, axis=0, weights=weights)

				# Normalize the new direction to be a unit vector
				new_direction = new_direction / np.linalg.norm(new_direction)

				# Calculate the displacement vector
				displacement_vector = AVOIDANCE_DISTANCE * (-new_direction)
				
				# Calculate the goal position by adding the displacement vector to the current position
				AVOIDANCE_GOAL_X = drone_position.x + displacement_vector[0]
				AVOIDANCE_GOAL_Y = drone_position.y + displacement_vector[1]
				AVOIDANCE_GOAL_Z = drone_position.z + displacement_vector[2]
				AVOIDANCE_GOAL_YAW = math.atan2(AVOIDANCE_GOAL_Y - drone_position.y, AVOIDANCE_GOAL_X - drone_position.x)
				
				############### Adjust Goal ##############
				## Yaw angle between 0 and 2pi
				if AVOIDANCE_GOAL_YAW < 0:
					AVOIDANCE_GOAL_YAW += 2 * math.pi
				
				# We don't want to avoid humans by going downwards
				# We calculate the vector from the centre of the body, and dont want this to impact
				if AVOIDANCE_GOAL_Z <= cz:
					AVOIDANCE_GOAL_Z = cz

				# Adjust values according to map boundaries
				AVOIDANCE_GOAL_X, AVOIDANCE_GOAL_Y, AVOIDANCE_GOAL_Z = adjust_goal_position(AVOIDANCE_GOAL_X, AVOIDANCE_GOAL_Y, AVOIDANCE_GOAL_Z)
				
				#RVIZ
				visualize_goal_avoidance_arrow(displacement_vector, drone_position, AVOIDANCE_GOAL_YAW)

				# Update planner that we are avoiding an obstacle
				avoidance.data = True
				avoidance_pub.publish(avoidance)
				
				# Update the planner with the new goal
				new_goal = PoseStamped()
				new_goal.pose.position.x = AVOIDANCE_GOAL_X
				new_goal.pose.position.y = AVOIDANCE_GOAL_Y
				new_goal.pose.position.z = AVOIDANCE_GOAL_Z
				new_goal_pub.publish(new_goal)
				
				AVOIDANCE_GOAL_YAW = current_yaw # Dont change the yaw toward the avoidance goal
				goal_x = AVOIDANCE_GOAL_X
				goal_y = AVOIDANCE_GOAL_Y
				goal_z = AVOIDANCE_GOAL_Z
				goal_yaw = AVOIDANCE_GOAL_YAW
				AVOIDANCE_GOAL = True
			else:
				# No avoidance goal
				goal_x = goal.x
				goal_y = goal.y
				goal_z = goal.z	
				goal_yaw = goal.yaw
	
				# Update the planner that we are no longer avoiding an obstacle
				avoidance.data = False
				avoidance_pub.publish(avoidance)

		else:
			goal_x = AVOIDANCE_GOAL_X
			goal_y = AVOIDANCE_GOAL_Y
			goal_z = AVOIDANCE_GOAL_Z	
			goal_yaw = AVOIDANCE_GOAL_YAW
	else:
		# Avoidance not activated, normal goal
		goal_x = goal.x
		goal_y = goal.y
		goal_z = goal.z	
		goal_yaw = goal.yaw

	# Difference between current pose and goal pose
	dx = goal_x - cx
	dy = goal_y - cy
	dz = goal_z - cz
	dyaw = (goal_yaw - current_yaw)

	# Rotate drone
	# How exact should the yaw angle be?
	rotation_finish = abs(dyaw) < ANGLE_DIFF
	
	if (not rotation_finish and not rotate):
		# Rotation is not finished, and we have not already started a rotation
		rotate = True

		# Rotate without linear speed
		target_twist.linear.x = 0
		target_twist.linear.y = 0
		target_twist.linear.z = 0
		angular_velocity_direction = get_shortest_rotation_direction(current_yaw, goal_yaw)
		
		# Twist message
		target_twist.angular.z = angular_velocity * angular_velocity_direction
		# Pose message
		target_pose = current_pose
		# Model state
		target_state.pose = target_pose
		target_state.twist = target_twist
		# Publish ModelState
		while not rospy.is_shutdown():
			connections = pub.get_num_connections()
			if (connections > 0):
				#pub.publish(target_state)
				sendDestination(target_state)
				GOAL_REACHED.data = False
				goal_reached_pub.publish(GOAL_REACHED)
				break
			rospy.Rate(10).sleep()
	
	elif rotation_finish:
		# Rotation is finished
		if rotate: # Check if we are currently rotating
			rotate = False
			# We are close enough, Stop the velocity
			target_twist.linear.x = 0
			target_twist.linear.y = 0
			target_twist.linear.z = 0
			target_twist.angular.z = 0
			# Pose message
			target_pose = Pose()
			target_pose.position.x = cx
			target_pose.position.y = cy
			target_pose.position.z = cz
			target_pose.orientation.x = crx
			target_pose.orientation.y = cry
			target_pose.orientation.z = crz
			target_pose.orientation.w = crw
			# ModelState
			target_state.pose = target_pose
			target_state.twist = target_twist
			# Publish ModelState
			#pub.publish(target_state)
			sendDestination(target_state)
			GOAL_REACHED.data = False
			goal_reached_pub.publish(GOAL_REACHED)
		else:
			# We have rotated and we are not rotating
			# How close should the drone move?
			reach = (abs(dx)**2+abs(dy)**2+abs(dz)**2)**0.5 < POSITION_DIFF
			# First move the position
			if (not reach):
				
				# Velocity messsage, Move towards goal
				if AVOIDANCE_GOAL:
					target_twist.linear.x = avoidance_velocity * dx/(dx**2+dy**2+dz**2+eps)**0.5	
					target_twist.linear.y = avoidance_velocity * dy/(dx**2+dy**2+dz**2+eps)**0.5	
					target_twist.linear.z = avoidance_velocity * dz/(dx**2+dy**2+dz**2+eps)**0.5
				else:
					target_twist.linear.x = linear_velocity * dx/(dx**2+dy**2+dz**2+eps)**0.5	
					target_twist.linear.y = linear_velocity * dy/(dx**2+dy**2+dz**2+eps)**0.5	
					target_twist.linear.z = linear_velocity * dz/(dx**2+dy**2+dz**2+eps)**0.5
		
				# Pose message
				target_pose = current_pose
				target_pose.position.x = drone_position.x
				target_pose.position.y = drone_position.y
				target_pose.position.z = drone_position.z
				target_state.pose = target_pose
				target_state.twist = target_twist
				while not rospy.is_shutdown():
					connections = pub.get_num_connections()
					if (connections > 0):
						# Publish ModelState
						#pub.publish(target_state)
						sendDestination(target_state)
						GOAL_REACHED.data = False
						goal_reached_pub.publish(GOAL_REACHED)
						break
					rospy.Rate(10).sleep()
			else:
				# Avoidance goal reached
				if AVOIDANCE_GOAL:
					AVOIDANCE_GOAL = False
					GOAL_REACHED.data = False
				else:
					# Planner goal reached
					GOAL_REACHED.data = True

				# We are close enough, Stop the velocity
				target_twist.linear.x = 0
				target_twist.linear.y = 0
				target_twist.linear.z = 0
				target_twist.angular.z = 0
				# Pose message
				target_pose = Pose()
				target_pose.position.x = goal_x
				target_pose.position.y = goal_y
				target_pose.position.z = goal_z
				q = quaternion_from_euler(0, 0, goal_yaw)
				target_pose.orientation.x = q[0]
				target_pose.orientation.y = q[1]
				target_pose.orientation.z = q[2]
				target_pose.orientation.w = q[3]
				# ModelState
				target_state.pose = target_pose
				target_state.twist = target_twist
				# Publish ModelState
				#pub.publish(target_state)
				sendDestination(target_state)
				goal_reached_pub.publish(GOAL_REACHED)
			
	

# Init
rospy.init_node("p2p_path", anonymous=True)
#pc_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, pointcloud_callback)

# Parameters. 
AVOIDANCE_ACTIVATED = rospy.get_param("~avoidance_mode")
if AVOIDANCE_ACTIVATED:
    print("Drone avoidance mode is enabled.")
else:
    print("Drone avoidance mode is disabled.")

state_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates)
goal_sub = message_filters.Subscriber("/goal", Goal)
ts = message_filters.ApproximateTimeSynchronizer([state_sub, goal_sub], 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)

# ModelState Subscriber
dynamic_objects = {} # Contain poses and velocities
dynamic_object_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, update_human_positions)

# ModelState
target_state = ModelState()
target_state.model_name = target_model_name

# Drone velocity
linear_velocity = rospy.get_param('/drone_linear_velocity',  0.35)
angular_velocity = rospy.get_param('drone_angular_velocity',  1)
avoidance_velocity = linear_velocity + 0.1

# Velocity message
target_twist = Twist()

# Pose message
target_pose = Pose()

# Update gazebo ModelState publisher 
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

# Goal reached message publisher to planners
goal_reached_pub = rospy.Publisher("/goal_reached", Bool, queue_size=1)

# Update avoidance mode and new avoidance goal
avoidance_pub = rospy.Publisher("/avoidance", Bool, queue_size=1)
new_goal_pub = rospy.Publisher("/new_goal", PoseStamped, queue_size=10)

# rviz
avoidance_goal_publisher = rospy.Publisher("goal_vector", Marker, queue_size=5)

rospy.spin()
