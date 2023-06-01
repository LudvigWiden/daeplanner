#! /usr/bin/env python
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
import time

#This variable needs to be globally known
current_pose = Pose()

def odom_callback(odom):
    global current_pose
    current_pose = odom.pose.pose
    
rospy.init_node("spawn_drone")
rospack = rospkg.RosPack()

# Publisher and Subscribers
set_model_state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)


## Spawn the drone into the world
rospy.wait_for_service('/gazebo/spawn_sdf_model')
spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

model_name = 'drone'
package_path = rospack.get_path('drone_gazebo')
model_path = package_path + "/models/drone_fixed/model.sdf"

model_xml = open(model_path, 'r').read()
robot_pose = Pose()

# Extract postion fron rosparam
spawn_pos = eval(rospy.get_param("~spawn_pos"))
robot_pose.position.x = float(spawn_pos[0])
robot_pose.position.y = float(spawn_pos[1])
robot_pose.position.z = float(spawn_pos[2])
reference_frame = 'world'
robot_frame = ''

spawn_model(model_name, model_xml, robot_frame, robot_pose, reference_frame)
current_pose = robot_pose
time.sleep(1) #Sleep to properly spawn the model before trying to move it

odom_subscriber = rospy.Subscriber('/odom', Odometry, odom_callback)

## Move the drone 1 m up into the air to prepare for warmup motion
target_height = current_pose.position.z + 1.0
k_p = 0.5
height_error_tol = 0.05
target_state = ModelState()
target_twist = Twist()

while not rospy.is_shutdown():
    
    height_error = target_height - current_pose.position.z

    # Add velocity to drone to reach 1 meter
    target_twist.linear.z = k_p * height_error

    # Construct velocity command to drone
    target_state.pose = current_pose
    target_state.twist = target_twist
    target_state.model_name = model_name
    set_model_state_pub.publish(target_state) 

    if abs(height_error) < height_error_tol:

        # Stop the drone
        target_twist.linear.z = 0
        
        # Construct velocity command to drone
        target_state.pose = current_pose
        target_state.twist = target_twist
        target_state.model_name = model_name
        set_model_state_pub.publish(target_state) 
        break

    rospy.sleep(0.1)