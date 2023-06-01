#!/usr/bin/env python
###  Script to calculate number of collisions and length of every collision
###  Will currently print out the metrics when 'true' is written to topic /write_out
###  rostopic pub /write_log std_msgs/Bool "data: true"
###

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
import os
import numpy as np
import csv

DRONE_MODEL_NAME = 'drone'
collisions = {}
collision_durations = []
collision_models = []
number_of_collisions = 0
start_time = None
start_times = {}
intervals = []

human_l = rospy.get_param('/human_width',  1)
human_w = rospy.get_param('/human_width',  1)
human_h = rospy.get_param('/human_height',  1.8)
drone_l = rospy.get_param('/drone_width',  0.4)
drone_w = rospy.get_param('/drone_width',  0.4)
drone_h = rospy.get_param('/drone_height',  0.1)


# Log files
homeDir = os.path.expanduser("~")
logfile_directory = os.path.join(homeDir, 'data',)
logfile_path = os.path.join(logfile_directory, 'collision.csv')
intervals_path = os.path.join(logfile_directory, 'intervals.csv')


def write_log(number_of_collisions, collision_durations, collision_models):

    if not os.path.exists(logfile_directory):
        os.makedirs(logfile_directory)

    with open(logfile_path, 'w') as logfile:
        writer = csv.writer(logfile)
        writer.writerow(['Total duration', 'nr_of_collisions'])
        writer.writerow([np.round(np.sum(collision_durations), 3), number_of_collisions])

    with open(intervals_path, 'w') as intervalfile:
        writer = csv.writer(intervalfile)
        writer.writerow(['start-time', 'duration', 'end-time'])
        for interval in intervals:
            writer.writerow([interval[0], np.round(interval[1] - interval[0], 3), interval[1]])
    
    #rospy.loginfo("Number of collisions: {}\n".format(number_of_collisions))
    #rospy.loginfo("Collisions durations: ")
    #rospy.loginfo(" ".join(str(duration) for duration in collision_durations))
    #rospy.loginfo("\n")
    #rospy.loginfo(" ".join(str(col) for col in collision_models))
    

# Aligned bounding boxes (AABB)
def collision_intersect(drone, human):
  return (
    #drone.minX <= human.maxX
    drone[0][0] <= human[0][1] and
    # drone.maxX >= human.minX 
    drone[0][1] >= human[0][0] and
    # drone.minY <= human.maxY 
    drone[1][0] <= human[1][1] and
    # drone.maxY >= human.minY 
    drone[1][1] >= human[1][0] and
   # drone.minZ <= human.maxZ 
    drone[2][0] <= human[2][1] and
    # drone.maxZ >= human.minZ
    drone[2][1] >= human[2][0]
  )


def check_collision(model_states):
    global number_of_collisions
    global collisions
    global start_times
    global start_time
    if DRONE_MODEL_NAME not in model_states.name:
        return
    
    if start_time is None:
        return

    #rospy.loginfo("check_collision")
    drone_index = model_states.name.index(DRONE_MODEL_NAME)
    drone_position = model_states.pose[drone_index].position
    drone_bounds = np.array([
        [drone_position.x - drone_l/2, drone_position.x + drone_l/2],
        [drone_position.y - drone_w/2, drone_position.y + drone_w/2],
        [drone_position.z, drone_position.z + drone_h]])


    # Loop over other models
    for i in range(len(model_states.name)):
        model_name = model_states.name[i]
        model_position = model_states.pose[i].position

        if model_name != DRONE_MODEL_NAME and model_name[0] == "p":
            human_bounds = np.array([
            [model_position.x - human_l/2, model_position.x + human_l/2],
            [model_position.y - human_w/2, model_position.y + human_w/2],
            [model_position.z, model_position.z + human_h]])

            # Check if there is a collision between the drone and human
            isCollision = collision_intersect(drone_bounds, human_bounds)
            
            # Collision threshold
            if isCollision: 
                # Check if we have already started a collision counter
                if model_name not in collisions:
                    #rospy.loginfo("Adding model name {0}".format(model_name))
                    collisions[model_name] = rospy.Time.now()
                    number_of_collisions += 1

                    #Save the start time for the current collision
                    start_times[model_name] = rospy.Time.now() - start_time

            else:
                # No collision between drone and some model
                if model_name in collisions:
                    collision_duration = rospy.Time.now() - collisions[model_name]
                    collision_durations.append(collision_duration.to_sec())
                    collision_models.append((model_name, collision_duration.to_sec()))
                    
                    start_time_model = start_times[model_name]
                    end_time_model = rospy.Time.now() - start_time
                    intervals.append((start_time_model.to_sec(), end_time_model.to_sec()))
                    
                    del collisions[model_name]
                    del start_times[model_name]
            
    # Add a sleep to slow down the subscriber
    rate.sleep()

def write_log_callback(msg):
    #rospy.loginfo("Writing")
    if msg.data:
        write_log(number_of_collisions, collision_durations, collision_models)

def clock_callback(msg):
    if msg.data:
        #Planner has started, start the internal collision clock
        global start_time
        start_time = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('collision_detection')
    rate = rospy.Rate(15)  # Create an instance of rospy.Rate()
    rospy.Subscriber('/gazebo/model_states', ModelStates, check_collision,  queue_size=1)
    rospy.Subscriber('/write_log', Bool, write_log_callback)
    rospy.Subscriber('/clock_start', Bool, clock_callback)
    while not rospy.is_shutdown():  # Loop until the node is shut down
        rate.sleep()  # Use rospy.Rate to slow down the subscriber
    #write_log(0, [], [])
