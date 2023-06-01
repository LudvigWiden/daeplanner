#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PointStamped
from pigain.srv import QueryDFM, QueryDFMResponse
from matplotlib.animation import FuncAnimation
from gazebo_msgs.msg import ModelState, ModelStates


"""
Keep track of history of dynamic obstacles
"""
class DynamicFrequencyMap:
    def __init__(self):
        self.dfm_query_srv = rospy.Service('dfm_query_server', QueryDFM, self.dfm_callback)
        self.min_x = rospy.get_param('/aeplanner/boundary/min',  [-15, -15, 0])[0]
        self.max_x = rospy.get_param('/aeplanner/boundary/max', [15, 15, 2.5])[0]
        self.min_y = rospy.get_param('/aeplanner/boundary/min',   [-15, -15, 0])[1]
        self.max_y = rospy.get_param('/aeplanner/boundary/max', [15, 15, 2.5])[1]
        self.visualizeDFM = rospy.get_param('/aeplanner/visualizeDFM', False)
        self.square_size = rospy.get_param('/aeplanner/dfm/square_size', 0.5)
        self.round_min_x = self.round(self.min_x)
        self.round_max_x = self.round(self.max_x)
        self.round_min_y = self.round(self.min_y)
        self.round_max_y = self.round(self.max_y)
        self.x_size = self.round_max_x - self.round_min_x
        self.y_size = self.round_max_y - self.round_min_y
        self.array_size_x = int(self.x_size / self.square_size)
        self.array_size_y = int(self.y_size / self.square_size)
        self.grid = np.zeros(shape=(self.array_size_x, self.array_size_y))
        self.dynamic_objects = {}
        self.point_sub = rospy.Subscriber('updateDFM', PointStamped, self.update_frequency_map_callback)
        self.dynamic_object_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.update_human_positions)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_timer_callback)

        # Visualize the grid
        if self.visualizeDFM:
            self.fig, self.ax = plt.subplots()
            self.im = self.ax.imshow(self.grid, cmap='Blues', vmin=0, vmax=1, origin='upper', extent=[0, self.y_size, 0, self.x_size])
            plt.colorbar(self.im, ax=self.ax, label='Value')
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.set_title('Dynamic Frequency Map')
            self.ani = FuncAnimation(self.fig, self.update_plot)
            plt.show()


        
    # Keep track of human positions
    def update_human_positions(self, model_states):
        for model in model_states.name:
            if "person_walking" in model:
                person_idx = model_states.name.index(model)
                person_pose = model_states.pose[person_idx]
                self.dynamic_objects[model] = person_pose

    # Update animation               
    def update_plot(self, frame):
        normalized_grid = self.normalize_grid()
        self.im.set_array(normalized_grid)
        return self.im

    # Round map boundaries 
    def round(self, num):
        if num < 0:
            return int(np.floor(num))
        else:
            return int(np.ceil(num))

    # Normalize frequency of observed obstacles
    def normalize_grid(self):
        max_val = np.max(self.grid)
        if max_val != 0:
            normalized_grid = self.grid / max_val
        else:
            normalized_grid = self.grid
        return normalized_grid

    # Return the normalized score of a requested position
    def dfm_callback(self, req):
        normalized_grid = self.normalize_grid()
        x = int((req.point.x - self.round_min_x) / self.square_size)
        y = int((req.point.y - self.round_min_y) / self.square_size)
        response = QueryDFMResponse()
        response.score =  normalized_grid[x, y]
        return response

    # Update the grid with new observations from topic
    def update_frequency_map_callback(self, msg):
        observation = (msg.point.x, msg.point.y)
        x = int((observation[0] - self.round_min_x) / self.square_size)
        y = int((observation[1] - self.round_min_y) / self.square_size)        
        self.grid[x, y] += 1

    # Update the grid with new observations from timer callback
    def update_timer_callback(self, timer):
        for person in self.dynamic_objects.values():
            observation = (person.position.x, person.position.y)
            x = int((observation[0] - self.round_min_x) / self.square_size)
            y = int((observation[1] - self.round_min_y) / self.square_size)  
            self.grid[x, y] += 1



if __name__ == '__main__':
    rospy.init_node('dynamic_frequency_map', anonymous=True)
    DFM = DynamicFrequencyMap()
    rospy.spin()