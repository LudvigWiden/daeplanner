# -*- coding: utf-8 -*-
import rospy
import numpy as np

from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.msg import Reference


def send_mrs_trajectory(
        pos_x = [0.0], 
        pos_y = [0.0], 
        pos_z = [0.0], 
        yaw = [0.0], 
        max_speed = 8.0,
        drone_number = 1
    ):
    topic = '/uav1/trajectory_generation/path'
    rospy.wait_for_service(topic)

    try:
        path_service = rospy.ServiceProxy(topic, PathSrv)

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

        # Modify to follow the next point
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


class globalPlanner:
    def __init__(self):
        print("Espere um momento, ja iremos comecar")
        rospy.sleep(1)
        
        # send_mrs_trajectory([20], [10], [5], [0], 3)
        # send_mrs_trajectory([100], [100], [5], [0], 3)
        send_mrs_trajectory([10, 100], [10, 100], [5, 5], [0, 0], 3)


def main():
    rospy.init_node("test_mrs_movement")
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
