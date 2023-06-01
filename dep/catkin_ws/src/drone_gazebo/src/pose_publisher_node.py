#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def odom_callback(odom):
    usePoseStamped = rospy.get_param("~usePoseStamped")
    if(usePoseStamped):
        #Needed in AEP with DEP-motion planner
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
    else:
        #Needed in NBVP with DEP-motion planner
        pose = PoseWithCovarianceStamped()
        pose.header = odom.header
        pose.pose = odom.pose

    pose_pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('pose_publisher_node')
    usePoseStamped = rospy.get_param("~usePoseStamped")
    if(usePoseStamped):
        pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
    else:
        pose_pub = rospy.Publisher('/pose', PoseWithCovarianceStamped, queue_size=10)

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

