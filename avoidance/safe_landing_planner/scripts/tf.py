#!/usr/bin/env python
import rospy

import tf
from tf2_ros import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
import time

def pose(data):
	transform_from = "map"
	transform_to   = "base_link"
	pos = data.pose.position
	ori = data.pose.orientation
	#print(pos)
	orientation = [ori.x, ori.y, ori.z, ori.w]
	position    = [pos.x, pos.y, pos.z]
	br = tf.TransformBroadcaster()
	br.sendTransform(position, orientation, rospy.Time.now(), transform_from, transform_to)

if __name__ == '__main__':
	
	rospy.init_node('dynamic_tf_broadcaster')
	#leaderPose = rospy.wait_for_message('/{}/d_odom'.format(robot_id), Odometry)

	rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose)
	rospy.spin()