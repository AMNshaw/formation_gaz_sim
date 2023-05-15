#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import pymap3d as pm
from mavros_msgs.srv import StreamRate

latO = 0
lonO = 0
altO = 0

lat = 0
lon = 0
alt = 0

pose_relative = PoseStamped()
gps_pose = PoseStamped()
gps_pose_init = PoseStamped()
gps_pose_initialized = PoseStamped()
time_leader = rospy.Time()

def time_cb(msg):
	global time_leader
	time_leader = msg.header.stamp

def gps_origin_cb(msg): 
	global latO, lonO, altO
	latO = msg.latitude
	lonO = msg.longitude
	altO = msg.altitude

def gps_self_cb(msg):
	global lat, lon, alt
	lat = msg.latitude
	lon = msg.longitude
	alt = msg.altitude

def gps_pose_cb(msg):
	global gps_pose
	gps_pose = msg

def pose_init_cb(msg):
	global gps_pose_init, pose_relative
	if msg.data == 1:
		pose_relative.pose.position.x = pm.geodetic2enu(lat, lon, alt, latO, lonO, altO)[0]
		pose_relative.pose.position.y = pm.geodetic2enu(lat, lon, alt, latO, lonO, altO)[1]
		pose_relative.pose.position.z = pm.geodetic2enu(lat, lon, alt, latO, lonO, altO)[2]
		gps_pose_init = gps_pose
		rospy.loginfo("pose initialized")
	else:
		rospy.loginfo("cannot init pose")

if __name__ == '__main__':
	
        rospy.init_node('gps_init_py')
        pose_init_sub = rospy.Subscriber('/uav_init', Int32, pose_init_cb)
        gps_origin_sub = rospy.Subscriber('/target/mavros/global_position/global', NavSatFix, gps_origin_cb)
        gps_self_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, gps_self_cb)
        gps_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, gps_pose_cb)
        leader_pose_sub = rospy.Subscriber('/leader_pose', PoseStamped, time_cb)

        gps_pose_pub = rospy.Publisher('mavros/local_position/pose_initialized', PoseStamped, queue_size=10)
        
        gps_pose_initialized.pose.position.x = 0
        gps_pose_initialized.pose.position.y = 0
        gps_pose_initialized.pose.position.z = 0
        gps_pose_initialized.header.stamp = rospy.Time()
        gps_pose_initialized.header.frame_id = 'map';

        rospy.loginfo("gps_init node constructed")
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            gps_pose_initialized.pose.position.x = gps_pose.pose.position.x - gps_pose_init.pose.position.x + pose_relative.pose.position.x
            gps_pose_initialized.pose.position.y = gps_pose.pose.position.y - gps_pose_init.pose.position.y + pose_relative.pose.position.y
            gps_pose_initialized.pose.position.z = gps_pose.pose.position.z - gps_pose_init.pose.position.z 
            gps_pose_initialized.pose.orientation = gps_pose.pose.orientation
            gps_pose_initialized.header.stamp = time_leader


            gps_pose_pub.publish(gps_pose_initialized)

            rate.sleep()
        rospy.spin()
