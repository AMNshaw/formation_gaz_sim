#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
from mavros_msgs.msg import HomePosition
import pymap3d as pm

class GPS_init:
	
	def __init__(self):
		self.latO = 0
		self.lonO = 0
		self.altO = 0
		self.lat = 0
		self.lon = 0
		self.alt = 0
		self.pose_relative = PoseStamped()
		self.self_pose = PoseStamped()
		self.pose_init = PoseStamped()
		self.pose_initialized = PoseStamped()
		rospy.init_node('gps_init_py', anonymous=True)
		
		self.pose_init_sub = rospy.Subscriber('/uav_init', Int32, self.pose_init_cb)
		self.gps_origin_sub = rospy.Subscriber('/target/mavros/home_position/home', HomePosition, self.gps_origin_cb)
		self.gps_self_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.gps_self_cb)
		self.gps_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_cb)
		
		self.pose_pub = rospy.Publisher('formation/pose', PoseStamped, queue_size=10)

	def init(self, pose_msg):
		pose_msg.pose.position.x = 0
		pose_msg.pose.position.y = 0
		pose_msg.pose.position.z = 0
		pose_msg.header.stamp = rospy.Time()
		pose_msg.header.frame_id = 'map'
	def pose_cb(self, msg):
		self.self_pose = msg
		self.pose_initialized.pose.position.x = self.self_pose.pose.position.x - self.pose_init.pose.position.x + self.pose_relative.pose.position.x
		self.pose_initialized.pose.position.y = self.self_pose.pose.position.y - self.pose_init.pose.position.y + self.pose_relative.pose.position.y
		self.pose_initialized.pose.position.z = self.self_pose.pose.position.z - self.pose_init.pose.position.z
		self.pose_initialized.pose.orientation = self.self_pose.pose.orientation
		self.pose_initialized.header.stamp = rospy.Time.now()
		self.pose_pub.publish(self.pose_initialized)
	def gps_origin_cb(self, msg):
		self.latO = msg.geo.latitude
		self.lonO = msg.geo.longitude
		self.altO = msg.geo.altitude
	def gps_self_cb(self, msg):
		self.lat = msg.latitude
		self.lon = msg.longitude
		self.alt = msg.altitude
		
	def pose_init_cb(self, msg):
		if msg.data == 1:
			self.pose_relative.pose.position.x = pm.geodetic2enu(self.lat, self.lon, self.alt, self.latO, self.lonO, self.altO)[0]
			self.pose_relative.pose.position.y = pm.geodetic2enu(self.lat, self.lon, self.alt, self.latO, self.lonO, self.altO)[1]
			self.pose_relative.pose.position.z = pm.geodetic2enu(self.lat, self.lon, self.alt, self.latO, self.lonO, self.altO)[2]
			self.pose_init = self.self_pose
			rospy.loginfo("Pose initialized")
		else:
			rospy.loginfo("cannot init pose")
	

if __name__ == '__main__':
	
		gps_init = GPS_init()
		gps_init.init(gps_init.self_pose)
		gps_init.init(gps_init.pose_init)
		gps_init.init(gps_init.pose_relative)
		
		rospy.spin()
