#!/usr/bin/python

import random as rd
import rospy

from sensor_msgs.msg import *


if __name__ == '__main__':
  rospy.init_node('sensor_msgs')
  pub_battery_state = rospy.Publisher('BatteryState', BatteryState, queue_size=10)
  pub_camera_info = rospy.Publisher('CameraInfo', CameraInfo, queue_size=10)
  pub_compressed_image = rospy.Publisher('CompressedImage', CameraInfo, queue_size=10)
  pub_fluid_pressure = rospy.Publisher('FluidPressure', CameraInfo, queue_size=10)
  pub_illuminance = rospy.Publisher('Illuminance', Illuminance, queue_size=10)
  pub_image = rospy.Publisher('Image', Image, queue_size=10)
  pub_imu = rospy.Publisher('Imu', Imu, queue_size=10)
  pub_joint_state = rospy.Publisher('JointState', JointState, queue_size=10)
  pub_joy = rospy.Publisher('Joy', Joy, queue_size=10)
  pub_joy_feedback = rospy.Publisher('JoyFeedback', JoyFeedback, queue_size=10)
  pub_magnetic_field = rospy.Publisher('MagneticField', MagneticField, queue_size=10)
  pub_nav_sat_fix = rospy.Publisher('NavSatFix', NavSatFix, queue_size=10)
  pub_nav_sat_status = rospy.Publisher('NavSatStatus', NavSatStatus, queue_size=10)
  pub_point_cloud = rospy.Publisher('PointCloud', PointCloud, queue_size=10)
  pub_range = rospy.Publisher('Range', Range, queue_size=10)
  pub_region_of_interest = rospy.Publisher('RegionOfInterest', RegionOfInterest, queue_size=10)
  pub_relative_humidity = rospy.Publisher('RelativeHumidity', RelativeHumidity, queue_size=10)
  pub_temperature = rospy.Publisher('Temperature', Temperature, queue_size=10)
  pub_time_reference = rospy.Publisher('TimeReference', TimeReference, queue_size=10)

  r = rospy.Rate(2)  # [Hz]

  while not rospy.is_shutdown():

    msg = Imu()
    msg.linear_acceleration.x = rd.uniform(0.8, 1.2)
    msg.linear_acceleration.y = rd.uniform(0.2, 0.5)
    msg.linear_acceleration.z = rd.uniform(0.1, 0.4)
    pub_imu.publish(msg)

    r.sleep()
