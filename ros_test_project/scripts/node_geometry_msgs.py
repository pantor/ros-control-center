#!/usr/bin/python

import random as rd
import rospy

from geometry_msgs.msg import *


if __name__ == '__main__':
  rospy.init_node('geometry_msgs')
  pub_accel = rospy.Publisher('Accel', Accel, queue_size=10)
  pub_accel_stamped = rospy.Publisher('AccelStamped', AccelStamped, queue_size=10)
  pub_accel_cov = rospy.Publisher('AccelWithCovariance', AccelWithCovariance, queue_size=10)
  pub_accel_cov_stamped = rospy.Publisher('AccelWithCovarianceStamped', AccelWithCovarianceStamped, queue_size=10)
  pub_inertia = rospy.Publisher('Inertia', Inertia, queue_size=10)
  pub_point = rospy.Publisher('Point', Point, queue_size=10)
  pub_point32 = rospy.Publisher('Point32', Point32, queue_size=10)
  pub_point_stamped = rospy.Publisher('PointStamped', PointStamped, queue_size=10)
  pub_polygon = rospy.Publisher('Polygon', Polygon, queue_size=10)
  pub_pose = rospy.Publisher('Pose', Pose, queue_size=10)
  pub_pose2d = rospy.Publisher('Pose2D', Pose2D, queue_size=10)
  pub_pose_array = rospy.Publisher('PoseArray', PoseArray, queue_size=10)
  pub_pose_stamped = rospy.Publisher('PoseStamped', Pose, queue_size=10)
  pub_quaternion = rospy.Publisher('Quaternion', Quaternion, queue_size=10)
  pub_quaternion_stamped = rospy.Publisher('QuaternionStamped', QuaternionStamped, queue_size=10)
  pub_transform = rospy.Publisher('Transform', Transform, queue_size=10)
  pub_twist = rospy.Publisher('Twist', Twist, queue_size=10)
  pub_vector3 = rospy.Publisher('Vector3', Vector3, queue_size=10)
  pub_vector3_stamped = rospy.Publisher('Vector3Stamped', Vector3Stamped, queue_size=10)
  pub_wrench = rospy.Publisher('Wrench', Wrench, queue_size=10)
  pub_wrench_stamped = rospy.Publisher('WrenchStamped', WrenchStamped, queue_size=10)

  r = rospy.Rate(2)  # [Hz]

  while not rospy.is_shutdown():

    msg = Point(x=rd.uniform(-10, 10), y=rd.uniform(-10, 10), z=rd.uniform(-10, 10))
    pub_point.publish(msg)

    r.sleep()
