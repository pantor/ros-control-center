#!/usr/bin/python

import random
import rospy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, TriggerResponse


def calibrate(req):
  rospy.loginfo('Calibrating...')
  return TriggerResponse(True, 'Calibrated.')


if __name__ == '__main__':
  rospy.init_node('imu')
  pub = rospy.Publisher('imu/imu', Imu, queue_size=10)
  s = rospy.Service('imu/calibrate', Trigger, calibrate)

  r = rospy.Rate(2)  # [Hz]

  while not rospy.is_shutdown():
    msg = Imu()
    msg.linear_acceleration.x = random.uniform(0.8, 1.2)
    msg.linear_acceleration.y = random.uniform(0.2, 0.5)
    msg.linear_acceleration.z = random.uniform(0.1, 0.4)
    pub.publish(msg)

    r.sleep()
