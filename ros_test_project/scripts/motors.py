#!/usr/bin/python

import rospy
from std_msgs.msg import Float32


def newControlValueLeft(data):
  rospy.loginfo('Left motor: {}'.format(data.data))


def newControlValueRight(data):
  rospy.loginfo('Right motor: {}'.format(data.data))


if __name__ == "__main__":
  rospy.init_node('motors')
  rospy.Subscriber('motors/left', Float32, newControlValueLeft)
  rospy.Subscriber('motors/right', Float32, newControlValueRight)

  rospy.spin()
