#!/usr/bin/python

import random as rd
import rospy

from std_srvs.srv import *


def empty(req):
  return EmptyResponse()


def set_bool(req):
  return SetBoolResponse(req.data, 'Message Text.')


def trigger(req):
  rospy.loginfo('Triggered.')
  success = rd.choice([True, False])
  return TriggerResponse(success, 'Message Success.' if success else 'Message Failure')


if __name__ == '__main__':
  rospy.init_node('std_srvs')
  ser_empty = rospy.Service('Empty', Empty, empty)
  ser_set_bool = rospy.Service('SetBool', SetBool, set_bool)
  ser_trigger = rospy.Service('Trigger', Trigger, trigger)

  rospy.spin()
