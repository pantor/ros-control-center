#!/usr/bin/python

import random as rd
import rospy

from std_msgs.msg import *


if __name__ == '__main__':
  rospy.init_node('std_msgs')
  pub_bool = rospy.Publisher('Bool', Bool, queue_size=10)
  pub_colorrgba = rospy.Publisher('ColorRGBA', ColorRGBA, queue_size=10)
  pub_duration = rospy.Publisher('Duration', Duration, queue_size=10)
  pub_empty = rospy.Publisher('Empty', Empty, queue_size=10)
  pub_float32 = rospy.Publisher('Float32', Float32, queue_size=10)
  pub_float64 = rospy.Publisher('Float64', Float64, queue_size=10)
  pub_int16 = rospy.Publisher('Int16', Int16, queue_size=10)
  pub_int32 = rospy.Publisher('Int32', Int32, queue_size=10)
  pub_int64 = rospy.Publisher('Int64', Int64, queue_size=10)
  pub_int8 = rospy.Publisher('Int8', Int8, queue_size=10)
  pub_string = rospy.Publisher('String', String, queue_size=10)
  pub_time = rospy.Publisher('Time', Time, queue_size=10)
  pub_uint16 = rospy.Publisher('UInt16', UInt16, queue_size=10)
  pub_uint32 = rospy.Publisher('UInt32', UInt32, queue_size=10)
  pub_uint64 = rospy.Publisher('UInt64', UInt64, queue_size=10)
  pub_uint8 = rospy.Publisher('UInt8', UInt8, queue_size=10)

  r = rospy.Rate(2)  # [Hz]

  while not rospy.is_shutdown():
    pub_bool.publish(Bool(data=rd.choice([True, False])))
    pub_colorrgba.publish(ColorRGBA(r=255, g=255, b=255, a=0))
    pub_empty.publish(Empty())
    pub_float32.publish(Float32(data=rd.uniform(-10.0, 10.0)))
    pub_float64.publish(Float64(data=rd.uniform(-10.0, 10.0)))
    pub_string.publish(String(data=rd.choice(['hello', 'world'])))
    pub_uint16.publish(UInt16(data=rd.choice([0, 1, 2])))
    pub_uint32.publish(UInt32(data=rd.choice([0, 1, 2])))
    pub_uint64.publish(UInt64(data=rd.choice([0, 1, 2])))
    pub_uint8.publish(UInt8(data=rd.choice([0, 1, 2])))

    r.sleep()
