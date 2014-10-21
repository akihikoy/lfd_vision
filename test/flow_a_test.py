#! /usr/bin/env python
import roslib; roslib.load_manifest('pr2_lfd_vision')
import rospy
import tf
import numpy as np
import numpy.linalg as la
import time
import pr2_lfd_vision.msg
import pr2_lfd_vision.srv

import dynamic_reconfigure.client
import sensor_msgs.msg

class TFrameRateController:
  def __init__(self):
    self.dparam_client= dynamic_reconfigure.client.Client('/sentis_tof_m100_1')

    self.dparam_client.update_configuration({'Frame_Rate':40})
    print 'Frame_Rate= %r' % self.dparam_client.get_configuration()['Frame_Rate']

  def __del__(self):
    self.dparam_client.update_configuration({'Frame_Rate':1})
    print 'Frame_Rate= %r' % self.dparam_client.get_configuration()['Frame_Rate']

def BBCountsCallback(msg):
  print msg

if __name__ == '__main__':
  rospy.init_node('test1')

  pub_bb= rospy.Publisher("/flow_analyzer/indexed_bb", pr2_lfd_vision.msg.IndexedBoundingBox)
  sub_bb_counts= rospy.Subscriber("/flow_analyzer/bb_counts", pr2_lfd_vision.msg.Int32Array, BBCountsCallback)

  #Before using rospy.Time.now(), this is necessary!!
  time.sleep(0.2)

  fr_ctrl= TFrameRateController()

  #PlanGrab()
  #rospy.spin()
  while not rospy.is_shutdown():
    bb= pr2_lfd_vision.msg.IndexedBoundingBox()
    bb.active= True
    bb.index= 0
    bb.pose.position.x= 0.0
    bb.pose.position.y= 0.0
    bb.pose.position.z= 0.3
    bb.pose.orientation.x= 0.0
    bb.pose.orientation.y= 0.0
    bb.pose.orientation.z= 0.0
    bb.pose.orientation.w= 1.0
    bb.dimensions= [0.1,0.1,0.1]
    pub_bb.publish(bb)
    time.sleep(2.0)

  del fr_ctrl
