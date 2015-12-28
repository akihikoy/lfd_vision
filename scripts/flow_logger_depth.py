#! /usr/bin/env python
import roslib; roslib.load_manifest('lfd_vision')
import rospy
import std_msgs.msg
import math,time

class TFlowLogger:
  def __init__(self,file_prefix):
    self.observed= [False]*1

    self.speed_angle= [0.0,0.0]
    self.speed_accum= 0.0
    rospy.Subscriber("/flow_speed_angle", std_msgs.msg.Float64MultiArray, self.FlowObserver)

    t= time.localtime()
    file_name= '%s%02i%02i%02i%02i%02i%02i.dat' % (file_prefix,t.tm_year%100,t.tm_mon,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec)
    self.log_fp= file(file_name,'w')

    self.start_time= rospy.Time.now()
    self.time= [self.start_time]*1

  def FlowObserver(self, msg):
    self.observed[0]= True
    t= rospy.Time.now()
    dt= (t-self.time[0]).to_sec()
    self.time[0]= t
    if msg.data[1]>1.1 and msg.data[1]<1.9 and msg.data[0]>3.0:
      self.speed_angle= msg.data
    else:
      self.speed_angle= [0.0]*2
    self.speed_accum+= self.speed_angle[0]*dt

    self.Log()

  def Log(self):
    if False in self.observed:  return
    t= (rospy.Time.now()-self.start_time).to_sec()
    line= '%f %f %f %f\n' % (t, self.speed_angle[0], self.speed_angle[1],
                                   self.speed_accum)
    print line,
    self.log_fp.write(line)

if __name__ == '__main__':
  rospy.init_node('flow_logger')
  file_prefix= rospy.get_param('~file_prefix', '/tmp/flow_logger')
  fl= TFlowLogger(file_prefix)
  rospy.spin()
