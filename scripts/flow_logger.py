#! /usr/bin/env python
import roslib; roslib.load_manifest('lfd_vision')
import rospy
import std_msgs.msg
import math,time

class TFlowLogger:
  def __init__(self,file_prefix):
    self.observed= [False]*2

    self.amount= 0.0
    self.amount_prev= 0.0
    self.amount_diff= 0.0
    rospy.Subscriber("/color_occupied_ratio", std_msgs.msg.Float64, self.AmountObserver)

    self.speed_angle= [0.0,0.0]
    self.speed_accum= 0.0
    rospy.Subscriber("/flow_speed_angle", std_msgs.msg.Float64MultiArray, self.FlowObserver)

    t= time.localtime()
    file_name= '%s%02i%02i%02i%02i%02i%02i.dat' % (file_prefix,t.tm_year%100,t.tm_mon,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec)
    self.log_fp= file(file_name,'w')

    self.start_time= rospy.Time.now()
    self.time= [self.start_time]*2

  #Observes amount of the material in the cup... just observing the color occupied ration
  def AmountObserver(self, msg):
    self.observed[0]= True
    t= rospy.Time.now()
    dt= (t-self.time[0]).to_sec()
    self.time[0]= t
    self.amount= msg.data
    self.amount_diff= (self.amount-self.amount_prev)/dt
    self.amount_prev= self.amount
    self.Log()

  def FlowObserver(self, msg):
    self.observed[1]= True
    t= rospy.Time.now()
    dt= (t-self.time[1]).to_sec()
    self.time[1]= t
    if msg.data[1]>1.1 and msg.data[1]<1.9 and msg.data[0]>20.0:
      self.speed_angle= msg.data
    else:
      self.speed_angle= [0.0]*2
    self.speed_accum+= self.speed_angle[0]*dt

  def Log(self):
    if False in self.observed:  return
    t= (rospy.Time.now()-self.start_time).to_sec()
    line= '%f %f %f %f %f %f\n' % (t, self.amount, self.speed_angle[0], self.speed_angle[1],
                                   self.amount_diff, self.speed_accum)
    print line,
    self.log_fp.write(line)

if __name__ == '__main__':
  rospy.init_node('flow_logger')
  file_prefix= rospy.get_param('~file_prefix', '/tmp/flow_logger')
  fl= TFlowLogger(file_prefix)
  rospy.spin()
