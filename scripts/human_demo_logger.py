#! /usr/bin/env python
import roslib; roslib.load_manifest('lfd_vision')
import rospy
import std_msgs.msg
import math,time

class THumanDemoLogger:
  def __init__(self,file_prefix):
    self.material_amount= 0.0
    self.material_amount_observed= False
    rospy.Subscriber("/color_occupied_ratio", std_msgs.msg.Float64, self.AmountObserver)

    self.xy1_observed= False
    self.xy1= [0,0]
    self.xy2_observed= False
    self.xy2= [0,0]
    rospy.Subscriber("/color_middle_xy1", std_msgs.msg.Int32MultiArray, self.XYObserver1)
    rospy.Subscriber("/color_middle_xy2", std_msgs.msg.Int32MultiArray, self.XYObserver2)

    self.start_time= rospy.Time.now()
    t= time.localtime()
    file_name= '%s%02i%02i%02i%02i%02i%02i.dat' % (file_prefix,t.tm_year%100,t.tm_mon,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec)
    self.log_fp= file(file_name,'w')

  #Observes amount of the material in the cup... just observing the color occupied ration
  def AmountObserver(self, msg):
    self.material_amount_observed= True
    self.material_amount= msg.data

    self.Log()

  def XYObserver1(self, msg):
    self.xy1_observed= True
    self.xy1= msg.data

  def XYObserver2(self, msg):
    self.xy2_observed= True
    self.xy2= msg.data

  def Log(self):
    if self.material_amount_observed and self.xy1_observed and self.xy2_observed:
      t= (rospy.Time.now()-self.start_time).to_sec()
      theta= math.atan2(self.xy1[1]-self.xy2[1],self.xy2[0]-self.xy1[0])
      line= '%f %f %f %f %f %f %f\n' % (t, self.material_amount, theta, self.xy1[0], self.xy1[1], self.xy2[0], self.xy2[1])
      print line,
      self.log_fp.write(line)


if __name__ == '__main__':
  rospy.init_node('human_demo_logger')
  file_prefix= rospy.get_param('~file_prefix', '/tmp/human_demo_logger')
  hdl= THumanDemoLogger(file_prefix)
  rospy.spin()
