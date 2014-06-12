#! /usr/bin/env python

import time
#import numpy as np
#import numpy.linalg as la
import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy
#import actionlib as al
#from cmnUtils import *
#import moveUtils
#import arWorldModel
#from sensor_msgs.msg import *
#from joyKind import *
#import kinematics_msgs.srv
#import pr2_controllers_msgs.msg
#import trajectory_msgs.msg
#import arm_navigation_msgs.srv
#import pr2_mechanism_msgs.srv
#import std_msgs.msg
import tf
#For rosbag:
#import subprocess
import os
import sys
import traceback
#import signal
#import copy
#import math
import core_tool
#For CUI
import readline
import threading

#Convert a vector to string
def VecToStr(vec,delim=' '):
  return delim.join(map(str,vec))


class TCUITool:

  def __init__(self):

    self.hist_file= '.trick_hist'
    try:
      readline.read_history_file(self.hist_file)
    except IOError:
      pass
    readline.parse_and_bind('tab: complete')
    self.write_history_file= readline.write_history_file

    self.t= core_tool.TCoreTool()


  def __del__(self):
    self.thread_cui.join()
    print 'Save history into ',self.hist_file
    self.write_history_file(self.hist_file)


  def _setup(self):
    self.t.Setup()
    #self.t.mu= self.mu
    #self.t.wm= self.wm

  def Start(self):
    print 'Setup...'
    #self._setup()
    thread_setup= threading.Thread(name='_setup', target=self._setup)
    thread_setup.start()

    self.thread_cui= threading.Thread(name='thread_cui', target=self.Interface)
    self.thread_cui.start()

    thread_setup.join()
    print 'Done: setup'


  def Interface(self):
    last_x= [0.,0.,0., 0.,0.,0.,1.]
    while not rospy.is_shutdown():
      cmd= raw_input('trick or exit | '+self.t.ArmStrS()+' > ').split()

      try:
        if len(cmd)==0:
          continue
        elif cmd[0] == 'exit':
          rospy.signal_shutdown('exit...')
        elif cmd[0] == 'reload' or cmd[0] == 'reloadf':
          old_dict= self.t.__dict__
          reload(core_tool)
          self.t= core_tool.TCoreTool()
          if cmd[0] == 'reload':
            for k,v in old_dict.items():
              self.t.__dict__[k]= v
          elif cmd[0] == 'reloadf':
            #self.t.Setup()
            self.t.mu= old_dict['mu']
            self.t.wm= old_dict['wm']
        elif cmd[0] == 'home':
          self.t.MoveArmsToSide()
        elif cmd[0]=='r':
          self.t.SwitchArm(0)
        elif cmd[0]=='l':
          self.t.SwitchArm(1)
        elif cmd[0]=='calc':
          if cmd[1]=='e2q':
            rot= tf.transformations.quaternion_from_euler(float(cmd[2]), float(cmd[3]), float(cmd[4]))
            print 'Quaternion: ',VecToStr(rot)
          elif cmd[1]=='e2qd':
            e= np.radians(map(float,cmd[2:5]))
            rot= tf.transformations.quaternion_from_euler(e[0], e[1], e[2])
            print 'Quaternion: ',VecToStr(rot)
          elif cmd[1]=='q2e':
            e= tf.transformations.euler_from_quaternion([float(cmd[2]), float(cmd[3]), float(cmd[4]), float(cmd[5])])
            print 'Euler: ',VecToStr(e)
          elif cmd[1]=='q2ed':
            e= tf.transformations.euler_from_quaternion([float(cmd[2]), float(cmd[3]), float(cmd[4]), float(cmd[5])])
            print 'Euler: ',VecToStr(np.degrees(e))
          else:
            print 'Invalid calc-command line: ',' '.join(cmd)
        elif cmd[0]=='bp':
          if len(cmd)==1 or cmd[1]=='show':
            print 'Base points are: ',self.t.base_x
          elif cmd[1]=='addx':
            x= self.t.CartPos()
            self.t.base_x[cmd[2]]= x
            print 'Added to base points[',cmd[2],']: ',VecToStr(self.t.base_x[cmd[2]])
          elif cmd[1]=='addxe':
            xe= self.t.CartPos(self.t.control_frame[self.t.whicharm])
            self.t.base_x[cmd[2]]= xe
            print 'Added to base points[',cmd[2],']: ',VecToStr(self.t.base_x[cmd[2]])
          elif cmd[1]=='add':
            x=[0.0]*7
            x[0:7]= map(float,cmd[3:10])
            self.t.base_x[cmd[2]]= x
            print 'Added to base points[',cmd[2],']: ',VecToStr(self.t.base_x[cmd[2]])
          else:
            print 'Invalid bp-command line: ',' '.join(cmd)
        elif cmd[0]=='lastx':
          if len(cmd)==1 or cmd[1]=='show':
            print 'Last x: ',VecToStr(last_x)
          elif cmd[1]=='set':
            if len(cmd)>=5:  last_x[0:3]= map(float,cmd[2:5])
            if len(cmd)>=9:  last_x[3:7]= map(float,cmd[5:9])
          elif cmd[1]=='arlocal':
            id= int(cmd[2])
            self.t.UpdateAR(id)
            if self.t.IsARAvailable(id):
              l_x= TransformLeftInv(self.t.ARX(id), last_x)
              print 'Local pose of last x on AR ',id,': ',VecToStr(l_x)
            else:
              print 'Error: AR marker not found: ',id
          elif cmd[1]=='bplocal':
            id= cmd[2]
            if id in self.t.base_x:
              l_x= TransformLeftInv(self.t.BPX(id), last_x)
              print 'Local pose of last x on base point ',id,': ',VecToStr(l_x)
            else:
              print 'Error: base point not found: ',id
          else:
            print 'Invalid lastx-command line: ',' '.join(cmd)
        elif cmd[0]=='c':
          if cmd[1]=='mann':
            self.t.ActivateMannController()
          elif cmd[1]=='std':
            self.t.ActivateStdController()
          else:
            print 'Invalid c-command line: ',' '.join(cmd)
        elif cmd[0]=='q':
          q= self.t.mu.arm[self.t.whicharm].getCurrentPosition()  #Joint angles
          print self.t.ArmStr(),'arm joint angles: ',VecToStr(q)
        elif cmd[0]=='x':
          last_x= self.t.CartPos()
          print self.t.ArmStr(),'arm endeffector position: ',VecToStr(last_x)
        elif cmd[0]=='xe':
          last_x= self.t.CartPos(self.t.control_frame[self.t.whicharm])
          print self.t.ArmStr(),'arm extended-endeffector position: ',VecToStr(last_x)
        elif cmd[0]=='ext':
          print self.t.ArmStr(),'arm extension: ',VecToStr(self.t.control_frame[self.t.whicharm])
        elif cmd[0]=='setext':
          if len(cmd)>=4:  self.t.control_frame[self.t.whicharm][0:3]= map(float,cmd[1:4])
          if len(cmd)>=8:  self.t.control_frame[self.t.whicharm][3:7]= map(float,cmd[4:8])
        elif cmd[0]=='moveq':
          if len(cmd)==9:
            dt= 2.0
            q_trg= [0.0]*7
            dt= float(cmd[1])
            q_trg[0:7]= map(float,cmd[2:9])
            self.t.MoveToJointPos(q_trg,dt)
          else:
            print 'Invalid moveq-arguments: ',' '.join(cmd)
        elif cmd[0]=='movex' or cmd[0]=='imovex':
          if len(cmd)==5 or len(cmd)==9:
            dt= 2.0
            x_trg= self.t.CartPos()
            if len(cmd)>=2:  dt= float(cmd[1])
            if len(cmd)>=5:  x_trg[0:3]= map(float,cmd[2:5])
            if len(cmd)>=9:  x_trg[3:7]= map(float,cmd[5:9])
            if cmd[0]=='movex':
              self.t.MoveToCartPos(x_trg,dt)
            else:
              self.t.MoveToCartPosI(x_trg,dt)
          else:
            print 'Invalid arguments: ',' '.join(cmd)
        elif cmd[0]=='movexe' or cmd[0]=='imovexe':
          if len(cmd)==5 or len(cmd)==9:
            dt= 2.0
            x_trg= self.t.CartPos()
            if len(cmd)>=2:  dt= float(cmd[1])
            if len(cmd)>=5:  x_trg[0:3]= map(float,cmd[2:5])
            if len(cmd)>=9:  x_trg[3:7]= map(float,cmd[5:9])
            if cmd[0]=='movexe':
              self.t.MoveToCartPos(x_trg,dt,self.t.control_frame[self.t.whicharm])
            else:
              self.t.MoveToCartPosI(x_trg,dt,self.t.control_frame[self.t.whicharm])
          else:
            print 'Invalid arguments: ',' '.join(cmd)
        elif cmd[0]=='grip':
          pos= float(cmd[1])
          max_effort= 20
          if len(cmd)>=3:  max_effort= float(cmd[2])
          self.t.CommandGripper(pos,max_effort)
        elif cmd[0]=='head':
          dt= float(cmd[1])
          pan= float(cmd[2])
          tilt= float(cmd[3])
          self.t.MoveHead(pan,tilt,dt)
        elif cmd[0]=='ar':
          id= int(cmd[1])
          self.t.UpdateAR(id)
          if self.t.IsARAvailable(id):
            print 'AR ',id,' pose in torso-frame: ',VecToStr(self.t.ARX(id))
        elif cmd[0]=='arraw':
          id= int(cmd[1])
          self.t.UpdateAR(id)
          if self.t.IsARObserved(id):
            print 'AR ',id,' pose in raw: ',VecToStr(self.t.ar_x[id])
        elif cmd[0]=='calib':
          self.t.Calibration()
        elif cmd[0]=='m':
          self.t.ExecuteMotion(cmd[1], cmd[2:])
        elif cmd[0]=='shake':
          if len(cmd)>=3:    self.t.ShakeGripper(shake_Hz=float(cmd[1]),shake_width=float(cmd[2]))
          elif len(cmd)==2:  self.t.ShakeGripper(shake_Hz=float(cmd[1]))
          else:              self.t.ShakeGripper()
        else:
          print 'Invalid command line: ',' '.join(cmd)
      except Exception as e:
        print 'Error(',type(e),'):'
        print '  ',e
        #print '  type: ',type(e)
        #print '  args: ',e.args
        #print '  message: ',e.message
        #print '  sys.exc_info(): ',sys.exc_info()
        print '  Traceback: '
        traceback.print_tb(sys.exc_info()[2])
        print 'Check the command line: ',' '.join(cmd)


if __name__ == '__main__':
  rospy.init_node('cuiToolNode')
  #joy_kind = rospy.get_param('~joy_kind', 'default')
  #base_path = rospy.get_param('~base_path', 'data/bagfiles')
  ct = TCUITool()
  ct.Start()
  #r = rospy.Rate(150)
  #while not rospy.is_shutdown():
    #ct.controlCart()
    #print '.',
    #r.sleep()
  rospy.spin()

