#! /usr/bin/env python

import time
import numpy as np
import numpy.linalg as la
import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy
#import actionlib as al
from cmnUtils import *
import moveUtils
import arWorldModel
#from sensor_msgs.msg import *
#from joyKind import *
#import kinematics_msgs.srv
import pr2_controllers_msgs.msg
import trajectory_msgs.msg
import arm_navigation_msgs.srv
import pr2_mechanism_msgs.srv
import std_msgs.msg
import tf
#For rosbag:
#import subprocess
import os
import sys
import traceback
#import signal
import copy
import math
#For CUI
import readline
import threading

def AskYesNo():
  return ask_yes_no()

#There must be a planning scene or FK / IK crashes
def SetupPlanningScene():
  print 'Waiting for set planning scene service...'
  rospy.wait_for_service('/environment_server/set_planning_scene_diff')
  set_plan = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
  req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
  set_plan(req)
  print 'OK'

#Convert a vector to string
def VecToStr(vec,delim=' '):
  return delim.join(map(str,vec))

#Quaternion to 3x3 rotation matrix
def QToRot(q):
  return tf.transformations.quaternion_matrix(q)[:3,:3]

#Quaternion to 3x3 rotation matrix
def RotToQ(R):
  M = tf.transformations.identity_matrix()
  M[:3,:3] = R
  return tf.transformations.quaternion_from_matrix(M)

#Convert a pose, x,y,z,quaternion(qx,qy,qz,qw) to pos (x,y,z) and 3x3 rotation matrix
def XToPosRot(x):
  p = np.array(x[0:3])
  R = tf.transformations.quaternion_matrix(x[3:7])[:3,:3]
  return p, R

#Convert pos p=(x,y,z) and 3x3 rotation matrix R to a pose, x,y,z,quaternion(qx,qy,qz,qw)
def PosRotToX(p,R):
  M = tf.transformations.identity_matrix()
  M[:3,:3] = R
  x = list(p)+[0.0]*4
  x[3:7] = tf.transformations.quaternion_from_matrix(M)
  return x

#This solves for x in "x_r = x_l * x", i.e. return "inv(x_l)*x_r"
#For example, get a local pose of x_r in the x_l frame
#x_* are [x,y,z,quaternion] form
def TransformLeftInv(x_l,x_r):
  pl,Rl= XToPosRot(x_l)
  pr,Rr= XToPosRot(x_r)
  p= np.dot(Rl.T, (pr-pl))
  R= np.dot(Rl.T, Rr)
  return PosRotToX(p,R)

#This solves for trans_x in "x_l = trans_x * x_r", i.e. return "x_l*inv(x_r)"
#For example, get a transformation, x_r to x_l
#x_* are [x,y,z,quaternion] form
def TransformRightInv(x_l,x_r):
  pl,Rl= XToPosRot(x_l)
  pr,Rr= XToPosRot(x_r)
  Rt= np.dot(Rl, Rr.T)
  pt= pl-np.dot(Rt,pr)
  return PosRotToX(pt,Rt)

#Compute "x2 * x1"; x* are [x,y,z,quaternion] form
def Transform(x2, x1):
  p2,R2= XToPosRot(x2)
  p1,R1= XToPosRot(x1)
  p= np.dot(R2,p1)+p2
  R= np.dot(R2, R1)
  return PosRotToX(p,R)

#Return the interpolation from x1 to x2 with N points
#p1 is not included
def CartPosInterpolation(x1,x2,N):
  p1,R1= XToPosRot(x1)
  p2,R2= XToPosRot(x2)
  dp= (p2-p1)/float(N)
  trans_R= np.dot(R2,R1.T)
  w= InvRodrigues(trans_R)
  traj=[]
  for t in range(N):
    R= np.dot(Rodrigues(float(t+1)/float(N)*w),R1)
    p1= p1+dp
    traj.append(PosRotToX(p1,R))
  return traj


class TCUITool:

  def __init__(self):

    self.hist_file= '.trick_hist'
    try:
      readline.read_history_file(self.hist_file)
    except IOError:
      pass
    readline.parse_and_bind('tab: complete')

    self.motion_path_prefix= 'motions.m_'
    self.loaded_motions= []

    #self.init = True
    #self.reset = True
    #self.curr_x=[[0.0]*7, [0.0]*7]
    self.whicharm= 1

    #self.linear_speed = [0.01,0.01,0.01]
    #self.angular_speed = [0.05,0.05,0.05]
    #self.time_step = 0.05
    self.control_time_step= 0.001
    #self.gripper_max_effort = [12.0,15.0]  # gripper's max effort when closing (right,left). -1: do not limit, 50: close gently
    #self.gripper_max_effort_strong = [50.0,50.0]
    self.vel_limits = [0.8, 0.8, 2.0, 2.0, 3.0, 3.0, 10.0]

    ##Extended IK
    #self.using_extended_ik = True
    self.control_frame= [[0.16,0.0,0.0, 0.0,0.0,0.0,1.0]]*2


    self.flow_control_theta_max= math.pi*0.8
    self.flow_control_dtheta_max= math.pi*0.1
    self.flow_control_dtheta_min= -math.pi*0.1
    self.flow_control_time_step= 0.01
    self.flow_control_gain_p= 50.0
    self.flow_control_gain_d= 1.0
    self.flow_move_back_duration= 3.0

    self.flow_shake_freq_max= 2.0
    self.flow_shake_freq_min= 0.1
    self.flow_shake_gain_p= 500.0
    self.flow_shake_gain_d= 10.0
    self.flow_shake_width= 0.04
    self.flow_shake_axis= [0.0,0.0,1.0]

    self.ar_x= {}
    self.x_marker_to_torso= []
    self.base_x= {}

    ##Current state:
    #self.deadman = False
    #self.dx_vec = [0.0, 0.0, 0.0]  # linear velocity
    #self.w_vec = [0.0, 0.0, 0.0]  # angular velocity
    #self.motion_request = MotionKind.NOTHING  # predefined motion

    joint_thresh=0.001
    self.joint_bounds = [joint_thresh]*10

    self.is_mannequin= [False,False]
    self.standard_controllers = ['r_arm_controller', 'l_arm_controller']
    self.mannequin_controllers = ['r_arm_controller_loose', 'l_arm_controller_loose']

    self.r_pub = rospy.Publisher("/r_arm_controller_loose/command", trajectory_msgs.msg.JointTrajectory)
    rospy.Subscriber("/r_arm_controller_loose/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.RightJointStateCallback)
    self.l_pub = rospy.Publisher("/l_arm_controller_loose/command", trajectory_msgs.msg.JointTrajectory)
    rospy.Subscriber("/l_arm_controller_loose/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.LeftJointStateCallback)

    self.head_pub = rospy.Publisher("/head_traj_controller/command", trajectory_msgs.msg.JointTrajectory)
    self.head_joint_names= ['head_pan_joint', 'head_tilt_joint']

    self.material_amount= 0
    self.material_amount_observed= False
    rospy.Subscriber("/color_occupied_ratio", std_msgs.msg.Float64, self.AmountObserver)


  def __del__(self):
    #self.stopRecord()
    self.thread_cui.join()
    readline.write_history_file(self.hist_file)


  def _setup(self):
    SetupPlanningScene()
    self.mu= moveUtils.MoveUtils()
    self.wm= arWorldModel.ARWorldModel()


  def Start(self):
    print 'Setup...'
    #self._setup()
    thread_setup= threading.Thread(name='_setup', target=self._setup)
    thread_setup.start()

    self.thread_cui= threading.Thread(name='thread_cui', target=self.Interface)
    self.thread_cui.start()

    thread_setup.join()
    print 'Done: setup'


  def ArmStr(self):
    return 'Right' if self.whicharm==0 else 'Left'
  def ArmStrS(self):
    return 'R' if self.whicharm==0 else 'L'


  def Interface(self):
    last_x= [0.,0.,0., 0.,0.,0.,1.]
    while not rospy.is_shutdown():
      cmd= raw_input('Ctrl+C & ENTER to quit | '+self.ArmStrS()+' > ').split()

      try:
        if len(cmd)==0:
          continue
        elif cmd[0] == 'home':
          self.MoveArmsToSide()
        elif cmd[0]=='r':
          self.SwitchArm(0)
        elif cmd[0]=='l':
          self.SwitchArm(1)
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
            print 'Base points are: ',self.base_x
          elif cmd[1]=='addx':
            x= self.CartPos()
            self.base_x[cmd[2]]= x
            print 'Added to base points[',cmd[2],']: ',VecToStr(self.base_x[cmd[2]])
          elif cmd[1]=='addxe':
            xe= self.CartPos(self.control_frame[self.whicharm])
            self.base_x[cmd[2]]= xe
            print 'Added to base points[',cmd[2],']: ',VecToStr(self.base_x[cmd[2]])
          elif cmd[1]=='add':
            x=[0.0]*7
            x[0:7]= map(float,cmd[3:10])
            self.base_x[cmd[2]]= x
            print 'Added to base points[',cmd[2],']: ',VecToStr(self.base_x[cmd[2]])
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
            self.UpdateAR(id)
            if self.IsARAvailable(id):
              l_x= TransformLeftInv(self.ARX(id), last_x)
              print 'Local pose of last x on AR ',id,': ',VecToStr(l_x)
            else:
              print 'Error: AR marker not found: ',id
          elif cmd[1]=='bplocal':
            id= cmd[2]
            if id in self.base_x:
              l_x= TransformLeftInv(self.BPX(id), last_x)
              print 'Local pose of last x on base point ',id,': ',VecToStr(l_x)
            else:
              print 'Error: base point not found: ',id
          else:
            print 'Invalid lastx-command line: ',' '.join(cmd)
        elif cmd[0]=='c':
          if cmd[1]=='mann':
            self.ActivateMannController()
          elif cmd[1]=='std':
            self.ActivateStdController()
          else:
            print 'Invalid c-command line: ',' '.join(cmd)
        elif cmd[0]=='q':
          q= self.mu.arm[self.whicharm].getCurrentPosition()  #Joint angles
          print self.ArmStr(),'arm joint angles: ',VecToStr(q)
        elif cmd[0]=='x':
          last_x= self.CartPos()
          print self.ArmStr(),'arm endeffector position: ',VecToStr(last_x)
        elif cmd[0]=='xe':
          last_x= self.CartPos(self.control_frame[self.whicharm])
          print self.ArmStr(),'arm extended-endeffector position: ',VecToStr(last_x)
        elif cmd[0]=='ext':
          print self.ArmStr(),'arm extension: ',VecToStr(self.control_frame[self.whicharm])
        elif cmd[0]=='setext':
          if len(cmd)>=4:  self.control_frame[self.whicharm][0:3]= map(float,cmd[1:4])
          if len(cmd)>=8:  self.control_frame[self.whicharm][3:7]= map(float,cmd[4:8])
        elif cmd[0]=='moveq':
          if len(cmd)==9:
            dt= 2.0
            q_trg= [0.0]*7
            dt= float(cmd[1])
            q_trg[0:7]= map(float,cmd[2:9])
            self.MoveToJointPos(q_trg,dt)
          else:
            print 'Invalid moveq-arguments: ',' '.join(cmd)
        elif cmd[0]=='movex' or cmd[0]=='imovex':
          if len(cmd)==5 or len(cmd)==9:
            dt= 2.0
            x_trg= self.CartPos()
            if len(cmd)>=2:  dt= float(cmd[1])
            if len(cmd)>=5:  x_trg[0:3]= map(float,cmd[2:5])
            if len(cmd)>=9:  x_trg[3:7]= map(float,cmd[5:9])
            if cmd[0]=='movex':
              self.MoveToCartPos(x_trg,dt)
            else:
              self.MoveToCartPosI(x_trg,dt)
          else:
            print 'Invalid arguments: ',' '.join(cmd)
        elif cmd[0]=='movexe' or cmd[0]=='imovexe':
          if len(cmd)==5 or len(cmd)==9:
            dt= 2.0
            x_trg= self.CartPos()
            if len(cmd)>=2:  dt= float(cmd[1])
            if len(cmd)>=5:  x_trg[0:3]= map(float,cmd[2:5])
            if len(cmd)>=9:  x_trg[3:7]= map(float,cmd[5:9])
            if cmd[0]=='movexe':
              self.MoveToCartPos(x_trg,dt,self.control_frame[self.whicharm])
            else:
              self.MoveToCartPosI(x_trg,dt,self.control_frame[self.whicharm])
          else:
            print 'Invalid arguments: ',' '.join(cmd)
        elif cmd[0]=='grip':
          pos= float(cmd[1])
          max_effort= 20
          if len(cmd)>=3:  max_effort= float(cmd[2])
          self.CommandGripper(pos,max_effort)
        elif cmd[0]=='head':
          dt= float(cmd[1])
          pan= float(cmd[2])
          tilt= float(cmd[3])
          self.MoveHead(pan,tilt,dt)
        elif cmd[0]=='ar':
          id= int(cmd[1])
          self.UpdateAR(id)
          if self.IsARAvailable(id):
            print 'AR ',id,' pose in torso-frame: ',VecToStr(self.ARX(id))
        elif cmd[0]=='arraw':
          id= int(cmd[1])
          self.UpdateAR(id)
          if self.IsARObserved(id):
            print 'AR ',id,' pose in raw: ',VecToStr(self.ar_x[id])
        elif cmd[0]=='calib':
          self.Calibration()
        elif cmd[0]=='m':
          self.ExecuteMotion(cmd[1], cmd[2:])
        elif cmd[0]=='shake':
          if len(cmd)>=3:    self.ShakeGripper(shake_Hz=float(cmd[1]),shake_width=float(cmd[2]))
          elif len(cmd)==2:  self.ShakeGripper(shake_Hz=float(cmd[1]))
          else:              self.ShakeGripper()
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


  def MoveArmsToSide(self):
    #self.init = True
    #self.reset = True
    #time.sleep(0.1)
    self.mu.arm[0].traj_client.wait_for_result()
    self.mu.arm[0].cart_exec.traj_client.wait_for_result()
    self.mu.arm[1].traj_client.wait_for_result()
    self.mu.arm[1].cart_exec.traj_client.wait_for_result()
    target = [[],[]]
    target[1] = [2.115, -0.020, 1.640, -2.070, 1.640, -1.680, 1.398]
    target[0] = [-2.115, 0.020, -1.640, -2.070, -1.640, -1.680, 1.398]
    threshold = 0.01
    print "Moving to home position"
    print "Left arm..."
    self.mu.arm[1].commandGripper(0.08, -1)
    q = np.array(self.mu.arm[1].getCurrentPosition())
    diff = la.norm(np.array(target[1])-q)
    if diff > threshold:
      self.mu.arm[1].moveToJointAngle(target[1], 4.0)
    else:
      print "Left arm is already at the goal ({diff})".format(diff=diff)
    print "Right arm..."
    self.mu.arm[0].commandGripper(0.08, -1)
    q = np.array(self.mu.arm[0].getCurrentPosition())
    diff = la.norm(np.array(target[0])-q)
    if diff > threshold:
      self.mu.arm[0].moveToJointAngle(target[0], 4.0)
    else:
      print "Right arm is already at the goal ({diff})".format(diff=diff)
    print "Done"


  def SetupSwitchControl(self):
    if not ('switch_control' in self.__dict__):
      print 'Waiting for switch controller service...'
      rospy.wait_for_service('pr2_controller_manager/switch_controller')
      self.switch_control = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pr2_mechanism_msgs.srv.SwitchController, persistent=True)
      print 'OK'

      self.switch_req = pr2_mechanism_msgs.srv.SwitchControllerRequest()
      self.switch_req.strictness = pr2_mechanism_msgs.srv.SwitchControllerRequest.BEST_EFFORT

  def ActivateMannController(self):
    self.SetupSwitchControl()
    if not self.is_mannequin[self.whicharm]:
      self.switch_req.stop_controllers = [self.standard_controllers[self.whicharm]]
      self.switch_req.start_controllers = [self.mannequin_controllers[self.whicharm]]
      resp = self.switch_control(self.switch_req)
      self.is_mannequin[self.whicharm]= True
      print self.ArmStr(),'arm uses mannequin controller'

  def ActivateStdController(self):
    self.SetupSwitchControl()
    if self.is_mannequin[self.whicharm]:
      self.switch_req.stop_controllers = [self.mannequin_controllers[self.whicharm]]
      self.switch_req.start_controllers = [self.standard_controllers[self.whicharm]]
      resp = self.switch_control(self.switch_req)
      self.is_mannequin[self.whicharm]= False
      print self.ArmStr(),'arm uses standard controller'

  #Borrowed from pr2_lfd_utils/src/recordInteraction.py
  def RightJointStateCallback(self, msg):
    if self.is_mannequin[0]:
      #max_error = max([abs(x) for x in msg.error.positions])
      exceeded = [abs(x) > y for x,y in zip(msg.error.positions, self.joint_bounds)]

      if any(exceeded):
        # Copy our current state into the commanded state
        cmd = trajectory_msgs.msg.JointTrajectory()
        cmd.header.stamp = msg.header.stamp
        cmd.joint_names = msg.joint_names
        cmd.points.append( trajectory_msgs.msg.JointTrajectoryPoint())
        cmd.points[0].time_from_start = rospy.Duration(.125)
        cmd.points[0].positions = msg.actual.positions
        self.r_pub.publish(cmd)

  #Borrowed from pr2_lfd_utils/src/recordInteraction.py
  def LeftJointStateCallback(self, msg):
    if self.is_mannequin[1]:
      #max_error = max([abs(x) for x in msg.error.positions])
      exceeded = [abs(x) > y for x,y in zip(msg.error.positions, self.joint_bounds)]

      if any(exceeded):
        # Copy our current state into the commanded state
        cmd = trajectory_msgs.msg.JointTrajectory()
        cmd.header.stamp = msg.header.stamp
        cmd.joint_names = msg.joint_names
        cmd.points.append( trajectory_msgs.msg.JointTrajectoryPoint())
        cmd.points[0].time_from_start = rospy.Duration(.125)
        cmd.points[0].positions = msg.actual.positions
        self.l_pub.publish(cmd)


  #Observes amount of the material in the cup... just observing the color occupied ration
  def AmountObserver(self, msg):
    self.material_amount_observed= True
    self.material_amount= msg.data


  def SwitchArm(self,arm):
    self.whicharm= arm
    print self.ArmStr(),'arm is selected'


  def MoveToJointPos(self, q_trg, dt=2.0, blocking=False):
    i= self.whicharm
    self.mu.arm[i].moveToJointAngle(q_trg, dt, blocking)


  def CartPos(self,x_ext=[]):
    i= self.whicharm
    q= self.mu.arm[i].getCurrentPosition()  #Joint angles
    x= self.mu.arm[i].cart_exec.makeFKRequest(q)
    if len(x_ext)==0:
      return x
    else:
      xe= Transform(x,x_ext)
      #l_p,l_R= XToPosRot(x_ext)
      #p,R= XToPosRot(x)
      #pe= np.dot(R,l_p) + p
      #Re= np.dot(R,l_R)
      #xe= PosRotToX(pe,Re)
      return xe


  def MakeIKRequest(self, x_trg, x_ext=[], v_start_angles=[]):
    i = self.whicharm
    if len(v_start_angles)==0:
      start_angles = self.mu.arm[i].getCurrentPosition()
    else:
      start_angles = v_start_angles

    x_trg[3:7] = x_trg[3:7] / la.norm(x_trg[3:7])

    if len(x_ext)==0:
      cart_pos= x_trg
    else:
      cart_pos= TransformRightInv(x_trg,x_ext)

    #Get the inverse kinematic solution for joint angles
    return self.mu.arm[i].cart_exec.makeIKRequest(cart_pos, start_angles)


  #Move to a cart position
  def MoveToCartPos(self, x_trg, dt=2.0, x_ext=[], blocking=False):
    i = self.whicharm
    x_trg[3:7] = x_trg[3:7] / la.norm(x_trg[3:7])

    if len(x_ext)==0:
      cart_pos= x_trg
    else:
      #Get a base pose from x_trg which is equal to x_ext in the base pose frame
      #i.e. solve for x in "x_trg = x * x_ext"
      cart_pos= TransformRightInv(x_trg,x_ext)
      #l_p,l_R= XToPosRot(x_ext)
      #p_d,R_d= XToPosRot(x_trg)
      #pw_d= p_d - np.dot(R_d,np.dot(l_R.T,l_p))
      #Rw_d= np.dot(R_d,l_R.T)
      #cart_pos= PosRotToX(pw_d,Rw_d)

    print "Target= ",cart_pos
    self.mu.arm[i].moveToCartPos(cart_pos, dt, blocking)
    if self.mu.arm[i].last_error_code != 1:
      print "Error code= "+str(self.mu.arm[i].last_error_code)


  #Move to a cart position with interpolation
  def MoveToCartPosI(self,x_trg,dt=2.0,x_ext=[],inum=30,blocking=False):
    i = self.whicharm
    angles_prev= self.mu.arm[i].getCurrentPosition()
    x_curr= np.array(self.CartPos(x_ext))

    interpolation_controller_version= 3
    if interpolation_controller_version==1:
      print '###Error in MoveToCartPosI: Version 1 is not worth to be kept'
    elif interpolation_controller_version==2:
      '''
      Version 2:
      IK -> q-interpolation -> control --> IK -> q-interpolation -> control --> ... --> IK -> q-interpolation -> control
        1. Generate a linear interpolation from x_curr to x_trg
        2. angles_prev <-- Current joint position (angles)
        3. For each interpolated point x:
          a. angles <-- ExtendedIK(x, x_ext, angles_prev)
          b. Generate a linear interpolation from angles_prev to angles
          c. Send the angles trajectory to controller
          d. angles_prev <-- angles
      '''
      x_traj= CartPosInterpolation(x_curr,x_trg,inum)
      #print x_traj
      idt= dt/float(inum)
      #fp=file('/tmp/trajxxx1.dat','w'); T=0.0
      for n in range(inum):
        x_curr= x_traj[n]
        resp= self.MakeIKRequest(x_curr, x_ext, angles_prev)
        if resp.error_code.val == 1:
          angles= np.array(resp.solution.joint_state.position)
          goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
          goal.trajectory.joint_names= self.mu.arm[i].goal.trajectory.joint_names
          traj_duration= InterpolateLinearly2(goal.trajectory.points, angles_prev, angles, idt, self.control_time_step, rot_adjust=True, vel_limits=self.vel_limits)
          goal.trajectory.header.stamp= rospy.Time.now()
          self.mu.arm[i].traj_client.send_goal(goal)
          angles_prev= angles

          #for p in goal.trajectory.points: fp.write(str(T+p.time_from_start.to_sec())+' '+VecToStr(p.positions)+'\n')
          #T+=traj_duration

          start_time= rospy.Time.now()
          while rospy.Time.now() < start_time + rospy.Duration(traj_duration):
            time.sleep(traj_duration*0.02)
        else:
          print "IK error: ",resp.error_code.val
          break
    elif interpolation_controller_version==3:
      '''
      Version 3:
      IK --> IK --> ... --> IK --> control
        1. Generate a linear interpolation from x_curr to x_trg
        2. angles_prev <-- Current joint position (angles)
        3. angles_traj <-- []
        4. For each interpolated point x:
          a. angles <-- ExtendedIK(x, x_ext, angles_prev)
          b. angles_traj <-- angles_traj + [angles]
          c. angles_prev <-- angles
        5. Send angles_traj to controller
      '''
      x_traj= CartPosInterpolation(x_curr,x_trg,inum)
      #print x_traj
      idt= dt/float(inum)
      goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
      goal.trajectory.joint_names= self.mu.arm[i].goal.trajectory.joint_names
      for n in range(inum):
        x_curr= x_traj[n]
        resp= self.MakeIKRequest(x_curr, x_ext, angles_prev)
        if resp.error_code.val == 1:
          angles= np.array(resp.solution.joint_state.position)
          jp = trajectory_msgs.msg.JointTrajectoryPoint()
          jp.positions = angles
          jp.time_from_start = rospy.Duration(idt*(n+1))
          goal.trajectory.points.append(jp)
          angles_prev= angles
        else:
          print "IK error: ",resp.error_code.val
          break
      if len(goal.trajectory.points)==inum:
        AngleTrajSmoother(goal.trajectory.points)
        #fp=file('/tmp/trajxxx1.dat','w')
        #for p in goal.trajectory.points: fp.write(str(p.time_from_start.to_sec())+' '+VecToStr(p.positions)+'\n')
        #print goal
        goal.trajectory.header.stamp= rospy.Time.now()
        self.mu.arm[i].traj_client.send_goal(goal)
        start_time= rospy.Time.now()
        while rospy.Time.now() < start_time + rospy.Duration(dt):
          time.sleep(dt*0.02)
    elif interpolation_controller_version==4:
      '''
      Version 4:
      IK -> q-interpolation --> IK -> q-interpolation --> ... --> IK -> q-interpolation --> control
        1. Generate a linear interpolation from x_curr to x_trg
        2. angles_prev <-- Current joint position (angles)
        3. angles_traj <-- []
        4. For each interpolated point x:
          a. angles <-- ExtendedIK(x, x_ext, angles_prev)
          b. Generate a linear interpolation from angles_prev to angles --> subtraj
          b. angles_traj <-- angles_traj + subtraj
          c. angles_prev <-- angles
        5. Send angles_traj to controller
      '''
      x_traj= CartPosInterpolation(x_curr,x_trg,inum)
      #print x_traj
      idt= dt/float(inum)
      T= rospy.Duration(0.0)
      ik_error= False
      goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
      goal.trajectory.joint_names= self.mu.arm[i].goal.trajectory.joint_names
      for n in range(inum):
        x_curr= x_traj[n]
        resp= self.MakeIKRequest(x_curr, x_ext, angles_prev)
        if resp.error_code.val == 1:
          angles= np.array(resp.solution.joint_state.position)
          subtraj= trajectory_msgs.msg.JointTrajectory()
          traj_duration= InterpolateLinearly2(subtraj.points, angles_prev, angles, idt, self.control_time_step, rot_adjust=True, vel_limits=self.vel_limits)
          for jp in subtraj.points:
            jp.time_from_start= T+jp.time_from_start
            goal.trajectory.points.append(jp)
          T= T+rospy.Duration(traj_duration)
          angles_prev= angles
        else:
          print "IK error: ",resp.error_code.val
          ik_error= True
          break
      if not ik_error:
        AngleTrajSmoother(goal.trajectory.points)
        #fp=file('/tmp/trajxxx1.dat','w')
        #for p in goal.trajectory.points: fp.write(str(p.time_from_start.to_sec())+' '+VecToStr(p.positions)+'\n')
        #print goal
        goal.trajectory.header.stamp= rospy.Time.now()
        self.mu.arm[i].traj_client.send_goal(goal)
        start_time= rospy.Time.now()
        while rospy.Time.now() < start_time + T:
          time.sleep(T.to_sec()*0.02)

  #pos: 0.08 (open), 0.0 (close), max_effort: 12~15 (weak), 50 (string), -1 (maximum)
  def CommandGripper(self,pos,max_effort,blocking=False):
    self.mu.arm[self.whicharm].commandGripper(pos,max_effort,blocking)


  def MoveHead(self,pan,tilt,dt):
    traj= trajectory_msgs.msg.JointTrajectory()
    traj.joint_names= self.head_joint_names
    traj.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

    traj.points[0].time_from_start= rospy.Duration(dt)
    traj.points[0].positions= [pan,tilt]
    traj.points[0].velocities= [0.0]*2
    traj.header.stamp = rospy.Time.now()
    self.head_pub.publish(traj)


  def UpdateAR(self,id):
    x= self.wm.getObjectById(id)
    if x != -1:  self.ar_x[id]= x
    else:  print 'Marker is not observed'

  def IsARObserved(self,id):
    return (id in self.ar_x)

  def IsARAvailable(self,id):
    if not self.IsARObserved(id):
      print 'AR marker ',id,' has not been observed'
      return False
    if len(self.x_marker_to_torso)!=7:
      print 'Calibration has not done. Execute "calib"'
      return False
    return True

  def ARX(self,id):
    #if not self.IsARAvailable(id):
      #return -1
    return Transform(self.x_marker_to_torso,self.ar_x[id])

  def BPX(self,id):
    return self.base_x[id]


  #Calibration to transform a marker pose to torso-frame
  def Calibration(self):
    print 'Do you want to calibrate?'
    if AskYesNo():
      print 'Let the robot hold a marker tag.  Use',self.ArmStr(),'arm.'
      print 'Make sure to put the marker at the position obtained by "xe" command.'
      print 'Any arm posture is OK.'
      print 'Then, type the marker ID.'
      id= int(raw_input('  ID (-1 to cancel calib) > '))
      if id!=-1:
        self.UpdateAR(id)
        if self.IsARObserved(id):
          xe= self.CartPos(self.control_frame[self.whicharm])
          self.x_marker_to_torso= TransformRightInv(xe, self.ar_x[id])
          print 'Result: ',VecToStr(self.x_marker_to_torso)
        else:
          print 'Marker ',id,' is not observed.  Make sure the camera position, then try again.'
    print 'Done'


  def FlowAmountControl(self, amount_trg, rot_axis, x_ext=[], max_duration=10.0, only_pour=False):
    if not self.material_amount_observed:
      print "Error: /color_occupied_ratio is not observed"
      return

    i= self.whicharm
    #angles_init= self.mu.arm[i].getCurrentPosition()
    x_init= np.array(self.CartPos(x_ext))

    goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    goal.trajectory.joint_names= self.mu.arm[i].goal.trajectory.joint_names
    goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

    theta= 0.0
    theta_prev= 0.0
    elapsed_time= 0.0
    damount= 0.0
    amount= self.material_amount
    tmpfp= file(os.environ['HOME']+'/tmp/flowc01','w')
    while elapsed_time<max_duration:
      amount_prev= amount
      amount= self.material_amount
      if amount >= amount_trg:
        print 'Poured! (',amount,' / ',amount_trg,')'
        break

      theta_prev= theta

      self.flow_control_kind= 3
      if self.flow_control_kind==1:
        damount= (amount-amount_prev)/self.flow_control_time_step
        dtheta= self.flow_control_gain_p * (amount_trg - amount) - self.flow_control_gain_d * damount
        if dtheta > self.flow_control_dtheta_max:  dtheta= self.flow_control_dtheta_max
        elif dtheta < self.flow_control_dtheta_min:  dtheta= self.flow_control_dtheta_min
        theta= theta+dtheta * self.flow_control_time_step
        if theta > self.flow_control_theta_max:  theta= self.flow_control_theta_max
        print elapsed_time,': ',amount,' (',damount,') / ',amount_trg,' : ',theta,', ',dtheta
        tmpfp.write('%f %f %f %f %f %f\n' % (elapsed_time,amount,damount,amount_trg,theta,dtheta))
      elif self.flow_control_kind==2:
        self.flow_control_gain_p2= 100.0
        self.flow_control_gain_d2= 20.0
        dtheta= (theta-theta_prev)/self.flow_control_time_step
        theta= self.flow_control_gain_p2 * (amount_trg - amount) - self.flow_control_gain_d2 * dtheta
        if theta > self.flow_control_theta_max:  theta= self.flow_control_theta_max
        print elapsed_time,': ',amount,' (',damount,') / ',amount_trg,' : ',theta,', ',dtheta
      elif self.flow_control_kind==3:
        #damount= (amount-amount_prev)/self.flow_control_time_step
        amount_trg_t= amount_trg/8.0 * elapsed_time
        if amount_trg_t>amount_trg: amount_trg_t= amount_trg
        dtheta= self.flow_control_gain_p * (amount_trg_t - amount)
        if dtheta > self.flow_control_dtheta_max:  dtheta= self.flow_control_dtheta_max
        elif dtheta < self.flow_control_dtheta_min:  dtheta= self.flow_control_dtheta_min
        theta= theta+dtheta * self.flow_control_time_step
        if theta > self.flow_control_theta_max:  theta= self.flow_control_theta_max
        elif theta < 0.0:  theta= 0.0
        print elapsed_time,': ',amount,' / ',amount_trg_t,' : ',theta,', ',dtheta
        tmpfp.write('%f %f %f %f %f\n' % (elapsed_time,amount,amount_trg_t,theta,dtheta))

      p_init,R_init= XToPosRot(x_init)
      dR= QToRot(tf.transformations.quaternion_about_axis(theta, rot_axis))
      R_trg= np.dot(dR,R_init)
      x_trg= PosRotToX(p_init,R_trg)
      #print '##',VecToStr(x_trg)

      angles_curr= self.mu.arm[i].getCurrentPosition()
      resp= self.MakeIKRequest(x_trg, x_ext, angles_curr)

      if resp.error_code.val == 1:
        angles= np.array(resp.solution.joint_state.position)
        goal.trajectory.points[0].positions = angles
        goal.trajectory.points[0].time_from_start = rospy.Duration(self.flow_control_time_step)
        #angles_curr= angles

        goal.trajectory.header.stamp= rospy.Time.now()
        self.mu.arm[i].traj_client.send_goal(goal)
        start_time= rospy.Time.now()
        while rospy.Time.now() < start_time + rospy.Duration(self.flow_control_time_step):
          time.sleep(self.flow_control_time_step*0.02)

      else:
        print "IK error: ",resp.error_code.val
        break

      elapsed_time+= self.flow_control_time_step

    if not only_pour:
      #Move back to the initial pose
      self.MoveToCartPosI(x_init,self.flow_move_back_duration,x_ext,inum=30,blocking=True)

      #goal.trajectory.points[0].positions = angles_init
      #goal.trajectory.points[0].time_from_start = rospy.Duration(self.flow_move_back_duration)

      #goal.trajectory.header.stamp= rospy.Time.now()
      #self.mu.arm[i].traj_client.send_goal(goal)
      #start_time= rospy.Time.now()
      #while rospy.Time.now() < start_time + rospy.Duration(self.flow_move_back_duration):
        #time.sleep(self.flow_move_back_duration*0.02)


  def FlowAmountControlWithShaking(self, amount_trg, x_ext=[], max_duration=10.0):
    if not self.material_amount_observed:
      print "Error: /color_occupied_ratio is not observed"
      return

    i= self.whicharm
    x_init= np.array(self.CartPos(x_ext))

    #Use FlowAmountControl to rotate the bottle
    #FIXME: this part should be separated
    self.FlowAmountControl(amount_trg, [1,0,0], x_ext, max_duration=10.0, only_pour=True)

    #Then, keep pouring with shaking

    theta= 0.0
    elapsed_time= 0.0
    damount= 0.0
    amount= self.material_amount
    dt= 1.0
    x_init2= np.array(self.CartPos(x_ext))
    while elapsed_time<max_duration:
      amount_prev= amount
      amount= self.material_amount
      if amount >= amount_trg:
        print 'Poured! (',amount,' / ',amount_trg,')'
        break
      damount= (amount-amount_prev)/dt
      shake_freq= self.flow_shake_gain_p * (amount_trg - amount) - self.flow_shake_gain_d * damount

      if shake_freq > self.flow_shake_freq_max:  shake_freq= self.flow_shake_freq_max
      elif shake_freq < self.flow_shake_freq_min:  shake_freq= self.flow_shake_freq_min

      print elapsed_time,': ',amount,' (',damount,') / ',amount_trg,' : ',shake_freq

      dt= 1.0/shake_freq
      #self.ShakeGripper(shake_freq,self.flow_shake_width,x_ext,self.flow_shake_axis)

      #>>>Shaking motion
      x_trg= copy.deepcopy(x_init2)
      shake_axis= copy.deepcopy(self.flow_shake_axis)  #FIXME: this code should be outside the loop
      shake_axis= np.array(shake_axis) / la.norm(shake_axis)  #FIXME: ditto
      x_trg[0:3]+= np.array(shake_axis)*self.flow_shake_width
      self.MoveToCartPosI(x_trg,dt/2.0,x_ext,inum=5,blocking=True)
      self.MoveToCartPosI(x_init2,dt/2.0,x_ext,inum=5,blocking=True)
      #<<<Shaking motion

      elapsed_time+= dt

    #Move back to the initial pose
    self.MoveToCartPosI(x_init,self.flow_move_back_duration,x_ext,inum=30,blocking=True)


  def ShakeGripper(self,shake_Hz=2.0,shake_width=0.04,x_ext=[],shake_axis=[0.0,0.0,1.0]):
    #self.deadman = False
    #self.init = True
    #self.reset = True
    i= self.whicharm
    angles_init= self.mu.arm[i].getCurrentPosition()
    x_init= np.array(self.CartPos(x_ext))

    x_trg= copy.deepcopy(x_init)
    shake_axis= np.array(shake_axis) / la.norm(shake_axis)
    x_trg[0:3]+= np.array(shake_axis)*shake_width

    dt= 1.0/shake_Hz/2.0
    self.MoveToCartPosI(x_trg,dt,x_ext,inum=5,blocking=True)
    self.MoveToCartPosI(x_init,dt,x_ext,inum=5,blocking=True)

    #resp= self.MakeIKRequest(x_trg, x_ext, angles_init)

    #if resp.error_code.val == 1:
      #dt= 1.0/shake_Hz/2.0

      #goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
      #goal.trajectory.joint_names= self.mu.arm[i].goal.trajectory.joint_names
      #goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
      #goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

      #angles= np.array(resp.solution.joint_state.position)
      #goal.trajectory.points[0].positions = angles
      #goal.trajectory.points[0].time_from_start = rospy.Duration(dt)
      #goal.trajectory.points[1].positions = angles_init
      #goal.trajectory.points[1].time_from_start = rospy.Duration(dt*2.0)

      #goal.trajectory.header.stamp= rospy.Time.now()
      #self.mu.arm[i].traj_client.send_goal(goal)
      #start_time= rospy.Time.now()
      #while rospy.Time.now() < start_time + rospy.Duration(dt*2.0):
        #time.sleep(dt*2.0*0.02)

    #else:
      #print "IK error: ",resp.error_code.val


  #Execute external motion script written in python,
  #which is imported as a module to this script, so we can share the memory
  def ExecuteMotion(self, fileid, args):
    modname= self.motion_path_prefix+fileid
    sub= __import__(modname,globals(),locals(),modname,-1)
    if sub:
      if modname in self.loaded_motions:
        reload(sub)
      else:
        self.loaded_motions.append(modname)
      sub.Run(self,args)
    else:
      print 'Cannot import motion file: ',modname
      return



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

