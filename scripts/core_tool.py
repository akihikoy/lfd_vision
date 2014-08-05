#! /usr/bin/env python
import time
import numpy as np
import numpy.linalg as la
import roslib; roslib.load_manifest('pr2_lfd_trick')
import rospy
#import actionlib as al
from cmnUtils import *
import moveUtils
#import arWorldModel
import ar_track_alvar.msg
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
import yaml


def AskYesNo():
  return ask_yes_no()

#Convert a vector to string
def VecToStr(vec,delim=' '):
  return delim.join(map(str,vec))

#Automatically estimate the type of input and convert to it
def EstStrConvert(v_str):
  try:
    return int(v_str)
  except ValueError:
    pass
  try:
    return float(v_str)
  except ValueError:
    pass
  if v_str=='True' or v_str=='true' :  return True
  if v_str=='False' or v_str=='false':  return False
  try:
    x=[]
    for v in v_str.split(' '):
      x.append(float(v))
    return x
  except ValueError:
    pass
  try:
    x=[]
    for v in v_str.split(','):
      x.append(float(v))
    return x
  except ValueError:
    pass
  try:
    x=[]
    for v in v_str.split('\t'):
      x.append(float(v))
    return x
  except ValueError:
    pass
  return v_str

def QFromAxisAngle(axis,angle):
  axis= axis / la.norm(axis)
  return tf.transformations.quaternion_about_axis(angle,axis)

#Add a sub-dictionary with the key into a dictionary d
def AddSubDict(d,key):
  if not key in d:  d[key]= {}

#Print a dictionary with a nice format
def PrintDict(d,max_level=-1,level=0):
  for k,v in d.items():
    if type(v)==dict:
      print '  '*level,'[',k,']=...'
      if max_level<0 or level<max_level:
        PrintDict(v,max_level,level+1)
    else:
      print '  '*level,'[',k,']=',v

#Insert a new dictionary to the base dictionary
def InsertDict(d_base, d_new):
  for k_new,v_new in d_new.items():
    if k_new in d_base and (type(v_new)==dict and type(d_base[k_new])==dict):
      InsertDict(d_base[k_new], v_new)
    else:
      d_base[k_new]= v_new

#Load a YAML and insert the data into a dictionary
def InsertYAML(d_base, file_name):
  d_new= yaml.load(file(file_name).read())
  InsertDict(d_base, d_new)

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

#Get weighted average of two rotation matrices (intuitively, (1-w2)*R1 + w2*R2)
def AverageRot(R1, R2, w2):
  w= InvRodrigues(np.dot(R2,R1.T))
  return np.dot(Rodrigues(w2*w),R1)

#Get weighted average of two rotation quaternions (intuitively, (1-w2)*q1 + w2*q2)
def AverageQ(q1, q2, w2):
  return RotToQ( AverageRot(QToRot(q1), QToRot(q2), w2) )

#Get weighted average of two pose (intuitively, (1-w2)*x1 + w2*x2)
def AverageX(x1, x2, w2):
  x= [0]*7
  x[0:3]= (1.0-w2)*np.array(x1[0:3])+w2*np.array(x2[0:3])
  x[3:]= AverageQ(x1[3:], x2[3:], w2)
  return x

#Get average of poses
def AverageXData(x_data):
  if len(x_data)==0:  return []
  x_avr= x_data[0]
  for i in range(1,len(x_data)):
    w2= 1.0/(1.0+float(i))
    x_avr= AverageX(x_avr, x_data[i], w2)
  return x_avr

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

#Fitting a circle to the data XY, return the center [x,y] and the radius
def CircleFit2D_SVD(XY):
  centroid= np.average(XY,0) # the centroid of the data set

  X= [XY[d][0]-centroid[0] for d in range(len(XY))] # centering data
  Y= [XY[d][1]-centroid[1] for d in range(len(XY))] # centering data
  Z= [X[d]**2 + Y[d]**2 for d in range(len(XY))]
  ZXY1= np.matrix([Z, X, Y, [1.0]*len(Z)]).transpose()
  U,S,V= la.svd(ZXY1,0)
  if S[3]/S[0]<1.0e-12:  # singular case
    print "SINGULAR"
    A= (V.transpose())[:,3]
  else:  # regular case
    R= np.average(np.array(ZXY1),0)
    N= np.matrix([[8.0*R[0], 4.0*R[1], 4.0*R[2], 2.0],
                  [4.0*R[1], 1.0, 0.0, 0.0],
                  [4.0*R[2], 0.0, 1.0, 0.0],
                  [2.0,      0.0, 0.0, 0.0]])
    W= V.transpose()*np.diag(S)*V
    D,E= la.eig(W*la.inv(N)*W)  # values, vectors
    idx= D.argsort()
    Astar= E[:,idx[1]]
    A= la.solve(W, Astar)

  A= np.array(A)[:,0]
  center= -A[1:3].transpose()/A[0]/2.0+centroid
  radius= math.sqrt(A[1]**2+A[2]**2-4.0*A[0]*A[3])/abs(A[0])/2.0
  return center, radius

#Fitting a circle to the data marker_data, return the center [x,y,z] and the radius
#  marker_data: a sequence of pose vector [0-2]: position x,y,z, [3-6]: orientation qx,qy,qz,qw
def CircleFit3D(marker_data):
  #Compute the normal vector
  p_mean= np.array([0.0,0.0,0.0])
  n_z= np.array([0.0,0.0,0.0])
  for x in marker_data:
    p,R= XToPosRot(x)
    n_z+= R[:,2]
    p_mean+= p
  n_z= n_z/float(len(marker_data))
  n_z= n_z/la.norm(n_z)
  p_mean= p_mean/float(len(marker_data))
  n_x= np.array([-n_z[1],n_z[0],0.0])
  n_x= n_x/la.norm(n_x)
  n_y= np.cross(n_z,n_x)

  print 'p_mean= ',p_mean
  print 'n_x= ',n_x
  print 'n_y= ',n_y
  print 'n_z= ',n_z

  #Project the data onto the plane n_x, n_y
  p_data= []
  for x in marker_data:
    p= np.array([x[0],x[1],x[2]])
    lx= np.dot(p-p_mean,n_x)
    ly= np.dot(p-p_mean,n_y)
    p_data.append([lx,ly])
    #print 'lx,ly= ',lx,ly

  #print p_data
  p_file= file('/tmp/original.dat','w')
  for x in marker_data:
    p_file.write('%f %f %f\n' % (x[0],x[1],x[2]))
  p_file.close()

  #Compute the circle parameters
  lcx, radius= CircleFit2D_SVD(p_data)
  x_center= lcx[0]*n_x + lcx[1]*n_y + p_mean
  #print x_center, radius

  p_file= file('/tmp/circle.dat','w')
  for ith in range(1000):
    th= (2.0*math.pi)/1000.0*float(ith)
    lx= lcx[0]+radius*math.cos(th)
    ly= lcx[1]+radius*math.sin(th)
    x= lx*n_x + ly*n_y + p_mean
    p_file.write('%f %f %f\n' % (x[0],x[1],x[2]))
  p_file.close()

  return x_center, radius

#Compute the sensor pose x_sensor on the robot frame from data
#  marker_data: a sequence of pose vector [0-2]: position x,y,z, [3-6]: orientation qx,qy,qz,qw
#  gripper_data: corresponding gripper pose sequence (on the robot frame)
#  x_g2m: marker pose on the gripper's local frame
def CalibrateSensorPose(marker_data, gripper_data, x_g2m):
  assert(len(marker_data)==len(gripper_data))
  #Marker poses on the robot frame
  robot_marker_data= map(lambda x: Transform(x,x_g2m), gripper_data)
  #Sensor poses
  x_sensor_data= [TransformRightInv(robot_marker_data[d], marker_data[d]) for d in range(len(marker_data))]
  #Average sensor poses to get x_sensor
  x_sensor= AverageXData(x_sensor_data)
  #x_sensor= x_sensor_data[0]
  print 'la.norm(x_sensor[3:]):',la.norm(x_sensor[3:])
  print '##gripper_data[0]:',gripper_data[0]
  print '##robot_marker_data[0]:',robot_marker_data[0]
  print '##Transform(gripper_data[0],x_g2m):',Transform(gripper_data[0],x_g2m)
  print '##marker_data[0]:',marker_data[0]
  print '##x_sensor_data[0]:',x_sensor_data[0]
  print '##Transform(x_sensor_data[0],marker_data[0]):',Transform(x_sensor_data[0],marker_data[0])
  #for x in x_sensor_data:
    #print x
  #print x_sensor
  err= [0.0]*7
  for d in range(len(robot_marker_data)):
    err= [err[k]+abs(Transform(x_sensor,marker_data[d])[k]-robot_marker_data[d][k]) for k in range(7)]
    #err+= la.norm(np.array(Transform(x_sensor,marker_data[d]))-np.array(robot_marker_data[d]))
  err= [err[k]/float(len(robot_marker_data)) for k in range(7)]
  print 'Error:',err
  return x_sensor


#There must be a planning scene or FK / IK crashes
def SetupPlanningScene():
  print 'Waiting for set planning scene service...'
  rospy.wait_for_service('/environment_server/set_planning_scene_diff')
  set_plan = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
  req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
  set_plan(req)
  print 'OK'



class TCoreTool:

  def __init__(self):

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


    self.flow_control_kind= 4
    self.flow_control_dtheta_max= math.pi*0.1
    self.flow_control_dtheta_min= -math.pi*0.1
    self.flow_control_time_step= 0.01
    self.flow_control_gain_p= 50.0
    self.flow_control_gain_d= 1.0  #DEPRECATED
    self.flow_control_gain_p2= 100.0  #Not tuned!!!
    self.flow_control_gain_d2= 20.0  #NOT USED
    self.flow_control_gain_p3= 1.0
    self.flow_control_gain_d3= 1.0
    self.flow_control_gain_p41= 1.0
    self.flow_control_gain_d41= 1.0
    self.flow_control_gain_p42= 0.1
    self.flow_control_gain_d42= 0.1
    #self.flow_move_back_duration= 3.0

    self.flow_shake_freq_max= 2.0
    self.flow_shake_freq_min= 0.1
    self.flow_shake_gain_p= 500.0
    self.flow_shake_gain_d= 10.0
    self.flow_shake_width= 0.04
    self.flow_shake_axis= [0.0,0.0,1.0]

    #self.ar_x= {}
    #self.x_marker_to_torso= []
    self.base_x= {}

    #Attributes of objects
    self.attributes= {}

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
    self.amount_observer_callback= None
    rospy.Subscriber("/color_occupied_ratio", std_msgs.msg.Float64, self.AmountObserver)

    #[id]=[pose x,y,z,qx,qy,qz,qw]
    self.ar_markers= {}
    self.ar_marker_frame_id= ""
    self.ar_marker_observer_callback= None
    #Sensor pose in robot frame
    self.x_sensor= []
    #Marker ID attached on the (left)-wrist link to adjust the marker error
    self.ar_adjust_m_id= 3
    self.ar_adjust_arm= 1  #Left arm
    self.l_x_m_wrist= []  #Marker pose in the wrist frame
    self.ar_adjust_ratio= 0.02  #Update ratio
    self.br= tf.TransformBroadcaster()
    rospy.Subscriber("/ar_pose_marker", ar_track_alvar.msg.AlvarMarkers, self.ARMarkerObserver)

  def __del__(self):
    #self.stopRecord()
    print 'TCoreTool: done'


  def Setup(self):
    SetupPlanningScene()
    self.mu= moveUtils.MoveUtils()
    #self.wm= arWorldModel.ARWorldModel()


  def ArmStr(self):
    return 'Right' if self.whicharm==0 else 'Left'
  def ArmStrS(self):
    return 'R' if self.whicharm==0 else 'L'


  #FIXME: Make a function to get an attribute
  #TODO: if there is no item with a key, search from a DEFAULT attribute
  #TODO: if no default, return None
  #def GetAttr(self,keys):
    #if type(keys)==str:
      #return self.attributes[keys]
    #elif type(keys)==list:
      #d= self.attributes
      #for n in keys:
        #d= d[n]
      #return d
  #Add a dictionary into attribute
  #FIXME: implement this
  #TODO:  element can be specified by a list of key
  #def AddDictAttr(self,keys):
    #if type(keys)==str:
      #AddSubDict(self.attributes,keys)
    #elif type(keys)==list:
      #...

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
    if self.amount_observer_callback:
      self.amount_observer_callback()

  def ARMarkerObserver(self, msg):
    for m in msg.markers:
      self.ar_markers[m.id]= m.pose.pose
      self.ar_marker_frame_id= m.header.frame_id

      if m.id==self.ar_adjust_m_id and len(self.l_x_m_wrist)==7:
        whicharm= self.whicharm
        self.whicharm= self.ar_adjust_arm
        x_m_wrist= self.CartPos(self.l_x_m_wrist)
        x_sensor2= TransformRightInv(x_m_wrist,self.ARXraw(self.ar_adjust_m_id))
        self.x_sensor= AverageX(self.x_sensor, x_sensor2, self.ar_adjust_ratio)
        self.whicharm= whicharm

    if len(self.x_sensor)==7:
      self.br.sendTransform(self.x_sensor[0:3],self.x_sensor[3:],
          rospy.Time.now(),
          self.ar_marker_frame_id,
          "torso_lift_link")

    if self.ar_marker_observer_callback:
      self.ar_marker_observer_callback()


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

  #Get cart pos of specified link_name
  #def CartPosAt(self,link_name,q):
    #link_name_org= self.mu.arm[i].cart_exec.FKreq.fk_link_names
    #self.mu.arm[i].cart_exec.FKreq.fk_link_names = [link_name]
    #x= self.mu.arm[i].cart_exec.makeFKRequest(q)
    #self.mu.arm[i].cart_exec.FKreq.fk_link_names = link_name_org


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
      raise


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
          raise
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
          raise
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
          raise
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


  #def UpdateAR(self,id):
    ##x= self.wm.getObjectById(id)
    #x= -1
    #if x != -1:  self.ar_x[id]= x
    #else:  print 'Marker is not observed'

  #def IsARObserved(self,id):
    #return (id in self.ar_x)

  #def IsARAvailable(self,id):
    #if not self.IsARObserved(id):
      #print 'AR marker ',id,' has not been observed'
      #return False
    #if len(self.x_marker_to_torso)!=7:
      #print 'Calibration has not done. Execute "calib"'
      #return False
    #return True

  def ARXraw(self,id):
    x_raw= self.ar_markers[id]
    xp= x_raw.position
    xq= x_raw.orientation
    return [xp.x, xp.y, xp.z,  xq.x, xq.y, xq.z, xq.w]

  def ARX(self,id):
    x= self.ARXraw(id)
    return Transform(self.x_sensor, x)

  def BPX(self,id):
    return self.base_x[id]


  ##Calibration to transform a marker pose to torso-frame
  #def Calibration(self):
    #print 'Do you want to calibrate?'
    #if AskYesNo():
      #print 'Let the robot hold a marker tag.  Use',self.ArmStr(),'arm.'
      #print 'Make sure to put the marker at the position obtained by "xe" command.'
      #print 'Any arm posture is OK.'
      #print 'Then, type the marker ID.'
      #id= int(raw_input('  ID (-1 to cancel calib) > '))
      #if id!=-1:
        #self.UpdateAR(id)
        #if self.IsARObserved(id):
          #xe= self.CartPos(self.control_frame[self.whicharm])
          #self.x_marker_to_torso= TransformRightInv(xe, self.ar_x[id])
          #print 'Result: ',VecToStr(self.x_marker_to_torso)
        #else:
          #print 'Marker ',id,' is not observed.  Make sure the camera position, then try again.'
    #print 'Done'


  def FlowAmountControl(self, amount_trg, rot_axis, max_theta, x_ext=[], trg_duration=8.0, max_duration=10.0):
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
    dtheta= 0.0
    theta_prev= 0.0
    elapsed_time= 0.0
    damount= 0.0
    amount= self.material_amount
    t= time.localtime()
    tmpfp= file('%s/tmp/flowc%02i%02i%02i%02i%02i%02i.dat' % (os.environ['HOME'],t.tm_year%100,t.tm_mon,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec),'w')
    while elapsed_time<max_duration:
      amount_prev= amount
      amount= self.material_amount
      if amount >= amount_trg:
        print 'Poured! (',amount,' / ',amount_trg,')'
        break

      theta_prev= theta

      #self.flow_control_kind= 4
      if self.flow_control_kind==1:
        #damount= (amount-amount_prev)/self.flow_control_time_step
        #dtheta= self.flow_control_gain_p * (amount_trg - amount) - self.flow_control_gain_d * damount
        dtheta= self.flow_control_gain_p * (amount_trg - amount)
        if dtheta > self.flow_control_dtheta_max:  dtheta= self.flow_control_dtheta_max
        elif dtheta < self.flow_control_dtheta_min:  dtheta= self.flow_control_dtheta_min
        theta= theta+dtheta * self.flow_control_time_step
        if theta > max_theta:  theta= max_theta
        elif theta < 0.0:  theta= 0.0
        print elapsed_time,': ',amount,' / ',amount_trg,' : ',theta,', ',dtheta
        tmpfp.write('%f %f %f %f %f %f\n' % (rospy.Time.now().to_nsec(),amount,amount_trg,amount_trg,theta,dtheta))
      elif self.flow_control_kind==2:
        #dtheta= (theta-theta_prev)/self.flow_control_time_step  #fix if use
        #theta= self.flow_control_gain_p2 * (amount_trg - amount) - self.flow_control_gain_d2 * dtheta
        theta= self.flow_control_gain_p2 * (amount_trg - amount)
        dtheta= (theta-theta_prev)/self.flow_control_time_step
        if dtheta > self.flow_control_dtheta_max:  dtheta= self.flow_control_dtheta_max
        elif dtheta < self.flow_control_dtheta_min:  dtheta= self.flow_control_dtheta_min
        theta= theta_prev+dtheta * self.flow_control_time_step
        if theta > max_theta:  theta= max_theta
        elif theta < 0.0:  theta= 0.0
        print elapsed_time,': ',amount,' / ',amount_trg,' : ',theta,', ',dtheta
        tmpfp.write('%f %f %f %f %f %f\n' % (rospy.Time.now().to_nsec(),amount,amount_trg,amount_trg,theta,dtheta))
      elif self.flow_control_kind==3:
        #damount= (amount-amount_prev)/self.flow_control_time_step
        amount_trg_t= amount_trg/trg_duration * elapsed_time
        if amount_trg_t>amount_trg: amount_trg_t= amount_trg
        dtheta= self.flow_control_gain_p3 * (amount_trg_t - amount) - self.flow_control_gain_d3 * dtheta
        if dtheta > self.flow_control_dtheta_max:  dtheta= self.flow_control_dtheta_max
        elif dtheta < self.flow_control_dtheta_min:  dtheta= self.flow_control_dtheta_min
        theta= theta+dtheta * self.flow_control_time_step
        if theta > max_theta:  theta= max_theta
        elif theta < 0.0:  theta= 0.0
        print elapsed_time,': ',amount,' / ',amount_trg_t,' : ',theta,', ',dtheta
        tmpfp.write('%f %f %f %f %f %f\n' % (rospy.Time.now().to_nsec(),amount,amount_trg_t,amount_trg,theta,dtheta))
      elif self.flow_control_kind==4:
        #damount= (amount-amount_prev)/self.flow_control_time_step
        amount_trg_t= amount_trg/trg_duration * elapsed_time
        if amount_trg_t>amount_trg: amount_trg_t= amount_trg
        if amount_trg_t - amount>=0:
          dtheta= self.flow_control_gain_p41 * (amount_trg_t - amount) - self.flow_control_gain_d41 * dtheta
        else:
          dtheta= self.flow_control_gain_p42 * (amount_trg_t - amount) - self.flow_control_gain_d42 * dtheta
        if dtheta > self.flow_control_dtheta_max:  dtheta= self.flow_control_dtheta_max
        elif dtheta < self.flow_control_dtheta_min:  dtheta= self.flow_control_dtheta_min
        theta= theta+dtheta * self.flow_control_time_step
        if theta > max_theta:  theta= max_theta
        elif theta < 0.0:  theta= 0.0
        print elapsed_time,': ',amount,' / ',amount_trg_t,' : ',theta,', ',dtheta
        tmpfp.write('%f %f %f %f %f %f\n' % (rospy.Time.now().to_nsec(),amount,amount_trg_t,amount_trg,theta,dtheta))

      p_init,R_init= XToPosRot(x_init)
      dR= QToRot(QFromAxisAngle(rot_axis,theta))
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
        raise

      elapsed_time+= self.flow_control_time_step

    #if not only_pour:
      ##Move back to the initial pose
      #self.MoveToCartPosI(x_init,self.flow_move_back_duration,x_ext,inum=30,blocking=True)

      ##goal.trajectory.points[0].positions = angles_init
      ##goal.trajectory.points[0].time_from_start = rospy.Duration(self.flow_move_back_duration)

      ##goal.trajectory.header.stamp= rospy.Time.now()
      ##self.mu.arm[i].traj_client.send_goal(goal)
      ##start_time= rospy.Time.now()
      ##while rospy.Time.now() < start_time + rospy.Duration(self.flow_move_back_duration):
        ##time.sleep(self.flow_move_back_duration*0.02)


  def FlowAmountControlWithShaking(self, amount_trg, x_ext=[], max_duration=10.0):
    if not self.material_amount_observed:
      print "Error: /color_occupied_ratio is not observed"
      return

    i= self.whicharm
    #x_init= np.array(self.CartPos(x_ext))

    ##Use FlowAmountControl to rotate the bottle
    ##this part should be separated
    #self.FlowAmountControl(amount_trg, [1,0,0], x_ext, max_duration=10.0, only_pour=True)

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
    #self.MoveToCartPosI(x_init,self.flow_move_back_duration,x_ext,inum=30,blocking=True)


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
      #raise


  #Load external motion script written in python,
  #which is imported as a module to this script, so we can share the memory
  def LoadMotion(self, fileid):
    modname= self.motion_path_prefix+fileid
    sub= __import__(modname,globals(),locals(),modname,-1)
    if sub:
      if modname in self.loaded_motions:
        reload(sub)
      else:
        self.loaded_motions.append(modname)
    else:
      print 'Cannot import motion file: ',modname
    return sub

  #Execute external motion script written in python,
  #which is imported as a module to this script, so we can share the memory
  def ExecuteMotion(self, fileid, args=[]):
    sub= self.LoadMotion(fileid)
    if sub:
      sub.Run(self,args)


if __name__ == '__main__':
  print 'Note: run cui_tool.py'



