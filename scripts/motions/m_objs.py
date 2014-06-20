#!/usr/bin/python
from core_tool import *
def Help():
  return '''Define object properties and store them into attributes.
  Usage: objs'''
def Run(t,args=()):
  #NOTE: an attribute named 'l_*' denotes a vector defined on the local frame

  #Left gripper
  if not 'wrist_l' in t.attributes:  t.attributes['wrist_l']={}
  #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
  if not 'x' in t.attributes['wrist_l']: t.attributes['wrist_l']['x']= []
  #Gripper pose
  t.attributes['wrist_l']['l_x_gripper']= [0.16,0.0,0.0, 0.0,0.0,0.0,1.0]

  #Right gripper
  if not 'wrist_r' in t.attributes:  t.attributes['wrist_r']={}
  #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
  if not 'x' in t.attributes['wrist_r']: t.attributes['wrist_r']['x']= []
  #Gripper pose
  t.attributes['wrist_r']['l_x_gripper']= [0.16,0.0,0.0, 0.0,0.0,0.0,1.0]


  #Bottle No.1
  if not 'b1' in t.attributes:  t.attributes['b1']={}
  t.attributes['b1']['help']= 'A white soft cup.'
  #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
  if not 'x' in t.attributes['b1']: t.attributes['b1']['x']= []
  #Gripper width before grab
  t.attributes['b1']['g_pre']= 0.08
  #Grab power (max effort):
  t.attributes['b1']['f_grab']= 15.0
  #Grab pose:
  t.attributes['b1']['l_x_grab']= [0.0143257412366, -0.00243017410404, 0.00332284373253, -0.0386798980774, 0.0474739514813, 0.0058252014884, 0.998106285144]
  #Pouring edge point:
  t.attributes['b1']['l_x_pour_e']= [0.0385446328044, -0.043639339547, 0.102811025179, 0.0,0.0,0.0,1.0]
  #Orientation to start pouring
  t.attributes['b1']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
  #Orientation where the flow is max
  t.attributes['b1']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
  t.attributes['b1']['l_axis_shake']= [0,0,-1]
  t.attributes['b1']['l_x_tap']= [-0.011706553252399636, 0.066821805995982878, 0.085212694401439817, 0.78612053767768464, -0.45186131636266486, -0.30451820262824753, -0.29172678191144796]


  #Bottle No.2
  if not 'b2' in t.attributes:  t.attributes['b2']={}
  t.attributes['b2']['help']= 'A yellow plastic pot.'
  #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
  if not 'x' in t.attributes['b2']: t.attributes['b2']['x']= []
  #Gripper width before grab
  t.attributes['b2']['g_pre']= 0.03
  #Grab power (max effort):
  t.attributes['b2']['f_grab']= 50.0
  #Grab pose:
  t.attributes['b2']['l_x_grab']= [0.0173701175184, 0.132963892485, 0.0746058303632, -0.0496690610081, 0.0729356705803, -0.0512348563445, 0.994780559637]
  #Pouring edge point:
  t.attributes['b2']['l_x_pour_e']= [-0.0017665710111, -0.15640101956, 0.169072305583, 0.0,0.0,0.0,1.0]
  #Orientation to start pouring
  t.attributes['b2']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
  #Orientation where the flow is max
  t.attributes['b2']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)


  #Bottle No.3
  if not 'b3' in t.attributes:  t.attributes['b3']={}
  t.attributes['b3']['help']= 'A green beer (Heineken) bottle.'
  #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
  if not 'x' in t.attributes['b3']: t.attributes['b3']['x']= []
  #Gripper width before grab
  t.attributes['b3']['g_pre']= 0.06
  #Grab power (max effort):
  t.attributes['b3']['f_grab']= 80.0
  #Grab pose:
  t.attributes['b3']['l_x_grab']= [0.00830453390238, 0.00361607117282, 0.0827408487728, -0.0127897898305, 0.0400529652898, -0.0388682992739, 0.99835937245]
  #Pouring edge point:
  t.attributes['b3']['l_x_pour_e']= [0.00290016130429, 0.00562692930508, 0.222366269117, 0.0,0.0,0.0,1.0]
  #Orientation to start pouring
  t.attributes['b3']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
  #Orientation where the flow is max
  t.attributes['b3']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
  t.attributes['b3']['l_axis_shake']= [0,0,-1]


  #Cup No.1
  if not 'c1' in t.attributes:  t.attributes['c1']={}
  t.attributes['c1']['help']= 'A transparent soft plastic cup.'
  #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
  if not 'x' in t.attributes['c1']: t.attributes['c1']['x']= []
  #Pouring location:
  #t.attributes['c1']['l_x_pour_l']= [0.0164938693088, 0.00293250989281, 0.230512294328, 0.0,0.0,0.0,1.0]
  t.attributes['c1']['l_x_pour_l']= [0.0164938693088, 0.01293250989281, 0.210512294328, 0.0,0.0,0.0,1.0]


  t.ExecuteMotion('attr')

