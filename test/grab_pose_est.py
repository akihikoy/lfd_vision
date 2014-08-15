#! /usr/bin/env python
import roslib; roslib.load_manifest('pr2_lfd_trick')
import rospy
import ar_track_alvar.msg
import visualization_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
import numpy.linalg as la
import math
import time
import sys
import cma
import random

###<<<Copied from core_tool.py
def ask_yes_no():
  while 1:
    sys.stdout.write('  (y|n) > ')
    ans= sys.stdin.readline().strip()
    if ans=='y' or ans=='Y':  return True
    elif ans=='n' or ans=='N':  return False

def AskYesNo():
  return ask_yes_no()

#Convert a vector to string
def VecToStr(vec,delim=' '):
  return delim.join(map(str,vec))

def QFromAxisAngle(axis,angle):
  axis= axis / la.norm(axis)
  return tf.transformations.quaternion_about_axis(angle,axis)

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

##Compute "x2 * x1"; x* are [x,y,z,quaternion] form
#def Transform(x2, x1):
  #p2,R2= XToPosRot(x2)
  #p1,R1= XToPosRot(x1)
  #p= np.dot(R2,p1)+p2
  #R= np.dot(R2, R1)
  #return PosRotToX(p,R)
###>>>copied from core_tool.py


class TSimpleVisualizer:
  def __init__(self, viz_dt=rospy.Duration(1.0)):
    self.viz_pub= rospy.Publisher('visualization_marker', visualization_msgs.msg.Marker)
    self.curr_id= 0
    self.viz_frame= '/base_link'
    self.viz_ns= 'visualizer'
    self.viz_dt= viz_dt
    #self.viz_dt= rospy.Duration()
    self.indexed_colors= [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1],[1,1,1]]

  def Reset(self):
    self.curr_id= 0

  def ICol(self, i):
    return self.indexed_colors[i%len(self.indexed_colors)]

  def GenMarker(self, x, scale, rgb, alpha):
    marker= visualization_msgs.msg.Marker()
    marker.header.frame_id= self.viz_frame
    marker.header.stamp= rospy.Time.now()
    marker.ns= self.viz_ns
    marker.id= self.curr_id
    marker.action= visualization_msgs.msg.Marker.ADD  # or DELETE
    marker.lifetime= self.viz_dt
    marker.scale.x= scale[0]
    marker.scale.y= scale[1]
    marker.scale.z= scale[2]
    marker.color.a= alpha
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.pose= XToGPose(x)
    self.curr_id+= 1
    return marker

  def AddMarker(self, x, scale=[0.02,0.02,0.004], rgb=[1,1,1], alpha=1.0):
    marker= self.GenMarker(x, scale, rgb, alpha)
    marker.type= visualization_msgs.msg.Marker.CUBE  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)

  def AddArrow(self, x, scale=[0.05,0.002,0.002], rgb=[1,1,1], alpha=1.0):
    marker= self.GenMarker(x, scale, rgb, alpha)
    marker.type= visualization_msgs.msg.Marker.ARROW  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)

  def AddCube(self, x, scale=[0.05,0.03,0.03], rgb=[1,1,1], alpha=1.0):
    marker= self.GenMarker(x, scale, rgb, alpha)
    marker.type= visualization_msgs.msg.Marker.CUBE  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)

  def AddCylinder(self, p1, p2, diameter, rgb=[1,1,1], alpha=1.0):
    ez= Vec(p2)-Vec(p1)
    ez= ez/la.norm(ez)
    ex= [ez[1],-ez[0],0.0]
    ex= ex/la.norm(ex)
    ey= np.cross(ez,ex)
    x= [0]*7
    x[0:3]= 0.5*(Vec(p1)+Vec(p2))
    x[3:]= RotToQ(np.matrix([ex,ey,ez]).T)
    length= la.norm(Vec(p2)-Vec(p1))

    scale= [diameter,diameter,length]
    marker= self.GenMarker(x, scale, rgb, alpha)
    marker.type= visualization_msgs.msg.Marker.CYLINDER  # or CUBE, SPHERE, ARROW, CYLINDER
    self.viz_pub.publish(marker)


def DegToRad(q):
  conv= lambda x: float(x)/180.0*math.pi
  if type(q) in (float,int):
    return conv(q)
  else:
    return map(conv, q)

def RadToDeg(q):
  conv= lambda x: float(x)/math.pi*180.0
  if type(q) in (float,int):
    return conv(q)
  else:
    return map(conv, q)

def Vec(x):
  return np.array(x)

#Calculating an angle [0,pi] between two 3-D vectors
def GetAngle(p1,p2):
  cos_th= np.dot(p1,p2) / (la.norm(p1)*la.norm(p2))
  if cos_th>1.0:  cos_th=1.0
  elif cos_th<-1.0:  cos_th=-1.0
  return math.acos(cos_th)

#Float version of range
def FRange1(xmin,xmax,num_div):
  return [xmin+(xmax-xmin)*x/float(num_div) for x in range(num_div+1)]

#Check if a is between [a_range[0],a_range[1]]
def IsIn(a, a_range):
  if a_range[0]<a_range[1]:
    return a_range[0]<=a and a<=a_range[1]
  else:
    return a_range[1]<=a and a<=a_range[0]

#Check if a is between [a_range[0][0],a_range[0][1]] or [a_range[1][0],a_range[1][1]] or ...
def IsIn2(a, a_range):
  for a_r in a_range:
    if IsIn(a,a_r):  return True
  return False

#Convert x to geometry_msgs/Pose
def XToGPose(x):
  pose= geometry_msgs.msg.Pose()
  pose.position.x= x[0]
  pose.position.y= x[1]
  pose.position.z= x[2]
  pose.orientation.x= x[3]
  pose.orientation.y= x[4]
  pose.orientation.z= x[5]
  pose.orientation.w= x[6]
  return pose

#Convert geometry_msgs/Pose to x
def GPoseToX(x):
  x= [0]*7
  x[0]= pose.position.x
  x[1]= pose.position.y
  x[2]= pose.position.z
  x[3]= pose.orientation.x
  x[4]= pose.orientation.y
  x[5]= pose.orientation.z
  x[6]= pose.orientation.w
  return x

#Compute "x2 * x1"; x* are [x,y,z,quaternion] form
#x1 can also be [x,y,z] or [quaternion]
def Transform(x2, x1):
  p2,R2= XToPosRot(x2)
  if len(x1)==7:
    p1,R1= XToPosRot(x1)
    p= np.dot(R2,p1)+p2
    R= np.dot(R2, R1)
    return PosRotToX(p,R)
  if len(x1)==3:  #i.e. [x,y,z]
    p1= x1
    p= np.dot(R2,p1)+p2
    return p
  if len(x1)==4:  #i.e. [quaternion]
    R1= QToRot(x1)
    R= np.dot(R2, R1)
    return RotToQ(R)


#Virtual setup:
x_b= [0.5, 0.05, 0.05, 0,0,0,1]
x_b[3:]= QFromAxisAngle([0,0,1],5.0/180.0*math.pi)
x_c= [0.45, -0.1,0.05, 0,0,0,1]

l_p_1= [-0.00189519, -0.000732017, 0.0221993]
l_p_2= [-0.00440757, -0.00170243,  0.0516281]
#l_p_1= [-0.00189519, -0.000732017, 0.01]
#l_p_2= [-0.00440757, -0.00170243,  0.04]

#q_i= [0,0,0,1]


#Some common computation:
p_1= Transform(x_b,l_p_1)
p_2= Transform(x_b,l_p_2)


#Evaluate a grab pose of a cylinder model for pouring
#p_g, R_g: grab point and orientation
#x_b: source container pose
#x_c: receiving container pose
#p_1, p_2: cylinder (bottom and top points)
def EvalGrabPose(p_g, R_g, x_b, x_c, p_1, p_2, verbose=False):
  score= 1.0
  e_x= R_g[:,0]
  e_y= R_g[:,1]
  e_z= R_g[:,2]

  #Requirement from pouring:
  a1= GetAngle(e_x, (Vec(x_c)-Vec(x_b))[0:3])
  if verbose: print 'a1=',RadToDeg(a1)
  if not IsIn(a1,DegToRad([70,110])):  return None
  score-= abs(a1-DegToRad(90))/math.pi

  #Requirement from left hand grab pose
  a2= math.atan2(e_x[1],e_x[0])
  if verbose: print 'a2=',RadToDeg(a2)
  if not IsIn2(a2,(DegToRad([170,180]),DegToRad([-180,30]))):  return None
  score-= abs(a2)/math.pi

  #Requirement from cylinder grab pose
  a3= GetAngle(e_z, Vec(p_2)-Vec(p_1))
  if verbose: print 'a3=',RadToDeg(a3)
  if not IsIn(a3,DegToRad([0,30])):  return None
  score-= abs(a3)/math.pi

  #Requirement from pouring:
  if p_g[2]-x_b[2] <= 0.03:  # Height of grab point is less than 3cm
    a4= math.atan2(e_x[2],la.norm([e_x[0],e_x[1]]))
    if verbose: print 'a4=',RadToDeg(a4)
    if not IsIn(a4, DegToRad([-90,-20])):  return None

  return score


def GetQFromParameters(a_1, a_2, a_3):
  ax= [math.cos(a_2)*math.cos(a_1), math.cos(a_2)*math.sin(a_1), math.sin(a_2)]
  return QFromAxisAngle(ax, a_3)

def EvalGrabPoseForFMin(parameters, p_g, x_b, x_c, p_1, p_2, f_none=100):
  a_1= parameters[0]
  a_2= parameters[1]
  a_3= parameters[2]
  R_g= QToRot(GetQFromParameters(a_1, a_2, a_3))
  score= EvalGrabPose(p_g, R_g, x_b, x_c, p_1, p_2)
  if score:  return -score
  else:  return f_none


viz= TSimpleVisualizer(viz_dt=rospy.Duration())

def PlanGrab():
  viz.Reset()
  viz.AddMarker(x_b, rgb=viz.ICol(0))
  viz.AddMarker(x_c, rgb=viz.ICol(1))
  viz.AddCylinder(p_1,p_2,0.04, rgb=viz.ICol(2), alpha=0.7)

  p_g= 0.5*(Vec(p_1)+Vec(p_2))

  #Solve by brute force search:
  score_best= -1e20
  best_a_1= 0
  best_a_2= 0
  best_a_3= 0
  for a_1 in FRange1(-math.pi,math.pi,360/10):
    for a_2 in FRange1(-0.5*math.pi,0.5*math.pi,180/10):
      for a_3 in FRange1(-math.pi,math.pi,360/10):
        R_g= QToRot(GetQFromParameters(a_1, a_2, a_3))
        score= EvalGrabPose(p_g, R_g, x_b, x_c, p_1, p_2)
        if score and score>score_best:
          score_best= score
          best_a_1= a_1
          best_a_2= a_2
          best_a_3= a_3

  q_g= GetQFromParameters(best_a_1, best_a_2, best_a_3)
  x_g= [0]*7
  x_g[0:3]= p_g
  x_g[3:]= q_g
  R_g= QToRot(q_g)
  print 'score=',score_best
  print 'a=',best_a_1, best_a_2, best_a_3
  print 'p_g=',p_g
  print 'R_g=',R_g
  EvalGrabPose(p_g, R_g, x_b, x_c, p_1, p_2, verbose=True)
  viz.AddCube(x_g, rgb=viz.ICol(3), alpha=0.7)
  viz.AddArrow(x_g, rgb=viz.ICol(3), alpha=0.7)

  #Solve by CMA-ES:
  fit= 100
  while fit>=99:
    options= {'CMA_diagonal':1, 'verb_time':0}
    options['bounds']= [[-math.pi,-0.5*math.pi,-math.pi],[math.pi,0.5*math.pi,math.pi]]
    options['tolfun']= 1.0e-4 # 1.0e-4
    options['verb_log']= False
    res= cma.fmin(lambda x: EvalGrabPoseForFMin(x,p_g,x_b,x_c,p_1,p_2,None), [0.0]*3, 0.25*math.pi, options)
    print('best solutions fitness = %f' % (res[1]))
    fit= res[1]
  print res

  q_g= GetQFromParameters(res[0][0], res[0][1], res[0][2])
  x_g= [0]*7
  x_g[0:3]= p_g
  x_g[3:]= q_g
  R_g= QToRot(q_g)
  print 'score=',-res[1]
  print 'a=',res[0]
  print 'p_g=',p_g
  print 'R_g=',R_g
  EvalGrabPose(p_g, R_g, x_b, x_c, p_1, p_2, verbose=True)
  viz.AddCube(x_g, rgb=viz.ICol(4), alpha=0.7)
  viz.AddArrow(x_g, rgb=viz.ICol(4), alpha=0.7)

  if score_best > -res[1]:
    print 'Check result'
    AskYesNo()


if __name__ == '__main__':
  rospy.init_node('test1')

  #Before using rospy.Time.now(), this is necessary!!
  time.sleep(0.2)

  #PlanGrab()
  #rospy.spin()
  while not rospy.is_shutdown():
    print '--------------------'
    x_c= [0.45, -0.1,0.05, 0,0,0,1]
    x_c[0]= x_b[0]+1.0*(random.random()-0.5)
    x_c[1]= x_b[1]+1.0*(random.random()-0.5)
    x_c[3:]= QFromAxisAngle([0,0,1],2.0*math.pi*(random.random()-0.5)*math.pi)
    print 'x_c=',x_c
    PlanGrab()
    print 'Continue?'
    if not AskYesNo():
      break


