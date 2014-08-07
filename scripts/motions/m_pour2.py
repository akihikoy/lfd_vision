#!/usr/bin/python
from core_tool import *
import copy
def Help():
  return '''Script to pour (DUPLICATED).  Use `pour'
  Usage: pour2'''
def Run(t,args=()):
  print Help()
  return

  bottle='b2'
  cup='c1'
  l_cf_e= t.control_frame[t.whicharm] #Local vector to the current control frame
  b= t.BPX('b') #Bottle base pose on the torso frame
  c= t.BPX('c') #Cup base pose on the torso frame
  #Grab pose on the bottle frame (constant):
  lx_grab= t.attributes[bottle]['l_x_grab']
  #Grab pose on the torso frame
  grabx= Transform(b,lx_grab)
  print 'grabx=',VecToStr(grabx)
  t.CommandGripper(t.attributes[bottle]['g_pre'],50,True)
  grabx0= copy.deepcopy(grabx)
  grabx0[0]= t.CartPos(l_cf_e)[0]
  t.MoveToCartPos(grabx0,3.0,l_cf_e,True)
  t.MoveToCartPos(grabx,3.0,l_cf_e,True)
  t.CommandGripper(0.0,t.attributes[bottle]['f_grab'],True)

  #Pouring edge point on the bottle frame (constant):
  lx_pour_e= t.attributes[bottle]['l_x_pour_e']

  x= t.CartPos()
  l_cf_pe= TransformLeftInv(x, Transform(b,lx_pour_e))
  print 'l_cf_pe=',VecToStr(l_cf_pe)

  #Move upward
  x= t.CartPos()
  x[2]+= 0.15
  t.MoveToCartPos(x,2.0,[],True)

  #Pouring location on the cup frame (constant):
  lx_pour_l= t.attributes[cup]['l_x_pour_l']
  pourlx= Transform(c,lx_pour_l)

  print 'pourlx=',VecToStr(pourlx)
  t.MoveToCartPos(pourlx,3.0,l_cf_pe,True)

  #Pouring orientation:
  pourq= [0.789370121064, 0.0178832059046, -0.0679767323576, 0.609880452856]

  pourexecx= pourlx
  pourexecx[3:7]= pourq  #Only change the orientation

  print 'pourexecx=',VecToStr(pourexecx)
  t.MoveToCartPosI(pourexecx,7.0,l_cf_pe,30,True)
