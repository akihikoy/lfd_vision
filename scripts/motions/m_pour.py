#!/usr/bin/python
from cuiTool import *
def Run(t,args):
  l_cf_e= t.control_frame[t.whicharm] #Local vector to the current control frame
  b= t.BPX('b') #Bottle base pose on the torso frame
  c= t.BPX('c') #Cup base pose on the torso frame
  #Grab pose on the bottle frame (constant):
  l_grabx= [0.0143257412366, -0.00243017410404, 0.00332284373253, -0.0386798980774, 0.0474739514813, 0.0058252014884, 0.998106285144]
  #Grab pose on the torso frame
  grabx= Transform(b,l_grabx)
  print grabx
  t.CommandGripper(0.08,50,True)
  t.MoveToCartPos(grabx,3.0,l_cf_e,True)
  t.CommandGripper(0.0,15,True)

  #Pouring edge point on the bottle frame (constant):
  l_pourex= [0.0385446328044, -0.043639339547, 0.102811025179, 0.0,0.0,0.0,1.0]

  x= t.CartPos()
  l_cf_pe= TransformLeftInv(x, Transform(b,l_pourex))
  print 'l_cf_pe=',l_cf_pe

  #Move upward
  x= t.CartPos()
  x[2]+= 0.15
  t.MoveToCartPos(x,2.0,[],True)

  #Pouring location on the cup frame (constant):
  l_pourlx= [0.0164938693088, 0.00293250989281, 0.230512294328, 0.0,0.0,0.0,1.0]
  pourlx= Transform(c,l_pourlx)

  t.MoveToCartPos(pourlx,3.0,l_cf_pe,True)

  #Pouring orientation:
  pourq= [0.789370121064, 0.0178832059046, -0.0679767323576, 0.609880452856]

  pourexecx= pourlx
  pourexecx[3:7]= pourq  #Only change the orientation

  t.MoveToCartPos(pourexecx,3.0,l_cf_pe,True)
