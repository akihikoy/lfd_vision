#!/usr/bin/python
from cuiTool import *
import copy
def Run(t,args):
  whicharm= t.whicharm
  t.SwitchArm(0)
  l_cf_e= t.control_frame[0] #Local vector to the current control frame
  xe= t.CartPos(l_cf_e)
  dt= 0.3

  x_trg= copy.deepcopy(xe)
  x_trg[2]-= 0.015

  for i in range(7):
    t.MoveToCartPosI(x_trg,dt/2.0,l_cf_e,inum=5,blocking=True)
    t.MoveToCartPosI(xe,dt/2.0,l_cf_e,inum=5,blocking=True)

  t.SwitchArm(whicharm)
