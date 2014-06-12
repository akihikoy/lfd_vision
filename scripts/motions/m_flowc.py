#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test the flow amount controller.
  Assumptions:
    Gripper holds a bottle
    Bottle is close to the cup
  Usage: flowc BOTTLE_ID
    BOTTLE_ID: identifier of bottle. e.g. b1'''
def Run(t,args=[]):
  bottle= args[0]
  print t.material_amount
  #lw_x_pour_e= t.control_frame[t.whicharm] #Local vector to the current control frame

  if not 'grabbed' in t.attributes[bottle]:
    print 'Error: not grabbed: ',bottle
    return

  whicharm= t.whicharm
  t.SwitchArm(t.attributes[bottle]['grabbed']['grabber_handid'])


  #>>>FIXME: The same computation of lw_x_pour_e is used in m_pour.py
  #Infere bottle pose
  t.ExecuteMotion('infer',[bottle,'x'])
  x_b= t.attributes[bottle]['x']
  if len(x_b)!=7:
    print 'Bottle ',bottle,' pose is not observed'
    t.SwitchArm(whicharm)
    return
  #Pouring edge point on the bottle frame:
  lb_x_pour_e= t.attributes[bottle]['l_x_pour_e']
  x_w= t.CartPos()
  #Pouring edge point in the wrist frame
  lw_x_pour_e= TransformLeftInv(x_w, Transform(x_b,lb_x_pour_e))
  print 'lw_x_pour_e=',VecToStr(lw_x_pour_e)
  #<<<FIXME


  #Store the current position to move back later
  xe_init= t.CartPos(lw_x_pour_e)
  #print 'lw_x_pour_e=',VecToStr(lw_x_pour_e)

  #Estimate axis and max-angle for flow amount control
  p_init, R_init= XToPosRot(xe_init)
  R_max= QToRot(t.attributes[bottle]['q_pour_max'])
  trans_R= np.dot(R_max,R_init.T)
  axis_angle= InvRodrigues(trans_R)
  max_theta= la.norm(axis_angle)
  axis= axis_angle / max_theta

  #Flow amount control
  amount_trg= 0.03
  #axis= [1,0,0]
  #max_theta= math.pi*0.8
  t.FlowAmountControl(amount_trg, axis, max_theta, x_ext=lw_x_pour_e, trg_duration=8.0, max_duration=10.0)

  print 'Moving back to the initial pose...'
  t.MoveToCartPosI(xe_init,3.0,x_ext=lw_x_pour_e,inum=30,blocking=True)

  t.SwitchArm(whicharm)

