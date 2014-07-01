#!/usr/bin/python
from core_tool import *
from state_machine import *
def Help():
  return '''Flow amount controller with tapping.
  Assumptions:
    Left gripper holds a bottle
    Bottle is close to the cup
    Right gripper is a ready pose for tapping
  Usage: flowc_tap BOTTLE_ID [, AMOUNT_TRG [, MAX_DURATION]]
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)'''
def Run(t,args=()):
  bottle= args[0]
  amount_trg= args[1] if len(args)>1 else 0.03
  max_duration= args[2] if len(args)>2 else 25.0

  whicharm= t.whicharm
  t.SwitchArm(1) #Left arm

  m_flowc_cmn= t.LoadMotion('flowc_cmn')
  l= m_flowc_cmn.TLocal()
  if not l.Setup(t, bottle,amount_trg,max_duration):
    t.SwitchArm(whicharm) #Original arm
    return

  l.m_infer= t.LoadMotion('infer')
  #Tapping pose in bottle frame
  l.lb_x_tap= t.attributes[l.bottle]['l_x_tap']

  t.SwitchArm(0) #Right arm
  l.x_r_ext= t.control_frame[0]
  l.x_r_init= np.array(t.CartPos(l.x_r_ext))
  t.SwitchArm(1) #Left arm

  l.flow_obs_sensitivity= 0.003

  def MoveRGripperToTap():
    t.SwitchArm(0) #Right arm
    print 'Moving R-gripper to the tapping pose...'
    l.tmpfp.write('%f %f %f %f %f %f mrt\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))

    l.m_infer.Run(t,(l.bottle,'x'))
    x_b= t.attributes[l.bottle]['x']  #Bottle pose in robot frame
    l.x_tap= Transform(x_b,l.lb_x_tap)  #Tapping pose in robot frame

    t.MoveToCartPosI(l.x_tap,3.0,l.x_r_ext,inum=30,blocking=True)
    l.elapsed_time+= 3.0

    l.tmpfp.write('%f %f %f %f %f %f mrt\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
    t.SwitchArm(1) #Left arm

  def MoveRGripperToInit():
    t.SwitchArm(0) #Right arm
    print 'Moving R-gripper to the initial pose...'
    l.tmpfp.write('%f %f %f %f %f %f mri\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
    t.MoveToCartPosI(l.x_r_init,3.0,l.x_r_ext,inum=30,blocking=True)
    l.elapsed_time+= 3.0
    l.tmpfp.write('%f %f %f %f %f %f mri\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
    t.SwitchArm(1) #Left arm

  def Vibrate(count,dt=0.4):
    l.amount_prev= l.amount
    l.amount= t.material_amount

    t.SwitchArm(0) #Right arm
    print 'vibrate:',l.elapsed_time,': ',l.amount,' / ',l.amount_trg #,' : ',dt
    #x_r= np.array(t.CartPos(l.x_r_ext))
    x_r= l.x_tap
    x_r_trg= copy.deepcopy(x_r)
    x_r_trg[2]-= 0.015  #FIXME: magic number!!!
    for i in range(count):
      t.MoveToCartPosI(x_r_trg,dt/2.0,l.x_r_ext,inum=5,blocking=True)
      l.tmpfp.write('%f %f %f %f %f %f tp\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
      t.MoveToCartPosI(x_r,dt/2.0,l.x_r_ext,inum=5,blocking=True)
      l.tmpfp.write('%f %f %f %f %f %f tp\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
      l.elapsed_time+= dt
    t.SwitchArm(1) #Left arm

  sm= TStateMachine()
  sm.Debug= True

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= l.IsTimeout
  timeout_action.NextState= 'stop'

  poured_action= TFSMConditionedAction()
  poured_action.Condition= l.IsPoured
  poured_action.NextState= 'stop'

  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].NewAction()
  sm['start'].Actions[-1]= poured_action
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['start'].Actions[-1].NextState= 'move_r_tap'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: l.ControlStep(0.2 * t.flow_control_dtheta_max)
  sm['start'].ElseAction.NextState= 'start'

  sm['move_r_tap']= TFSMState()
  sm['move_r_tap'].EntryAction= MoveRGripperToTap
  sm['move_r_tap'].NewAction()
  sm['move_r_tap'].Actions[-1]= poured_action
  sm['move_r_tap'].NewAction()
  sm['move_r_tap'].Actions[-1]= timeout_action
  sm['move_r_tap'].ElseAction.Condition= lambda: True
  sm['move_r_tap'].ElseAction.NextState= 'tap'

  sm['tap']= TFSMState()
  sm['tap'].NewAction()
  sm['tap'].EntryAction= lambda: Vibrate(3)
  sm['tap'].Actions[-1]= poured_action
  sm['tap'].NewAction()
  sm['tap'].Actions[-1]= timeout_action
  sm['tap'].NewAction()
  sm['tap'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['tap'].Actions[-1].Action= lambda: Vibrate(3)
  sm['tap'].Actions[-1].NextState= 'tap'
  sm['tap'].ElseAction.Condition= lambda: True
  sm['tap'].ElseAction.NextState= 'move_r_init'

  sm['move_r_init']= TFSMState()
  sm['move_r_init'].EntryAction= MoveRGripperToInit
  sm['move_r_init'].NewAction()
  sm['move_r_init'].Actions[-1]= poured_action
  sm['move_r_init'].NewAction()
  sm['move_r_init'].Actions[-1]= timeout_action
  sm['move_r_init'].ElseAction.Condition= lambda: True
  sm['move_r_init'].ElseAction.NextState= 'pour'

  sm['pour']= TFSMState()
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['pour'].Actions[-1].NextState= 'move_r_tap'
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: l.ControlStep(0.2 * t.flow_control_dtheta_max)  #FIXME: magic number
  sm['pour'].ElseAction.NextState= 'pour'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= lambda: (MoveRGripperToInit(), l.MoveBackToInit())
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  sm['stop'].ElseAction.NextState= EXIT_STATE

  sm.Run()

  t.SwitchArm(whicharm) #Original arm
