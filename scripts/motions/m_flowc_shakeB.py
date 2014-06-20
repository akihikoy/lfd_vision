#!/usr/bin/python
from core_tool import *
from state_machine import *
def Help():
  return '''Flow amount controller with shaking type B.
  Assumptions:
    Gripper holds a bottle
    Bottle is close to the cup
  Usage: flowc_shakeB BOTTLE_ID [, AMOUNT_TRG [, MAX_DURATION]]
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)'''
  #Usage: flowc_shakeB AMOUNT_TRG, ROT_AXIS, MAX_THETA, X_EXT, MAX_DURATION
    #AMOUNT_TRG: Target amount
    #ROT_AXIS: Rotation axis
    #MAX_THETA: Maximum theta
    #X_EXT: Control point (default=[])
    #MAX_DURATION: Maximum duration (default=10.0)
    ##TRG_DURATION: Target duration (default=8.0)
def Run(t,args=()):
  bottle= args[0]
  amount_trg= args[1] if len(args)>1 else 0.03
  max_duration= args[2] if len(args)>2 else 25.0

  m_flowc_cmn= t.LoadMotion('flowc_cmn')
  l= m_flowc_cmn.TLocal()
  if not l.Setup(t, bottle,amount_trg,max_duration):
    return

  l.m_infer= t.LoadMotion('infer')
  #Shaking axis in bottle frame
  l.lb_axis_shake= t.attributes[l.bottle]['l_axis_shake']


  def Shake(count,shake_freq):
    l.amount_prev= l.amount
    l.amount= t.material_amount

    print 'shake:',l.elapsed_time,': ',l.amount,' / ',l.amount_trg,' : ',shake_freq
    l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),l.amount,l.amount_trg,l.amount_trg,shake_freq,-999))

    dt= 1.0/shake_freq

    #>>>Shaking motion
    x_trg= copy.deepcopy(l.x_init2)
    l.m_infer.Run(t,(l.bottle,'x'))
    x_b= t.attributes[l.bottle]['x']  #Bottle pose in robot frame
    p_b,R_b= XToPosRot(x_b)
    axis_shake= np.dot(R_b,l.lb_axis_shake)  #Shaking axis in robot frame
    axis_shake= np.array(axis_shake) / la.norm(axis_shake)
    x_trg[0:3]+= np.array(axis_shake)*t.flow_shake_width
    for n in range(count):
      t.MoveToCartPosI(x_trg,dt/2.0,l.x_ext,inum=5,blocking=True)
      l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,shake_freq,-999))
      t.MoveToCartPosI(l.x_init2,dt/2.0,l.x_ext,inum=5,blocking=True)
      l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,shake_freq,-999))
      l.elapsed_time+= dt
    #<<<Shaking motion


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
  sm['start'].Actions[-1].Condition= lambda: l.IsFlowObserved(0.05)  #FIXME
  sm['start'].Actions[-1].NextState= 'shake'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: not l.IsFlowObserved() and l.IsThetaEqTo(l.max_theta)
  sm['start'].Actions[-1].NextState= 'to_initial'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_max)
  sm['start'].ElseAction.NextState= 'start'

  sm['shake']= TFSMState()
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= poured_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= timeout_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= l.IsFlowObserved
  sm['shake'].Actions[-1].Action= lambda: Shake(4,t.flow_shake_freq_max)
  sm['shake'].Actions[-1].NextState= 'shake'
  #sm['shake'].NewAction()
  #sm['shake'].Actions[-1].Condition= lambda: not l.IsFlowObserved() and l.IsThetaEqTo(l.max_theta)
  #sm['shake'].Actions[-1].NextState= 'to_initial'
  #sm['shake'].ElseAction.Condition= lambda: True
  #N= int(0.1*max_theta/(t.flow_control_dtheta_max*t.flow_control_time_step))
  #sm['shake'].ElseAction.Action= lambda: (l.ControlStepN(1.0 * t.flow_control_dtheta_max,N), Shake(1,t.flow_shake_freq_max))  #FIXME: magic number
  #sm['shake'].ElseAction.NextState= 'shake'
  sm['shake'].ElseAction.Condition= lambda: True
  sm['shake'].ElseAction.NextState= 'to_initial'

  sm['to_initial']= TFSMState()
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= poured_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= timeout_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: l.IsThetaEqTo(0.0)
  sm['to_initial'].Actions[-1].NextState= 'start'
  sm['to_initial'].ElseAction.Condition= lambda: True
  sm['to_initial'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_min)
  sm['to_initial'].ElseAction.NextState= 'to_initial'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= l.MoveBackToInit
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  sm['stop'].ElseAction.NextState= EXIT_STATE

  #sm.Show()
  sm.Run()

