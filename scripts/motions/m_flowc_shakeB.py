#!/usr/bin/python
from core_tool import *
from state_machine_paa import *
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

  #Shaking axis in bottle frame
  l.lb_axis_shake= t.attributes[l.bottle]['l_axis_shake1']
  l.shake_width= t.attributes[l.bottle]['shake_width1']


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
  sm['start'].Actions[-1].Condition= lambda: l.IsFlowObserved(0.003)  #FIXME
  sm['start'].Actions[-1].NextState= 'shake'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: not l.IsFlowObserved(0.003) and l.IsThetaEqTo(l.max_theta)
  sm['start'].Actions[-1].NextState= 'to_initial'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: l.ControlStep(2.0*t.flow_control_dtheta_max)
  sm['start'].ElseAction.NextState= 'start'

  sm['shake']= TFSMState()
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= poured_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= timeout_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= lambda: l.IsFlowObserved(0.003)
  sm['shake'].Actions[-1].Action= lambda: l.Shake(4, l.lb_axis_shake, l.shake_width, t.flow_shake_freq_max)
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
  sm['to_initial'].Actions[-1].Condition= lambda: l.IsFlowObserved(0.003)  #FIXME
  sm['to_initial'].Actions[-1].NextState= 'shake'
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: l.IsThetaEqTo(0.0)
  sm['to_initial'].Actions[-1].NextState= 'start'
  sm['to_initial'].ElseAction.Condition= lambda: True
  sm['to_initial'].ElseAction.Action= lambda: l.ControlStep(2.0*t.flow_control_dtheta_min)
  sm['to_initial'].ElseAction.NextState= 'to_initial'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= l.MoveBackToInit
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  sm['stop'].ElseAction.NextState= EXIT_STATE

  #sm.Show()
  sm.Run()

