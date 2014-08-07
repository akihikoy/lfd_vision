#!/usr/bin/python
from core_tool import *
from state_machine import *
def Help():
  return '''Flow amount controller (ver 5: using state machine).
  Assumptions:
    Gripper holds a bottle
    Bottle is close to the cup
  Usage: flowc_sm BOTTLE_ID [, AMOUNT_TRG [, MAX_DURATION]]
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=10.0)'''
  #Usage: flowc_sm AMOUNT_TRG, ROT_AXIS, MAX_THETA, X_EXT, MAX_DURATION
    #AMOUNT_TRG: Target amount
    #ROT_AXIS: Rotation axis
    #MAX_THETA: Maximum theta
    #X_EXT: Control point (default=[])
    #MAX_DURATION: Maximum duration (default=10.0)
    ##TRG_DURATION: Target duration (default=8.0)
def Run(t,args=()):
  bottle= args[0]
  amount_trg= args[1] if len(args)>1 else 0.03
  max_duration= args[2] if len(args)>2 else 10.0

  m_flowc_cmn= t.LoadMotion('flowc_cmn')
  l= m_flowc_cmn.TLocal()
  if not l.Setup(t, bottle,amount_trg,max_duration):
    return

  l.flow_obs_sensitivity= 0.003

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
  sm['start'].Actions[-1].NextState= 'pour'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_max)
  sm['start'].ElseAction.NextState= 'start'

  sm['pour']= TFSMState()
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['pour'].Actions[-1].Action= lambda: l.ControlStep(0.0)
  sm['pour'].Actions[-1].NextState= 'pour'
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: l.ControlStep(0.5 * t.flow_control_dtheta_max)  #FIXME: magic number
  sm['pour'].ElseAction.NextState= 'pour'

  sm['stop']= TFSMState()
  #sm['stop'].EntryAction= l.MoveBackToInit
  #sm['stop'].ElseAction.Condition= lambda: True
  #sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  #sm['stop'].ElseAction.NextState= EXIT_STATE
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: l.IsThetaEqTo(0.0)
  sm['stop'].Actions[-1].Action= lambda: Print('End of pouring')
  sm['stop'].Actions[-1].NextState= EXIT_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_min)
  sm['stop'].ElseAction.NextState= ORIGIN_STATE

  sm.Run()

  l.Close()
