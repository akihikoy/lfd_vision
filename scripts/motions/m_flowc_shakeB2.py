#!/usr/bin/python
from core_tool import *
from state_machine_paa import *
def Help():
  return '''Flow amount controller with shaking type B.  The shake_width is automatically adjusted.
  Assumptions:
    Gripper holds a bottle
    Bottle is close to the cup
  Usage: flowc_shakeB2 BOTTLE_ID [, AMOUNT_TRG [, MAX_DURATION]]
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)'''
  #Usage: flowc_shakeB2 AMOUNT_TRG, ROT_AXIS, MAX_THETA, X_EXT, MAX_DURATION
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
  #l.shake_width= t.attributes[l.bottle]['shake_width1']
  #l.lb_axis_shake= t.attributes[l.bottle]['l_axis_shake3']
  #l.shake_width= t.attributes[l.bottle]['shake_width3']
  #l.lb_axis_shake= [1.0,0,-1.0]
  #l.lb_axis_shake= np.array(l.lb_axis_shake)/la.norm(l.lb_axis_shake)


  sm= TStateMachine()
  sm.Debug= True

  #NOTE: search the best shake_width
  #shake_ctrl=[shake_width,shake_freq]
  sm.Params['shake_ctrl']= TContParamNoGrad()
  shake_ctrl= sm.Params['shake_ctrl']
  #shake_ctrl.Mean= [0.04, 2.0]
  #shake_ctrl.Std= 0.02
  #shake_ctrl.Min= [0.01, 0.5]
  #shake_ctrl.Max= [0.06,  3.0]
  shake_ctrl.Mean= [0.06]
  shake_ctrl.Std= 0.02
  shake_ctrl.Min= [0.02]
  shake_ctrl.Max= [0.08]
  #shake_ctrl.Mean= [0.03]
  #shake_ctrl.Std= 0.02
  #shake_ctrl.Min= [0.01]
  #shake_ctrl.Max= [0.05]
  shake_ctrl.Init()
  def get_shake_ctrl():
    #return shake_ctrl.Param()
    return [shake_ctrl.Param()[0], 2.0]
    #return [shake_ctrl.Param()[0], 3.0]
    #return [0.04, 2.0]

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
  #sm['shake'].Actions[-1].Action= lambda: l.Shake(4, l.lb_axis_shake, l.shake_width, t.flow_shake_freq_max)
  sm['shake'].Actions[-1].Action= lambda: ( shake_ctrl.Select(), l.Shake(4, l.lb_axis_shake, get_shake_ctrl()[0], get_shake_ctrl()[1]), shake_ctrl.Update((t.material_amount-l.amount_prev)*get_shake_ctrl()[1]) )
  #(t.material_amount-l.amount_prev)*get_shake_ctrl()[1]: average changing of amount (Param()[1] is freq)
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

  #sm.Show()
  sm.Run()

  l.Close()