#!/usr/bin/python
from core_tool import *
from state_machine_paa import *
def Help():
  return '''General flow amount controller.
  Assumptions:
    Gripper holds a bottle
    Bottle is close to the cup
  Usage: flowc_gen BOTTLE_ID [, AMOUNT_TRG [, MAX_DURATION]]
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)'''
  #Usage: flowc_gen AMOUNT_TRG, ROT_AXIS, MAX_THETA, X_EXT, MAX_DURATION
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
  #l.lb_axis_shake= t.attributes[l.bottle]['l_axis_shake1']
  #l.shake_width= t.attributes[l.bottle]['shake_width1']
  #l.lb_axis_shake= t.attributes[l.bottle]['l_axis_shake3']
  #l.shake_width= t.attributes[l.bottle]['shake_width3']
  l.lb_axis_shake= [0.0,0.0,-1.0]
  #l.lb_axis_shake= np.array(l.lb_axis_shake)/la.norm(l.lb_axis_shake)
  #l.shake_width= 0.03
  #l.shake_freq= 2.0
  l.shake_width= 0.025
  l.shake_freq= 2.5
  #l.shake_freq= 3.0
  l.shake_widthA= 0.06

  l.flow_obs_sensitivity= 0.003

  sm= TStateMachine()
  sm.Debug= True


  #Search the best trick_id
  sm.Params['trick_id']= TDiscParam()
  trick_id= sm.Params['trick_id']
  trick_id.Candidates= ['std_pour','shake_A','shake_B']
  trick_id.Means= [1.0,0.5,0.5]
  trick_id.SqMeans= [1.0+1.0,0.25+1.0,0.25+1.0]
  trick_id.Init()
  def select_trick_id():
    l.trick_id_amount_begin= t.material_amount
    l.trick_id_time_begin= l.elapsed_time
    trick_id.Select()
  def get_trick_id():
    return trick_id.Param()
    #return 'shake_B'
  def update_trick_id():
    if trick_id.Param():
      score= (t.material_amount - l.trick_id_amount_begin) / (l.elapsed_time - l.trick_id_time_begin)
      trick_id.Update(score)

  #Search the best lb_axis_shake
  sm.Params['shake_axis_theta']= TContParamNoGrad()
  shake_axis_theta= sm.Params['shake_axis_theta']
  shake_axis_theta.Mean= [math.pi/4.0]
  shake_axis_theta.Std= 1.0
  shake_axis_theta.Min= [0.0]
  shake_axis_theta.Max= [math.pi/2.0]
  shake_axis_theta.Init()
  def select_shake_axis():
    l.shake_axis_amount_begin= t.material_amount
    l.shake_axis_time_begin= l.elapsed_time
    shake_axis_theta.Select()
  def get_shake_axis():
    return [math.sin(shake_axis_theta.Param()[0]),0.0,-math.cos(shake_axis_theta.Param()[0])]
  def update_shake_axis():
    score= (t.material_amount - l.shake_axis_amount_begin) / (l.elapsed_time - l.shake_axis_time_begin)
    shake_axis_theta.Update(score)


  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= l.IsTimeout
  timeout_action.NextState= 'stop'

  poured_action= TFSMConditionedAction()
  poured_action.Condition= l.IsPoured
  poured_action.NextState= 'start'
  #poured_action.NextState= 'stop'

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].EntryAction= lambda: ( update_trick_id(), select_trick_id() )
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= l.IsPoured
  sm['start'].Actions[-1].NextState= 'stop'
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: get_trick_id()=='std_pour'
  sm['start'].Actions[-1].NextState= 'std_pour'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: get_trick_id()=='shake_A'
  sm['start'].Actions[-1].NextState= 'shake_A'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: get_trick_id()=='shake_B'
  sm['start'].Actions[-1].NextState= 'shake_B'

  #Standard flow amount controller
  sm.NewState('std_pour')
  sm['std_pour'].ElseAction.Condition= lambda: True
  sm['std_pour'].ElseAction.NextState= 'to_initial'

  sm.NewState('to_initial')
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= poured_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= timeout_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: l.IsThetaEqTo(0.0)
  sm['to_initial'].Actions[-1].NextState= 'find_flow_p'
  sm['to_initial'].ElseAction.Condition= lambda: True
  sm['to_initial'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_min)
  sm['to_initial'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('find_flow_p')
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= poured_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= timeout_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['find_flow_p'].Actions[-1].NextState= 'pour'
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: l.IsThetaEqTo(l.max_theta)
  sm['find_flow_p'].Actions[-1].NextState= 'start'
  sm['find_flow_p'].ElseAction.Condition= lambda: True
  sm['find_flow_p'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_max)
  sm['find_flow_p'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('pour')
  sm['pour'].EntryAction= lambda: l.ChargeTimer(0.2)
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['pour'].Actions[-1].Action= lambda: ( l.ChargeTimer(0.2), l.ControlStep(0.0) )
  sm['pour'].Actions[-1].NextState= ORIGIN_STATE
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: l.IsTimerTimeout()
  sm['pour'].Actions[-1].NextState= 'start'
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: l.ControlStep(0.5 * t.flow_control_dtheta_max)  #FIXME: magic number
  sm['pour'].ElseAction.NextState= ORIGIN_STATE

  #Shake-A flow amount controller
  sm.NewState('shake_A')
  sm['shake_A'].ElseAction.Condition= lambda: True
  sm['shake_A'].ElseAction.NextState= 'to_max'

  sm.NewState('to_max')
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1]= poured_action
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1]= timeout_action
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1].Condition= lambda: l.IsThetaEqTo(l.max_theta)
  sm['to_max'].Actions[-1].NextState= 'shake'
  sm['to_max'].ElseAction.Condition= lambda: True
  sm['to_max'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_max)
  sm['to_max'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('shake')
  sm['shake'].EntryAction= lambda: l.ChargeTimer(2.0)
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= poured_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= timeout_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['shake'].Actions[-1].Action= lambda: ( l.ChargeTimer(2.0), l.Shake(2, l.lb_axis_shake, l.shake_widthA, l.shake_freq) )
  sm['shake'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= lambda: not l.IsTimerTimeout()
  sm['shake'].Actions[-1].Action= lambda: ( l.Shake(2, l.lb_axis_shake, l.shake_widthA, l.shake_freq) )
  sm['shake'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake'].ElseAction.Condition= lambda: True
  sm['shake'].ElseAction.NextState= 'start'

  #Shake-B flow amount controller
  sm.NewState('shake_B')
  sm['shake_B'].ElseAction.Condition= lambda: True
  sm['shake_B'].ElseAction.NextState= 'find_flow_m'

  sm.NewState('find_flow_m')
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1]= poured_action
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1]= timeout_action
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['find_flow_m'].Actions[-1].NextState= 'shake2'
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1].Condition= lambda: l.IsThetaEqTo(0.0)
  sm['find_flow_m'].Actions[-1].NextState= 'find_flow_p2'
  sm['find_flow_m'].ElseAction.Condition= lambda: True
  sm['find_flow_m'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_min)
  sm['find_flow_m'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('find_flow_p2')
  sm['find_flow_p2'].NewAction()
  sm['find_flow_p2'].Actions[-1]= poured_action
  sm['find_flow_p2'].NewAction()
  sm['find_flow_p2'].Actions[-1]= timeout_action
  sm['find_flow_p2'].NewAction()
  sm['find_flow_p2'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['find_flow_p2'].Actions[-1].NextState= 'shake2'
  sm['find_flow_p2'].NewAction()
  sm['find_flow_p2'].Actions[-1].Condition= lambda: l.IsThetaEqTo(l.max_theta)
  sm['find_flow_p2'].Actions[-1].NextState= 'shake2'
  sm['find_flow_p2'].ElseAction.Condition= lambda: True
  sm['find_flow_p2'].ElseAction.Action= lambda: l.ControlStep(t.flow_control_dtheta_max)
  sm['find_flow_p2'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('shake2')
  sm['shake2'].EntryAction= lambda: l.ChargeTimer(2.0)
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1]= poured_action
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1]= timeout_action
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['shake2'].Actions[-1].Action= lambda: ( l.ChargeTimer(2.0), select_shake_axis(), l.Shake(2, get_shake_axis(), l.shake_width, l.shake_freq), update_shake_axis() )
  sm['shake2'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1].Condition= lambda: not l.IsTimerTimeout()
  sm['shake2'].Actions[-1].Action= lambda: ( select_shake_axis(), l.Shake(2, get_shake_axis(), l.shake_width, l.shake_freq), update_shake_axis() )
  sm['shake2'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake2'].ElseAction.Condition= lambda: True
  sm['shake2'].ElseAction.NextState= 'start'


  sm.NewState('stop')
  sm['stop'].EntryAction= l.MoveBackToInit
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  sm['stop'].ElseAction.NextState= EXIT_STATE

  #sm.Show()
  sm.Run()
  for (key,param) in sm.Params.items():
    print 'Param ',key,':'
    if isinstance(param,TDiscParam):
      print '  Means:',param.Means
      print '  UCB:',param.UCB()
      print '  SqMeans:',param.SqMeans
    elif isinstance(param,TContParamGrad):
      print '  Mean:',param.Mean
    elif isinstance(param,TContParamNoGrad):
      print '  xopt:',param.es.result()[0]
      print '  fopt:',param.es.result()[1]
      print '  xmean:',param.es.result()[5]
      print '  stds:',param.es.result()[6]

