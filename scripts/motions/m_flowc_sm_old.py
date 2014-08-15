#!/usr/bin/python
from core_tool import *
from state_machine import *
def Help():
  return '''Flow amount controller (ver 5: using state machine).  This is an old code; use "flowc_sm" instead.
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
class TLocal:
  what_is_this= 'This is an object to store variables shared in the states of the state machine.'
def Run(t,args=()):
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
  t.ExecuteMotion('infer',(bottle,'x'))
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

  #<<<FIXME: the above code is copied from m_flowc.py; a part of code is overwrapping with the below code!!!


  #State machine:
  l= TLocal()
  l.amount_trg= args[1] if len(args)>1 else 0.03
  l.max_duration= args[2] if len(args)>2 else 10.0

  l.rot_axis= axis
  l.max_theta= max_theta
  l.x_ext= lw_x_pour_e
  #l.trg_duration= args[4] if len(args)>4 else 8.0


  if not t.material_amount_observed:
    print "Error: /color_occupied_ratio is not observed"
    return

  l.i= t.whicharm
  #angles_init= t.mu.arm[l.i].getCurrentPosition()
  l.x_init= np.array(t.CartPos(l.x_ext))

  l.goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
  l.goal.trajectory.joint_names= t.mu.arm[l.i].goal.trajectory.joint_names
  l.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

  l.theta= 0.0
  #l.dtheta= 0.0
  #l.theta_prev= 0.0
  l.elapsed_time= 0.0
  l.damount= 0.0
  l.amount= t.material_amount
  l.amount_prev= t.material_amount
  #l.amount_base= t.material_amount

  now= time.localtime()
  l.tmpfp= file('%s/tmp/flowc%02i%02i%02i%02i%02i%02i.dat' % (os.environ['HOME'],now.tm_year%100,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec),'w')

  def IsFlowObserved(sensitivity=0.02):  #FIXME: using magic number
    threshold= l.amount_trg * sensitivity
    return t.material_amount-l.amount_prev > threshold
    #amount= t.material_amount
    #if amount-l.amount_base > threshold
      #l.amount_base= amount
      #return True
    #return False

  def IsPoured():
    if t.material_amount >= l.amount_trg:
      print 'Poured! (',t.material_amount,' / ',l.amount_trg,')'
      return True
    return False

  def IsTimeout():
    return l.elapsed_time > l.max_duration

  def ControlStep(dtheta):
    l.amount_prev= l.amount
    l.amount= t.material_amount

    if dtheta > t.flow_control_dtheta_max:  dtheta= t.flow_control_dtheta_max
    elif dtheta < t.flow_control_dtheta_min:  dtheta= t.flow_control_dtheta_min
    l.theta= l.theta+dtheta * t.flow_control_time_step
    if l.theta > l.max_theta:  l.theta= l.max_theta
    elif l.theta < 0.0:  l.theta= 0.0

    print l.elapsed_time,': ',l.amount,' / ',l.amount_trg,' : ',l.theta,', ',dtheta
    l.tmpfp.write('%f %f %f %f %f %f\n' % (rospy.Time.now().to_nsec(),l.amount,l.amount_trg,l.amount_trg,l.theta,dtheta))

    p_init,R_init= XToPosRot(l.x_init)
    dR= QToRot(QFromAxisAngle(l.rot_axis,l.theta))
    R_trg= np.dot(dR,R_init)
    x_trg= PosRotToX(p_init,R_trg)
    #print '##',VecToStr(x_trg)

    angles_curr= t.mu.arm[l.i].getCurrentPosition()
    resp= t.MakeIKRequest(x_trg, l.x_ext, angles_curr)

    if resp.error_code.val == 1:
      angles= np.array(resp.solution.joint_state.position)
      l.goal.trajectory.points[0].positions = angles
      l.goal.trajectory.points[0].time_from_start = rospy.Duration(t.flow_control_time_step)
      #angles_curr= angles

      l.goal.trajectory.header.stamp= rospy.Time.now()
      t.mu.arm[l.i].traj_client.send_goal(l.goal)
      start_time= rospy.Time.now()
      while rospy.Time.now() < start_time + rospy.Duration(t.flow_control_time_step):
        time.sleep(t.flow_control_time_step*0.02)

    else:
      print "IK error: ",resp.error_code.val
      return  #FIXME: error proc

    l.elapsed_time+= t.flow_control_time_step

  def MoveBackToInit():
    print 'Moving back to the initial pose...'
    t.MoveToCartPosI(l.x_init,3.0,l.x_ext,inum=30,blocking=True)


  sm= TStateMachine()

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= IsTimeout
  timeout_action.NextState= 'stop'

  poured_action= TFSMConditionedAction()
  poured_action.Condition= IsPoured
  poured_action.NextState= 'stop'

  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].NewAction()
  sm['start'].Actions[-1]= poured_action
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: IsFlowObserved(0.05)  #FIXME:magic number
  sm['start'].Actions[-1].NextState= 'pour'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: ControlStep(t.flow_control_dtheta_max)
  sm['start'].ElseAction.NextState= 'start'

  sm['pour']= TFSMState()
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= IsFlowObserved
  sm['pour'].Actions[-1].Action= lambda: ControlStep(0.0)
  sm['pour'].Actions[-1].NextState= 'pour'
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: ControlStep(0.5 * t.flow_control_dtheta_max)  #FIXME: magic number
  sm['pour'].ElseAction.NextState= 'pour'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= MoveBackToInit
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  sm['stop'].ElseAction.NextState= EXIT_STATE

  sm.Run()
