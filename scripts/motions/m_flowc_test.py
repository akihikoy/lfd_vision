#!/usr/bin/python
from core_tool import *
def Help():
  return '''Flow amount controller (copy of ver 4 to test to write as a motion script).
  Usage: flowc_test AMOUNT_TRG, ROT_AXIS, MAX_THETA, X_EXT, TRG_DURATION, MAX_DURATION
    AMOUNT_TRG: Target amount
    ROT_AXIS: Rotation axis
    MAX_THETA: Maximum theta
    X_EXT: Control point (default=[])
    TRG_DURATION: Target duration (default=8.0)
    MAX_DURATION: Maximum duration (default=10.0)'''
def Run(t,args=()):
  amount_trg= args[0]
  rot_axis= args[1]
  max_theta= args[2]
  x_ext= args[3] if len(args)>3 else []
  trg_duration= args[4] if len(args)>4 else 8.0
  max_duration= args[5] if len(args)>5 else 10.0


  if not t.material_amount_observed:
    print "Error: /color_occupied_ratio is not observed"
    return

  i= t.whicharm
  #angles_init= t.mu.arm[i].getCurrentPosition()
  x_init= np.array(t.CartPos(x_ext))

  goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
  goal.trajectory.joint_names= t.mu.arm[i].goal.trajectory.joint_names
  goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

  theta= 0.0
  dtheta= 0.0
  theta_prev= 0.0
  elapsed_time= 0.0
  damount= 0.0
  amount= t.material_amount
  now= time.localtime()
  tmpfp= file('%s/tmp/flowc%02i%02i%02i%02i%02i%02i.dat' % (os.environ['HOME'],now.tm_year%100,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec),'w')
  while elapsed_time<max_duration:
    amount_prev= amount
    amount= t.material_amount
    if amount >= amount_trg:
      print 'Poured! (',amount,' / ',amount_trg,')'
      break

    theta_prev= theta

    t.flow_control_kind= 4
    if t.flow_control_kind==4:
      #damount= (amount-amount_prev)/t.flow_control_time_step
      amount_trg_t= amount_trg/trg_duration * elapsed_time
      if amount_trg_t>amount_trg: amount_trg_t= amount_trg
      if amount_trg_t - amount>=0:
        dtheta= t.flow_control_gain_p41 * (amount_trg_t - amount) - t.flow_control_gain_d41 * dtheta
      else:
        dtheta= t.flow_control_gain_p42 * (amount_trg_t - amount) - t.flow_control_gain_d42 * dtheta
      if dtheta > t.flow_control_dtheta_max:  dtheta= t.flow_control_dtheta_max
      elif dtheta < t.flow_control_dtheta_min:  dtheta= t.flow_control_dtheta_min
      theta= theta+dtheta * t.flow_control_time_step
      if theta > max_theta:  theta= max_theta
      elif theta < 0.0:  theta= 0.0
      print elapsed_time,': ',amount,' / ',amount_trg_t,' : ',theta,', ',dtheta
      tmpfp.write('%f %f %f %f %f %f\n' % (rospy.Time.now().to_nsec(),amount,amount_trg_t,amount_trg,theta,dtheta))

    p_init,R_init= XToPosRot(x_init)
    dR= QToRot(QFromAxisAngle(rot_axis,theta))
    R_trg= np.dot(dR,R_init)
    x_trg= PosRotToX(p_init,R_trg)
    #print '##',VecToStr(x_trg)

    angles_curr= t.mu.arm[i].getCurrentPosition()
    resp= t.MakeIKRequest(x_trg, x_ext, angles_curr)

    if resp.error_code.val == 1:
      angles= np.array(resp.solution.joint_state.position)
      goal.trajectory.points[0].positions = angles
      goal.trajectory.points[0].time_from_start = rospy.Duration(t.flow_control_time_step)
      #angles_curr= angles

      goal.trajectory.header.stamp= rospy.Time.now()
      t.mu.arm[i].traj_client.send_goal(goal)
      start_time= rospy.Time.now()
      while rospy.Time.now() < start_time + rospy.Duration(t.flow_control_time_step):
        time.sleep(t.flow_control_time_step*0.02)

    else:
      print "IK error: ",resp.error_code.val
      break

    elapsed_time+= t.flow_control_time_step



