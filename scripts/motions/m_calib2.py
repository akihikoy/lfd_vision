#!/usr/bin/python
from core_tool import *
def Help():
  return '''Calibrate external RGB-D sensor.
  Usage: calib2 MARKER_ID
    Execute the calibration. Follow the instruction.
    MARKER_ID: Marker ID used to calibration (default=0)'''
def Run(t,args=()):
  m_id= args[0] if len(args)>0 else 0

  calib_angle= -0.5*math.pi
  obs_count= 200
  duration= 5

  if not m_id in t.ar_markers:
    print 'Marker not observed: %i' % m_id
    return

  #whicharm= t.whicharm
  #if whicharm!=1:
    #t.SwitchArm(1) #Left arm
  #else:
    #whicharm= -1

  #def exit_proc:
    #if whicharm!=-1:
      #t.SwitchArm(whicharm)

  #print 'Start calibration. ',t.ArmStr(),'arm is relaxed. OK?'
  #if not AskYesNo():
    ##exit_proc()
    #return

  #print '###CAUTION:',t.ArmStr(),'arm is relaxed'
  #t.ActivateMannController()
  #while True:
    #print 'Move',t.ArmStr(),'arm to preferred pose (wrist-roll should be zero)'
    #print 'Is it OK?'
    #print 'If say Y,',t.ArmStr(),'wrist-roll joint moves to zero'
    #print 'Then,',t.ArmStr(),'wrist-flex joint moves %f degree' % (calib_angle*180.0/math.pi)
    #if AskYesNo():
      #break
  #print '###CAUTION:',t.ArmStr(),'arm is fixed'
  #t.ActivateStdController()

  print 'Moveing',t.ArmStr(),'wrist to zero...'
  i= t.whicharm
  angles_trg= list(t.mu.arm[i].getCurrentPosition())
  print angles_trg
  angles_trg[-1]= 0.0
  t.MoveToJointPos(angles_trg,dt=2.0,blocking=True)

  print 'After 0.5s,',t.ArmStr(),'wrist-flex joint moves %f degree' % (calib_angle*180.0/math.pi)
  print 'OK?'
  if not AskYesNo():
    #exit_proc()
    return

  print 'After 0.5s,',t.ArmStr(),'wrist-flex joint moves %f degree' % (calib_angle*180.0/math.pi)
  del t.ar_markers[m_id]
  time.sleep(0.5)


  marker_data= []
  gripper_data= []

  goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
  goal.trajectory.joint_names= t.mu.arm[i].goal.trajectory.joint_names
  goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

  angles_trg= list(t.mu.arm[i].getCurrentPosition())
  angle_step= calib_angle/float(obs_count)
  time_step= duration/float(obs_count)
  for k in range(obs_count):
    if m_id in t.ar_markers:
      x= t.ar_markers[m_id]
      xp= x.position
      xq= x.orientation
      marker_data.append([xp.x,xp.y,xp.z, xq.x,xq.y,xq.z,xq.w])
      gripper_data.append(t.CartPos())  # Node: position[l_wrist_flex_link]==position[l_wrist_roll_link]

      del t.ar_markers[m_id]

    angles_trg[-2]+= angle_step
    goal.trajectory.points[0].positions = angles_trg
    goal.trajectory.points[0].time_from_start = rospy.Duration(time_step)
    goal.trajectory.header.stamp= rospy.Time.now()
    t.mu.arm[i].traj_client.send_goal(goal)
    start_time= rospy.Time.now()
    while rospy.Time.now() < start_time + rospy.Duration(time_step):
      time.sleep(time_step*0.02)


  x_center, radius= CircleFit3D(marker_data)

  print 'x_center,radius=',x_center,radius

  #print "radius=",radius
  #print "marker_data=",marker_data
  #print "gripper_data=",gripper_data


  x_g2m= [radius,0.0,0.0, 0,0,0,1]
  p,R= XToPosRot(gripper_data[0])
  print 'R[2,1]=',R[2,1]
  if R[2,1]>0:  # gripper ey axis is up
    x_g2m[3:]= QFromAxisAngle([1,0,0],-math.pi*0.5)
  else:  # gripper ey axis is down
    x_g2m[3:]= QFromAxisAngle([1,0,0],math.pi*0.5)
  print 'x_g2m=',x_g2m
  t.x_sensor= CalibrateSensorPose(marker_data, gripper_data, x_g2m)

  print 'Done.'
  print 'x_sensor=',t.x_sensor

  #exit_proc()

