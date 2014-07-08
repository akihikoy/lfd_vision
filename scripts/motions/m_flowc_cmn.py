#!/usr/bin/python
from core_tool import *
from state_machine import *
def Help():
  return '''Common routines for flow amount controllers.  Do not use this directly.'''
def Run(t,args=()):
  print 'Error:',Help()

class TLocal:
  what_is_this= 'This is an object to store variables shared in the states of the state machine.'

  def Setup(self, t, bottle, amount_trg, max_duration):
    l= self
    self.t= t

    l.bottle= bottle
    l.amount_trg= amount_trg
    l.max_duration= max_duration

    print t.material_amount
    #lw_x_pour_e= t.control_frame[t.whicharm] #Local vector to the current control frame

    if not t.material_amount_observed:
      print "Error: /color_occupied_ratio is not observed"
      return False

    if not 'grabbed' in t.attributes[l.bottle]:
      print 'Error: not grabbed: ',l.bottle
      return False

    whicharm= t.whicharm
    t.SwitchArm(t.attributes[l.bottle]['grabbed']['grabber_handid'])


    #>>>FIXME: The same computation of lw_x_pour_e is used in m_pour.py
    #Infer l.bottle pose
    t.ExecuteMotion('infer',(l.bottle,'x'))
    x_b= t.attributes[l.bottle]['x']
    if len(x_b)!=7:
      print 'Bottle ',l.bottle,' pose is not observed'
      t.SwitchArm(whicharm)
      return False
    #Pouring edge point on the l.bottle frame:
    lb_x_pour_e= t.attributes[l.bottle]['l_x_pour_e']
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
    R_max= QToRot(t.attributes[l.bottle]['q_pour_max'])
    trans_R= np.dot(R_max,R_init.T)
    axis_angle= InvRodrigues(trans_R)
    max_theta= la.norm(axis_angle)
    axis= axis_angle / max_theta

    #<<<FIXME: the above code is copied from m_flowc.py; a part of code is overwrapping with the below code!!!

    print 'axis:',axis
    print 'max_theta:',max_theta


    l.rot_axis= axis
    l.max_theta= max_theta
    l.x_ext= lw_x_pour_e
    #l.trg_duration= args[4] if len(args)>4 else 8.0

    l.i= t.whicharm
    #angles_init= t.mu.arm[l.i].getCurrentPosition()
    l.x_init= np.array(t.CartPos(l.x_ext))
    l.x_init2= copy.deepcopy(l.x_init)

    l.goal= pr2_controllers_msgs.msg.JointTrajectoryGoal()
    l.goal.trajectory.joint_names= t.mu.arm[l.i].goal.trajectory.joint_names
    l.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())

    l.theta= 0.0
    #l.dtheta= 0.0
    #l.theta_prev= 0.0
    l.elapsed_time= 0.0
    l.timer_time= 0.0
    l.damount= 0.0
    l.amount= t.material_amount
    l.amount_prev= t.material_amount

    l.m_infer= t.LoadMotion('infer')

    l.initialized_time= rospy.Time.now()
    l.is_poured= False
    l.first_poured_time= -1.0

    now= time.localtime()
    l.log_file_name= '%s/tmp/flowc%02i%02i%02i%02i%02i%02i.dat' % (os.environ['HOME'],now.tm_year%100,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec)
    l.log_file_nameB= '%s/tmp/flowcB%02i%02i%02i%02i%02i%02i.dat' % (os.environ['HOME'],now.tm_year%100,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec)
    l.ctrl_type= '-'
    print 'Logging to',l.log_file_name,', ',l.log_file_nameB
    l.tmpfp= file(l.log_file_name,'w')
    l.logfp= file(l.log_file_nameB,'w')
    t.amount_observer_callback= l.Logger

    return True

  def __del__(self):
    l= self; t= self.t
    l.Close()

  def Close(self):
    l= self; t= self.t
    t.amount_observer_callback= None
    if not l.tmpfp.closed:
      l.tmpfp.close()
      print 'Logged to',l.log_file_name
    if not l.logfp.closed:
      l.logfp.close()
      print 'Logged to',l.log_file_nameB

  def Reset(self):
    l= self; t= self.t
    l.Close()
    l.Setup(t, l.bottle, l.amount_trg, l.max_duration)

  def Logger(self):
    l= self; t= self.t
    l.logfp.write('%f %f %f %f %s\n' % (rospy.Time.now().to_nsec(),l.amount,l.amount_trg,l.theta,l.ctrl_type))

  def IsFlowObserved(self, sensitivity=0.01):  #FIXME: using magic number
    l= self; t= self.t
    threshold= sensitivity
    return t.material_amount-l.amount_prev > threshold

  def IsPoured(self):
    l= self; t= self.t
    if t.material_amount >= l.amount_trg:
      print 'Poured! (',t.material_amount,' / ',l.amount_trg,')'
      if not l.is_poured:
        l.is_poured= True
        l.first_poured_time= rospy.Time.now().to_sec() - l.initialized_time.to_sec()
      return True
    return False

  def IsTimeout(self):
    l= self; t= self.t
    if l.elapsed_time > l.max_duration:
      print '###TIMEOUT!### (',t.material_amount,' / ',l.amount_trg,')'
      return True
    return False

  def ChargeTimer(self,dt):
    l= self; t= self.t
    l.timer_time= l.elapsed_time+dt

  def IsTimerTimeout(self):
    l= self; t= self.t
    return l.timer_time<=l.elapsed_time

  def IsThetaEqTo(self,th,threshold_rate=0.001):
    l= self; t= self.t
    threshold= threshold_rate*l.max_theta
    return abs(l.theta-th) < threshold

  def ControlStep(self,dtheta):
    l= self; t= self.t
    l.ctrl_type= 'c'
    l.amount_prev= l.amount
    l.amount= t.material_amount

    if dtheta > t.flow_control_dtheta_max:  dtheta= t.flow_control_dtheta_max
    elif dtheta < t.flow_control_dtheta_min:  dtheta= t.flow_control_dtheta_min
    l.theta= l.theta+dtheta * t.flow_control_time_step
    if l.theta > l.max_theta:  l.theta= l.max_theta
    elif l.theta < 0.0:  l.theta= 0.0

    print l.elapsed_time,': ',l.amount,' / ',l.amount_trg,' : ',l.theta,', ',dtheta
    l.tmpfp.write('%f %f %f %f %f %f c\n' % (rospy.Time.now().to_nsec(),l.amount,l.amount_trg,l.amount_trg,l.theta,dtheta))

    p_init,R_init= XToPosRot(l.x_init)
    dR= QToRot(QFromAxisAngle(l.rot_axis,l.theta))
    R_trg= np.dot(dR,R_init)
    x_trg= PosRotToX(p_init,R_trg)
    #print '##',VecToStr(x_trg)

    #Store x_trg in future use (cf. Shake)
    l.x_init2= copy.deepcopy(x_trg)

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
      raise

    l.elapsed_time+= t.flow_control_time_step

  def ControlStepN(self,dtheta,count):
    l= self; t= self.t
    for i in range(count):
      ControlStep(l,dtheta)

  def MoveBackToInit(self):
    l= self; t= self.t
    l.ctrl_type= 'mbi'
    print 'Moving back to the initial pose...'
    l.tmpfp.write('%f %f %f %f %f %f mbi\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
    t.MoveToCartPosI(l.x_init,3.0,l.x_ext,inum=30,blocking=True)
    l.tmpfp.write('%f %f %f %f %f %f mbi\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))


  def Shake(self,count,lb_axis_shake,shake_width,shake_freq):
    l= self; t= self.t
    l.ctrl_type= 's2'
    l.amount_prev= l.amount
    l.amount= t.material_amount

    print 'shake:',l.elapsed_time,': ',l.amount,' / ',l.amount_trg,' : ',shake_freq
    l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),l.amount,l.amount_trg,l.amount_trg,shake_freq,-999))

    dt= 1.0/shake_freq

    #>>>Shaking motion
    x_trg1= copy.deepcopy(l.x_init2)
    x_trg2= copy.deepcopy(l.x_init2)
    l.m_infer.Run(t,(l.bottle,'x'))
    x_b= t.attributes[l.bottle]['x']  #Bottle pose in robot frame
    p_b,R_b= XToPosRot(x_b)
    axis_shake= np.dot(R_b,lb_axis_shake)  #Shaking axis in robot frame
    axis_shake= np.array(axis_shake) / la.norm(axis_shake)
    x_trg1[0:3]+= np.array(axis_shake)*(0.5*shake_width)
    x_trg2[0:3]-= np.array(axis_shake)*(0.5*shake_width)

    #inum= 5
    inum= 10
    t.MoveToCartPosI(x_trg1,dt/2.0,l.x_ext,inum,blocking=True)
    l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,shake_freq,-999))
    l.elapsed_time+= dt/2.0
    for n in range(count-1):
      t.MoveToCartPosI(x_trg2,dt/2.0,l.x_ext,inum,blocking=True)
      l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,shake_freq,-999))
      t.MoveToCartPosI(x_trg1,dt/2.0,l.x_ext,inum,blocking=True)
      l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,shake_freq,-999))
      l.elapsed_time+= dt
    t.MoveToCartPosI(x_trg2,dt/2.0,l.x_ext,inum,blocking=True)
    l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,shake_freq,-999))
    l.elapsed_time+= dt/2.0
    t.MoveToCartPosI(l.x_init2,dt/2.0,l.x_ext,inum,blocking=True)
    l.tmpfp.write('%f %f %f %f %f %f s2\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,shake_freq,-999))
    l.elapsed_time+= dt/2.0
    #<<<Shaking motion
