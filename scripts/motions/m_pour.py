#!/usr/bin/python
from core_tool import *
import copy
def Help():
  return '''Whole pouring procedure.
  Usage: pour BOTTLE_ID, CUP_ID [, AMOUNT_TRG [, MAX_DURATION, [CONSERVATIVE]]]
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    CUP_ID: identifier of cup. e.g. 'c1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)
    CONSERVATIVE: Robot becomes conservative, i.e. asking at each step (default=False)'''
def Run(t,args=()):
  bottle= args[0]
  cup= args[1]
  amount_trg= args[2] if len(args)>2 else 0.03
  max_duration= args[3] if len(args)>3 else 25.0
  conservative= args[4] if len(args)>4 else False

  #Use left hand
  whicharm= t.whicharm
  t.SwitchArm(1)

  def ExitProc():
    if 'base_marker_id' in t.attributes[bottle]:
      del t.attributes[bottle]['base_marker_id']
    if 'base_marker_id' in t.attributes[cup]:
      del t.attributes[cup]['base_marker_id']
    t.SwitchArm(whicharm)


  #Estimating bottle/cup positions from the AR markers
  #TODO: change by arguments
  m_id_bottle= 1
  m_id_cup= 2
  if (not m_id_bottle in t.ar_markers) or (not m_id_cup in t.ar_markers):
    print 'Marker observation: %i, %r' % (m_id_bottle, m_id_bottle in t.ar_markers)
    print 'Marker observation: %i, %r' % (m_id_cup, m_id_cup in t.ar_markers)
    return
  #t.attributes[bottle]['x']= t.ARX(m_id_bottle)
  #t.attributes[cup]['x']= t.ARX(m_id_cup)
  #Registering the base markers for inference (script infer is available)
  t.attributes[bottle]['base_marker_id']= m_id_bottle
  t.attributes[cup]['base_marker_id']= m_id_cup

  #Current control point in the wrist frame
  #lw_xe= t.control_frame[t.whicharm]
  #x_b= t.attributes[bottle]['x'] #Bottle base pose on the torso frame
  #if len(x_b)!=7:
    #print 'Bottle ',bottle,' pose is not observed'
    #ExitProc()
    #return

  ##FIXME
  #x_c= t.attributes[cup]['x'] #Cup base pose on the torso frame
  #if len(x_c)!=7:
    #print 'Cup',cup,' pose is not observed'
    #ExitProc()
    #return

  #Grab pose on the bottle frame:
  #lb_x_grab= t.attributes[bottle]['l_x_grab']
  #Grab pose on the torso frame
  #x_grab= Transform(x_b,lb_x_grab)
  #print 'x_grab=',VecToStr(x_grab)
  #t.CommandGripper(t.attributes[bottle]['g_pre'],50,True)
  #x_grab0= copy.deepcopy(x_grab)
  #x_grab0[0]= t.CartPos(lw_xe)[0]
  #t.MoveToCartPos(x_grab0,3.0,lw_xe,True)
  #t.MoveToCartPos(x_grab,3.0,lw_xe,True)


  #TODO: move to m_pour_est.py
  #Estimating grab pose l_x_grab:
  if 'l_x_grab_set' in t.attributes[bottle]:
    candidates= t.attributes[bottle]['l_x_grab_set']
    l_x_grab_avr= np.average(candidates,0)
    #Find a grab point whose z position is closest to the average
    i_closest= -1
    d_closest= 1.0e20
    for i in range(len(candidates)):
      if abs(candidates[i][2]-l_x_grab_avr[2]) < d_closest:
        d_closest= abs(candidates[i][2]-l_x_grab_avr[2])
        i_closest= i
    if i_closest>=0:
      t.attributes[bottle]['l_x_grab']= candidates[i_closest]
  if not 'l_x_grab' in t.attributes[bottle]:
    print 'Cannot estimate l_x_grab of',bottle
    ExitProc()
    return
  print 'l_x_grab=',t.attributes[bottle]['l_x_grab']

  print 'Continue motion?: pregrab and grab'
  if not conservative or AskYesNo():
    t.ExecuteMotion('pregrab',(bottle,'l'))
    t.ExecuteMotion('grab',(bottle,'l'))
  else:
    ExitProc()
    return

  #FIXME
  #Infere bottle pose
  t.ExecuteMotion('infer',(bottle,'x'))
  x_b= t.attributes[bottle]['x']
  if len(x_b)!=7:
    print 'Bottle ',bottle,' pose is not observed'
    ExitProc()
    return

  #TODO: move to m_pour_est.py
  #Estimating pouring edge point l_x_pour_e:
  if 'l_x_pour_e_set' in t.attributes[bottle]:
    t.ExecuteMotion('infer',(bottle,'x'))
    x_b= t.attributes[bottle]['x']
    t.ExecuteMotion('infer',(cup,'x'))
    x_c= t.attributes[cup]['x']
    #Choose a point that is closest to the cup
    candidates= t.attributes[bottle]['l_x_pour_e_set']
    i_closest= -1
    d_closest= 1.0e20
    for i in range(len(candidates)):
      candidate= Transform(x_b,candidates[i])
      d= la.norm(np.array(candidate[0:3])-np.array(x_c[0:3]))
      if d<d_closest:
        d_closest= d
        i_closest= i
    if i_closest>=0:
      t.attributes[bottle]['l_x_pour_e']= candidates[i_closest]
  if not 'l_x_pour_e' in t.attributes[bottle]:
    print 'Cannot estimate l_x_pour_e of',bottle
    ExitProc()
    return
  print 'l_x_pour_e=',t.attributes[bottle]['l_x_pour_e']


  #TODO: move to m_pour_est.py
  #Estimating pouring location l_x_pour_l on the cup frame:
  if 'l_x_pour_e_set' in t.attributes[cup]:
    t.ExecuteMotion('infer',(cup,'x'))
    x_c= t.attributes[cup]['x']
    t.ExecuteMotion('infer',(bottle,'x'))
    x_b= t.attributes[bottle]['x']
    #Choose a point that is the middle of the pouring edge center and the closest point from the bottle
    candidates= t.attributes[cup]['l_x_pour_e_set']
    l_x_pour_e_center= np.average(candidates,0)
    i_closest= -1
    d_closest= 1.0e20
    for i in range(len(candidates)):
      candidate= Transform(x_c,candidates[i])
      d= la.norm(np.array(candidate[0:3])-np.array(x_b[0:3]))
      if d<d_closest:
        d_closest= d
        i_closest= i
    if i_closest>=0:
      t.attributes[cup]['l_x_pour_l']= 0.5*np.array(l_x_pour_e_center)+0.5*np.array(candidates[i_closest])
      #FIXME: a magic parameter; 3cm above of the computed point
      t.attributes[cup]['l_x_pour_l'][2]+= 0.03
  if not 'l_x_pour_l' in t.attributes[cup]:
    print 'Cannot estimate l_x_pour_l of',cup
    ExitProc()
    return
  print 'l_x_pour_l=',t.attributes[cup]['l_x_pour_l']

  #Estimating initial pouring orientation q_pour_start:
  if 'pour_start_angle' in t.attributes[bottle]:
    pour_start_angle= t.attributes[bottle]['pour_start_angle']
    t.ExecuteMotion('infer',(bottle,'x'))
    x_b= np.array(t.attributes[bottle]['x'])
    t.ExecuteMotion('infer',(cup,'x'))
    x_c= np.array(t.attributes[cup]['x'])

    ax_gravity= [0,0,-1]  #TODO: define in attributes
    axis= np.cross(x_c[0:3]-x_b[0:3],ax_gravity)
    axis= axis / la.norm(axis)

    t.attributes[bottle]['q_pour_start']= QFromAxisAngle(axis,pour_start_angle)
  if not 'q_pour_start' in t.attributes[bottle]:
    print 'Cannot estimate q_pour_start of',bottle
  print 'q_pour_start=',t.attributes[bottle]['q_pour_start']

  print 'Continue motion?: prepour'
  if not conservative or AskYesNo():
    t.ExecuteMotion('prepour',(bottle,cup))
  else:
    ExitProc()
    return

  print 'Continue motion?: flowc_gen'
  if not conservative or AskYesNo():
    t.ExecuteMotion('flowc_gen',(bottle,amount_trg,max_duration))
  else:
    ExitProc()
    return

  print 'Continue motion?: backtog and release'
  if not conservative or AskYesNo():
    t.ExecuteMotion('backtog',(bottle,))
    t.ExecuteMotion('release',(bottle,))
  else:
    ExitProc()
    return

  ExitProc()

