#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer attributes.
  Usage: infer OBJ_ID, ELEM_ID'''
def Run(t,args=()):
  obj= args[0]
  elem= args[1]

  def ExitProc():
    print 'Inferred: [',obj,'][',elem,']= ',t.attributes[obj][elem]

  #Infer x (pose)
  if elem=='x':
    #Infer x when obj is grabbed:
    if 'grabbed' in t.attributes[obj]:
      grabber_handid= t.attributes[obj]['grabbed']['grabber_handid']

      whicharm= t.whicharm
      t.whicharm= grabber_handid
      x_w= t.CartPos()
      t.whicharm= whicharm

      grabber_wrist= t.attributes[obj]['grabbed']['grabber_wrist']
      lw_xe= t.attributes[grabber_wrist]['l_x_gripper']
      lo_x_grab= t.attributes[obj]['l_x_grab']
      t.attributes[obj][elem]= TransformRightInv( Transform(x_w, lw_xe), lo_x_grab )
      #print '###lo_x_grab:',lo_x_grab
      #print '###x_grab:',Transform(t.attributes[obj][elem],lo_x_grab)
      #print '###x_e:',Transform(x_w, lw_xe)
      #print '###lw_xe:',lw_xe
      #print '###x_w:',x_w

      ExitProc()
      return

    #Infer x when reference marker is known:
    if 'ref_marker_id' in t.attributes[obj] and 'ref_marker_pose' in t.attributes[obj]:
      if t.attributes[obj]['ref_marker_id'] in t.ar_markers and len(t.attributes[obj]['ref_marker_pose'])==7:
        print '###infer from ref',t.attributes[obj]['ref_marker_id']
        x_m_ref= t.ARX(t.attributes[obj]['ref_marker_id'])
        t.attributes[obj][elem]= TransformRightInv(x_m_ref, t.attributes[obj]['ref_marker_pose'])
        ExitProc()
        return

    #Infer x when base marker is known:
    if 'base_marker_id' in t.attributes[obj]:
      print '###infer from base',t.attributes[obj]['base_marker_id']
      t.attributes[obj][elem]= t.ARX(t.attributes[obj]['base_marker_id'])
      ExitProc()
      return

  print 'Not inferred: [',obj,'][',elem,']'
