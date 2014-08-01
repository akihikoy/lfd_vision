#!/usr/bin/python
from core_tool import *
def Help():
  return '''Define object properties and store them into attributes (ver. 2).
  Usage: objs2'''
def Run(t,args=()):
  #NOTE: an attribute named 'l_*' denotes a vector defined on the local frame

  #FIXME: loading from YAML is very slow.  on-demand loading should be required

  InsertYAML(t.attributes, 'models/robot/gripper.yaml')

  AddSubDict(t.attributes, 'b50')
  InsertYAML(t.attributes['b50'], 'models/obj/b50.yaml')
  #FIXME: the following values should be DEFAULT values
  #Gripper width before grab
  t.attributes['b50']['g_pre']= 0.08
  #Grab power (max effort):
  t.attributes['b50']['f_grab']= 50.0
  #Orientation to start pouring
  t.attributes['b50']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
  #Orientation where the flow is max
  t.attributes['b50']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)

  AddSubDict(t.attributes, 'b51')
  InsertYAML(t.attributes['b51'], 'models/obj/b51.yaml')
  #FIXME: the following values should be DEFAULT values
  #Gripper width before grab
  t.attributes['b51']['g_pre']= 0.08
  #Grab power (max effort):
  t.attributes['b51']['f_grab']= 50.0
  #Orientation to start pouring
  t.attributes['b51']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
  #Orientation where the flow is max
  t.attributes['b51']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)


  t.ExecuteMotion('attr',(0,))
