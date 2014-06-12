#!/usr/bin/python
from core_tool import *
def Help():
  return '''List up the attributes or assign value to an element.
  Usage: attr
    List up the attributes
  Usage: attr OBJ_ID ELEM_ID VALUE
    Assign VALUE to [OBJ_ID][ELEM_ID]'''
def PrintDict(d,indent=0):
  for k,v in d.items():
    if type(v)==dict:
      print '  '*indent,'[',k,']=...'
      PrintDict(v,indent+1)
    else:
      print '  '*indent,'[',k,']=',v
def Run(t,args=[]):
  if len(args)==0:
    PrintDict(t.attributes)
  else:
    obj= args[0]
    elem= args[1]
    value= VecToStr(args[2:])
    if not obj in t.attributes:
      print 'No object named: ',obj
      return
    if elem in t.attributes[obj]:
      prev= t.attributes[obj][elem]
      t.attributes[obj][elem]= EstStrConvert(value)
      if type(prev)!=type(t.attributes[obj][elem]):
        print 'WARNING: type of [',obj,'][',elem,'] changed!'
        print '  Previous type: ',type(prev)
        print '  Previous value: ',prev
    else:
      t.attributes[obj][elem]= EstStrConvert(value)
    print 'Assigned: [',obj,'][',elem,']= ',t.attributes[obj][elem]
