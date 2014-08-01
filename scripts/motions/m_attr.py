#!/usr/bin/python
from core_tool import *
def Help():
  return '''List up the attributes or assign value to an element.
  Usage: attr [MAX_LEVEL]
    List up the attributes
    MAX_LEVEL: Maximum level of printed attribute (default=-1)
  Usage: attr OBJ_ID, ELEM_ID, VALUE
    Assign VALUE to [OBJ_ID][ELEM_ID]
  Example:
    attr 'c1', 'x', [0,0,0, 0,0,0,1]  '''
def Run(t,args=()):
  if len(args)==0:
    PrintDict(t.attributes)
  elif len(args)==1:
    PrintDict(t.attributes,args[0])
  else:
    obj= args[0]
    elem= args[1]
    if len(args)>2:
      value= VecToStr(args[2:])
    else:
      value= args[2]
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
