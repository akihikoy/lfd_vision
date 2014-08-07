#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test.
  Usage: test [FLOAT_VALUE]'''
def Run(t,args=()):
  print t
  print 'Hello world!'
  #t.MoveArmsToSide()
  if len(args)!=1:
    print Help()
  else:
    print 'args[0]=',float(args[0])
