#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of using other script (shorter way).
  Usage: test3'''
def Run(t,args=[]):
  #t.LoadMotion('test').Run(t,[3.14*2])
  t.ExecuteMotion('test',[3.14*2])
