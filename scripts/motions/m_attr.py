#!/usr/bin/python
from core_tool import *
def Help():
  return '''Print attributes.
  Usage: attr'''
def PrintDict(d,indent=0):
  for k,v in d.items():
    if type(v)==dict:
      print '  '*indent,'[',k,']=...'
      PrintDict(v,indent+1)
    else:
      print '  '*indent,'[',k,']=',v
def Run(t,args=[]):
  PrintDict(t.attributes)
