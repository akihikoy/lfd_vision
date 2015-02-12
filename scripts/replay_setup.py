#!/usr/bin/python
#Usage: replay_setup.py CONDITION.yaml(e.g. pourCond20150117-014218.yaml) DATABASE.yaml
import sys
import numpy as np
import numpy.linalg as la

file_name_cond= sys.argv[1]
file_name_db= sys.argv[2]

#Speedup YAML using CLoader/CDumper
from yaml import load as yamlload
from yaml import dump as yamldump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper

#Load a YAML and return a dictionary
def LoadYAML(file_name):
  return yamlload(file(file_name).read(), Loader=Loader)


print 'Loading condition',file_name_cond,'...'
condition= LoadYAML(file_name_cond)

print 'Loading database',file_name_db,'...'
database= LoadYAML(file_name_db)
print 'Loaded database (size:%i) from: %r' % (len(database), file_name_db)

rcv= condition['receiver']
src= condition['source']
print 'FOR THE SAME SETUP:'
print 'objs2'
print 'init0'
print 'initr0'
print 'c t.MoveToJointAngles(%r,dt=3.0,arm=LEFT)'%condition['init']['l_q']
print "attr 'set','%s','x',%r"%(rcv,condition['init']['receiver_x'])
print "attr 'set','%s','x',%r"%(src,condition['init']['source_x'])
print '----'
print 'TO COMPLETE REPLAY:'
if 'lw_x_pour_e' in condition:  print "attr 'set',CURR,'lw_x_pour_e',%r"%(condition['lw_x_pour_e'])
if 'x_pour_l0' in condition:  print "attr 'set',CURR,'x_pour_l0',%r"%(condition['x_pour_l0'])

for didx in condition['new_in_database']:
  situation, inferred_data, assessment= database[didx]
  for inferred in inferred_data:
    print "attr 'set','%s',%r"%("','".join(inferred[0]),inferred[1])

