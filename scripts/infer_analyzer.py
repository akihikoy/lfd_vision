#!/usr/bin/python
#Usage: infer_analyzer.py DATABASE.yaml [START_INDEX(0)]
import sys
import numpy as np
import numpy.linalg as la

file_name= sys.argv[1]
start_idx= sys.argv[2] if len(sys.argv)>2 else 0

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

def LineFitByLeastSq(data_x, data_f, f_reg=0.0):
  V= np.array(data_f)
  Theta= np.array([data_x,[1.0]*len(data_x)]).T
  w= la.inv(Theta.T.dot(Theta)+f_reg*np.eye(2)).dot(Theta.T).dot(V)
  return w


print 'Loading database',file_name,'...'
database= LoadYAML(file_name)
print 'Loaded database (size:%i) from: %r' % (len(database), file_name)

table= {}  #infer_type:[[score,infer_time],[score,infer_time],...]
infer_types= []

for situation, inferred_data, assessment in database[start_idx:]:
  infer_type= situation['infer_type']
  score= situation['infer_info']['score']
  infer_time= assessment['infer_time']

  if infer_type not in table:  table[infer_type]= []
  table[infer_type].append([score,infer_time])

  if infer_type not in infer_types:  infer_types.append(infer_type)

for infer_type in infer_types:
  data= table[infer_type]
  scores= [d[0] for d in data]
  infer_times= [d[1] for d in data]

  score_avr= sum(scores)/float(len(scores))
  infer_time_avr= sum(infer_times)/float(len(infer_times))

  score_w= LineFitByLeastSq(range(len(data)), scores)
  infer_time_w= LineFitByLeastSq(range(len(data)), infer_times)

  print '%s %i\t%f %f %f\t%f %f %f' % (
        infer_type,
        len(data),
        score_avr, score_w[0], score_w[1],
        infer_time_avr, infer_time_w[0], infer_time_w[1] )

  with file('/tmp/score_%s.dat'%infer_type,'w') as fp:
    i_s= 0
    for i in range(len(scores)):
      fp.write('%f %f %f\n'%(i, scores[i], score_w[0]*float(i)+score_w[1]))

  with file('/tmp/infer_time_%s.dat'%infer_type,'w') as fp:
    i_s= 0
    for i in range(len(infer_times)):
      fp.write('%f %f %f\n'%(i, infer_times[i], infer_time_w[0]*float(i)+infer_time_w[1]))

print '----'
print 'Plot /tmp/score_*.dat with:'
print "bash -c 'for f in /tmp/score_*;do recho $f; cat $f | qplot -x - u 1:3 w l - u 1:2 pt 6; done'"
print 'Plot /tmp/infer_time_*.dat with:'
print "bash -c 'for f in /tmp/infer_time_*;do recho $f; cat $f | qplot -x - u 1:3 w l - u 1:2 pt 6; done'"
