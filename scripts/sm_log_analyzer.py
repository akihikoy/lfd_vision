#!/usr/bin/python
#Usage: sm_log_analyzer.py [STATE_MACHINE_LOG_FILES(e.g. pour*.dat)]
import sys
import numpy as np
import numpy.linalg as la

sm_id= '.sm'

file_names= sys.argv[1:]
file_names.sort()

table= {}  #id:[[entry_time,exit_time,result,file_names],[entry_time,exit_time,result,file_names],...]
states= []

def LineFitByLeastSq(data_x, data_f, f_reg=None):
  if f_reg==None:
    f_reg= 0.0
    if len(data_x)<=2:  f_reg= 0.1
  V= np.array(data_f)
  Theta= np.array([data_x,[1.0]*len(data_x)]).T
  w= la.inv(Theta.T.dot(Theta)+f_reg*np.eye(2)).dot(Theta.T).dot(V)
  return w

for file_name in file_names:
  print 'Analyzing',file_name
  fp= file(file_name)
  while True:
    line= fp.readline()
    if not line: break
    values= line.strip().split(' ')
    if len(values)==0:  continue
    if values[1]=='sm_entry':
      if sm_id not in table:  table[sm_id]= []
      table[sm_id].append([float(values[0]),0.0,'',file_name])
      if sm_id not in states:  states.append(sm_id)
    elif values[1]=='state_entry':
      state= values[2]  #state id
      if state not in table:  table[state]= []
      table[state].append([float(values[0]),0.0,'',file_name])
      if state not in states:  states.append(state)
    elif values[1]=='sm_exit':
      table[sm_id][-1][1]= float(values[0])  #time
      table[sm_id][-1][2]= values[4]  #result
      if table[sm_id][-1][1]-table[sm_id][-1][0]<0 or table[sm_id][-1][2]=='':
        print '###Error in',file_name,sm_id
    elif values[1]=='state_exit':
      state= values[2]  #state id
      table[state][-1][1]= float(values[0])  #time
      table[state][-1][2]= values[4]  #result
      if table[state][-1][1]-table[state][-1][0]<0 or table[state][-1][2]=='':
        print '###Error in',file_name,state
  fp.close()

#print table

failure_list= []  #[[file_name,state,failure_code],[...],...]
print '----'
print '#state average_duration num_of_executions num_of_failures failure_codes duration_info...'
sum_of_durations= 0.0
#for state,data in table.iteritems():
for state in states:
  data= table[state]
  #durations= [(d[1]-d[0])*1.0e-9 for d in data]
  durations= []
  durations_s= []
  success_idxs= []
  failures= []
  #failures+= filter(lambda x:x!='success', [d[2] for d in data])
  for d,i in zip(data,range(len(data))):
    if d[1]>0.0:
      durations.append((d[1]-d[0])*1.0e-9)
      if d[2]!='success':
        failures.append(d[2])
        failure_list.append([d[3],state,d[2]])
      else:
        success_idxs.append(i)
        durations_s.append(durations[-1])
    else:
      failures.append('failure.forcedquit')
      failure_list.append([d[3],state,'failure.forcedquit'])
  average_duration= sum(durations)/float(len(durations))
  if state!=sm_id:  sum_of_durations+= sum(durations)
  duration_s_w= LineFitByLeastSq(success_idxs, durations_s)
  failure_codes= ','.join(failures) if len(failures)>0 else 'N/A'
  print '%s\t%f\t%i\t%i\t%s\t%f %f [%s]' % (
        state,
        average_duration,
        len(data),  #len(durations),
        len(failures),
        failure_codes,
        duration_s_w[0],
        duration_s_w[1],
        ','.join(map(str,durations)) )
  with file('/tmp/duration_%s.dat'%state,'w') as fp:
    i_s= 0
    for i in range(len(durations)):
      if i in success_idxs:
        fp.write('%f %f %f %f\n'%(i, durations[i], duration_s_w[0]*float(i)+duration_s_w[1], durations_s[i_s]))
        i_s+= 1
      else:
        fp.write('%f %f %f\n'%(i, durations[i], duration_s_w[0]*float(i)+duration_s_w[1]))


print '----'
print 'sum_of_durations:',sum_of_durations

print '----'
failure_list.sort()
for file_name,state,failure_code in failure_list:
  print '%s\t%s\t%s' % (
        file_name,
        state,
        failure_code )

print '----'
print 'Plot /tmp/duration_*.dat with:'
print "bash -c 'for f in /tmp/duration_*;do recho $f; cat $f | qplot -x - u 1:3 w l - u 1:4 pt 6; done'"
