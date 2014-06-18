#!/usr/bin/python
import sys

PYTHON= 0
MATLAB= 1

file_name= ''
comment_style= PYTHON
out_delim= ' '
rostime_1st= True  #True: first column is assumed to be ROS Time (nano sec) and converted into sec
to_elapsed_1st= True  #True: first column is assumed to be time and the first value of every row is converted to elapsed time from the first row's time

file_name= sys.argv[1]


init= True
in_delim=''
comment_char= '#' if comment_style==PYTHON else '%'

fp= file(file_name)
while True:
  line= fp.readline()
  if not line: break
  if line[0]=='%' or line[0]=='#':
    d= [[len(line.split(' ')),' '],[len(line.split(',')),','],[len(line.split('\t')),'\t']]
    d.sort()
    in_delim= d[-1][1]
    #print comment_char+line[1:],
    print comment_char+out_delim.join(line[1:].split(in_delim)),
    continue
  if init:
    d= [[len(line.split(' ')),' '],[len(line.split(',')),','],[len(line.split('\t')),'\t']]
    d.sort()
    in_delim= d[-1][1]
    if to_elapsed_1st:
      rost_init= float(line.split(in_delim)[0])
    init= False
  values= line.split(in_delim)
  if rostime_1st or to_elapsed_1st:
    if rostime_1st and to_elapsed_1st:
      values[0]= str( (float(values[0])-rost_init)*1.0e-9 )
    elif rostime_1st:
      values[0]= str( float(values[0])*1.0e-9 )
    elif to_elapsed_1st:
      values[0]= str( float(values[0])-rost_init )
  print out_delim.join(values),

