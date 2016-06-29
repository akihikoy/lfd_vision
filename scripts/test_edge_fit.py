#!/usr/bin/python
#\file    test_edge_fit.py
#\brief   certain python script
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.28, 2016
import roslib; roslib.load_manifest('lfd_vision')
import rospy
import lfd_vision.srv
import math

if __name__=='__main__':
  rospy.init_node('test_edge_fit')
  fit_edge= rospy.ServiceProxy('/usb_stereo/fit_edge', lfd_vision.srv.FitEdge, persistent=False)

  N= 20
  #rad= 0.041; height= 0.06
  #rad=0.05; height= 0.14
  rad=0.016; height= 0.10
  dth= 2.0*math.pi/float(N)
  req= lfd_vision.srv.FitEdgeRequest()
  req.XMin= [-0.1,-0.1,-0.1, -0.2,-0.2,-0.2]
  req.XMax= [+0.1,+0.1,+0.1, +0.2,+0.2,+0.2]
  req.Sig0= [0.02,0.02,0.02, 0.01,0.01,0.01]
  req.LPoints3d= sum([[rad*math.cos(i*dth), height, rad*math.sin(i*dth)] for i in range(N)],[])
  req.pose0= [0.0720665, 0.0529803, 0.292865, 0.1076, -0.0118153, -0.0105634, 0.994068]

  while not rospy.is_shutdown():
    res= fit_edge(req)
    print 'res=',res
    req.pose0= res.pose
