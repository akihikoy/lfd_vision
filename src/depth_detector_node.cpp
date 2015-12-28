//-------------------------------------------------------------------------------------------
/*! \file    depth_detector_node.cpp
    \brief   Depth detector node (TEST, copied from color_detector_node.cpp).
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jun.05, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/flow_finder.h"
#include "lfd_vision/vision_util.h"
#include "lfd_vision/sentis_m100.h"
//-------------------------------------------------------------------------------------------
#include "lfd_vision/Int32Array.h"
#include "lfd_vision/IndexedBoundingBox.h"
#include "lfd_vision/SetBoundingBox.h"
#include "lfd_vision/SetBBEquation.h"
#include "lfd_vision/ReadRegister.h"
#include "lfd_vision/WriteRegister.h"
#include "lfd_vision/SetFrameRate.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
//-------------------------------------------------------------------------------------------
namespace trick
{

class TSentisM100Node : public TSentisM100
{
public:
  TSentisM100Node(ros::NodeHandle &node)
      :
        TSentisM100(),
        node_(node)
    {
      srv_write_register_= node_.advertiseService("write_register", &TSentisM100Node::SrvWriteRegister, this);
      srv_read_register_= node_.advertiseService("read_register", &TSentisM100Node::SrvReadRegister, this);
      srv_set_frame_rate_= node_.advertiseService("set_frame_rate", &TSentisM100Node::SrvSetFrameRate, this);
    }

  bool SrvWriteRegister(lfd_vision::WriteRegister::Request &req, lfd_vision::WriteRegister::Response &res)
    {
      res.success= WriteRegister(req.address, req.value);
      return true;
    }

  bool SrvReadRegister(lfd_vision::ReadRegister::Request &req, lfd_vision::ReadRegister::Response &res)
    {
      res.value= ReadRegister(req.address);
      res.success= IsNoError("");
      return true;
    }

  bool SrvSetFrameRate(lfd_vision::SetFrameRate::Request &req, lfd_vision::SetFrameRate::Response &res)
    {
      res.success= SetFrameRate(req.frame_rate);
      return true;
    }

private:
  ros::NodeHandle     &node_;
  ros::ServiceServer  srv_write_register_;
  ros::ServiceServer  srv_read_register_;
  ros::ServiceServer  srv_set_frame_rate_;

};
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

static bool running(true);

void OnMouseCamera(int event, int x, int y, int flags, void *)
{
}
//-------------------------------------------------------------------------------------------

void OnMouseMask(int event, int x, int y, int flags, void *)
{
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    running=!running;
    std::cerr<<(running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "depth_detector_node");
  ros::NodeHandle node("~");
  int rotate90n(0);
  std::string vout_base;
  node.param("rotate90n",rotate90n,0);
  node.param("vout_base",vout_base,std::string("/tmp/vout_"));

  int init_fps, tcp_port, udp_port;
  std::string tcp_ip, udp_ip;
  node.param("init_fps",init_fps,1);
  node.param("tcp_ip",tcp_ip,std::string("192.168.0.10"));
  node.param("udp_ip",udp_ip,std::string("224.0.0.1"));
  node.param("tcp_port",tcp_port,10001);
  node.param("udp_port",udp_port,10002);

  double dist_thresh(0.5);
  node.param("dist_thresh",dist_thresh,0.5);

  TSentisM100Node tof_sensor(node);
  tof_sensor.Init(init_fps, /*data_format=*/DEPTH_AMP_DATA, tcp_ip.c_str(), udp_ip.c_str(), tcp_port, udp_port);
  // tof_sensor.PrintRegisters(0);
  // tof_sensor.PrintRegisters(1);
  // tof_sensor.SetFrameRate(40);
  int sentis_fps(1);


  TFlowFinder flow_finder;
  flow_finder.SetOptFlowWinSize(cv::Size(3,3));
  flow_finder.SetOptFlowSpdThreshold(3.0);
  flow_finder.SetErodeDilate(1);
  flow_finder.SetAmountRange(/*min=*/-1.0, /*max=*/-1.0);
  flow_finder.SetSpeedRange(/*min=*/-1.0, /*max=*/-1.0);
  // cv::BackgroundSubtractorMOG2 bkg_sbtr(/*int history=*/5, /*double varThreshold=*/5.0, /*bool detectShadows=*/true);


  ros::Publisher flow_pub;
  flow_pub= node.advertise<std_msgs::Float64MultiArray>("/flow_speed_angle", 1);

  cv::namedWindow("camera",1);
  cv::namedWindow("mask_img",1);
  cv::Mat frame, frame_tmp, disp_img;

  cv::setMouseCallback("camera", OnMouseCamera, NULL);
  cv::setMouseCallback("mask_img", OnMouseMask, NULL);

  cv::VideoWriter vout_camera, vout_mask;
  double time_prev= GetCurrentTime(), fps(10.0), fps_alpha(0.05);
  int    show_fps(0);

  ros::Rate loop_rate(40);  // 40 Hz
  for(int f(0);ros::ok();++f)
  {
    if(running)
    {
      if(!tof_sensor.GetDataAsCVMat(&frame,&frame_tmp))  continue;
      // if(!tof_sensor.GetDataAsCVMat(&frame_tmp,&frame))  continue;
      if(rotate90n!=0)  Rotate90N(frame,frame,rotate90n);

      cv::threshold(frame, disp_img, /*thresh=*/dist_thresh, /*maxval=*/255.0, cv::THRESH_TOZERO_INV);

      frame*= 255.0;
      frame.convertTo(frame,CV_8UC1);
      cv::cvtColor(frame,frame,CV_GRAY2RGB);
      cv::imshow("camera", frame);

      disp_img= 255.0*(1.0-disp_img/dist_thresh);
      disp_img.convertTo(disp_img,CV_8UC1);
      cv::threshold(disp_img, disp_img, /*thresh=*/254, /*maxval=*/0, cv::THRESH_TOZERO_INV);
      cv::threshold(disp_img, disp_img, /*thresh=*/200, /*maxval=*/0, cv::THRESH_TOZERO);
      cv::dilate(disp_img,disp_img,cv::Mat(),cv::Point(-1,-1), 2);
      cv::erode(disp_img,disp_img,cv::Mat(),cv::Point(-1,-1), 1);
      // cv::imshow("mask_img", disp_img);

      // flow_finder.Update(frame);
      // flow_finder.DrawFlow(disp_img, CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/3);
      // // for(int i(0);i<flow_finder.FlowElements().size();++i) std::cerr<<f<<" "<<flow_finder.FlowElements()[i]<<std::endl;

      flow_finder.UpdateProc2_ContourAnalysis(disp_img);
      cv::cvtColor(disp_img,disp_img,CV_GRAY2RGB);
      flow_finder.DrawFlow(disp_img, CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/3);
      // for(int i(0);i<flow_finder.FlowElements().size();++i) std::cerr<<f<<" "<<flow_finder.FlowElements()[i]<<std::endl;


      // TEST: Compute average speed, angle
      double sum_amt(0.0);
      cv::Vec2d avr_xy(0.0,0.0), avr_pvel(0.0,0.0), avr_vel(0.0,0.0);
      for(std::list<TFlowElement>::const_iterator itr(flow_finder.FlowElements().begin()),itr_end(flow_finder.FlowElements().end());
          itr!=itr_end; ++itr)
      {
        double amt= itr->Amount;
        sum_amt+= amt;
        avr_xy[0]+= amt*itr->X;
        avr_xy[1]+= amt*itr->Y;
        avr_pvel[0]+= amt*itr->Speed;
        avr_pvel[1]+= amt*std::fabs(itr->Angle);
      }
      if(sum_amt>1.0e-6)
      {
        avr_xy/= sum_amt;
        avr_pvel/= sum_amt;
      }
      else
      {
        avr_xy= cv::Vec2d(0.0,0.0);
        avr_pvel= cv::Vec2d(0.0,0.0);
      }
      avr_vel= cv::Point2d(avr_pvel[0]*std::cos(avr_pvel[1]), avr_pvel[0]*std::sin(avr_pvel[1]));
      const double len(2.0);
      if(avr_pvel[0]>3.0)
        cv::line(disp_img, cv::Point2d(avr_xy-0.5*len*avr_vel), cv::Point2d(avr_xy+0.5*len*avr_vel), CV_RGB(255,128,0), 3);
      // std::cerr<<"spd,angle: "<<avr_pvel<<std::endl;
      std_msgs::Float64MultiArray flow_msg;
      flow_msg.data.resize(2);
      flow_msg.data[0]= avr_pvel[0];
      flow_msg.data[1]= avr_pvel[1];
      flow_pub.publish(flow_msg);


      cv::imshow("mask_img", disp_img);

      if(vout_camera.isOpened())  vout_camera<<frame;
      if(vout_mask.isOpened())    vout_mask<<disp_img;

      // get fps
      fps= fps_alpha*(1.0/(GetCurrentTime()-time_prev)) + (1.0-fps_alpha)*fps;
      time_prev= GetCurrentTime();
      if(show_fps==0)  {std::cerr<<"fps: "<<fps<<std::endl;  show_fps=20;}
      --show_fps;

    }  // running
    else
    {
      usleep(200*1000);
      // unsigned short fps(1);
      // if(tof_sensor.GetFrameRate(fps))
      // {
        // sentis_fps...
      // }
      // tof_sensor.Sleep();
    }

    // keyboard interface:
    int c(cv::waitKey(1));
    if(c=='\x1b'||c=='q') break;
    else if(c=='w')
    {
      if(vout_camera.isOpened() || vout_mask.isOpened())
      {
        vout_camera.release();
        vout_mask.release();
        std::cerr<<"###Finished: video output"<<std::endl;
      }
      else
      {
        int i(0);
        std::string file_name_camera, file_name_mask;
        do
        {
          std::stringstream ss1;
          ss1<<vout_base<<"camera"<<i<<".avi";
          file_name_camera= ss1.str();
          std::stringstream ss2;
          ss2<<vout_base<<"mask"<<i<<".avi";
          file_name_mask= ss2.str();
          ++i;
        } while(FileExists(file_name_camera) || FileExists(file_name_mask));
        OpenVideoOut(vout_camera, file_name_camera.c_str(), fps, cv::Size(frame.cols,frame.rows));
        OpenVideoOut(vout_mask, file_name_mask.c_str(), fps, cv::Size(disp_img.cols,disp_img.rows));
      }
    }
    else if(c==' ')
    {
      running=!running;
      std::cerr<<(running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  tof_sensor.Sleep();

  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
