//-------------------------------------------------------------------------------------------
/*! \file    color_detector_node.cpp
    \brief   Color detector node.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jun.05, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/color_detector.h"
#include "lfd_vision/flow_finder.h"
#include "lfd_vision/mov_detector.h"
#include "lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include "lfd_vision/ColDetSensor.h"
#include "lfd_vision/ColDetViz.h"
#include "lfd_vision/ColDetVizPrimitive.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
//-------------------------------------------------------------------------------------------
namespace trick
{
const char *DefaultFileNames[]={
    "default_colors1.dat",
    "default_colors2.dat",
    "default_colors3.dat",
    "default_colors4.dat",
    "default_colors5.dat",
    "default_colors6.dat",
    "default_colors7.dat"};
const char *DefaultFlowFileName= "default_colors0.dat";
std::string ColorFilesBase= "";

int  CDIdx(0);
bool Running(true);
TMultipleColorDetector ColDetector;
TMultipleColorDetector FlowColDetector;  // Color detector for flow
TFlowFinder FlowFinder;
TMovingObjectDetector MovDetector;

int NumDetectors(1);
int NRotate90(0);
TEasyVideoOut VideoOut;
int VizMode(2);  // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only, 5: 0.5*camera + flow + flow-mask
int FlowMaskMode(2);  // 0: none, 1: removing moving objects, 2: mask with colors

std::vector<lfd_vision::ColDetVizPrimitive> VizObjs;  // External visualization requests.
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

/*
  Shift+Left click: add a color
  Shift+Right click: reset colors
  Left double click: reset amount
  Right click: pause/resume
*/
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(flags==cv::EVENT_FLAG_SHIFTKEY)
  {
    if(CDIdx>=0)
      ColDetector.CameraWindowMouseCallback(CDIdx, event, x, y, flags);
    else
      FlowColDetector.CameraWindowMouseCallback(0, event, x, y, flags);
  }
  else
  {
    if(CDIdx>=0)
      ColDetector.MaskWindowMouseCallback(CDIdx, event, x, y, flags);
    else
      FlowColDetector.MaskWindowMouseCallback(0, event, x, y, flags);

    if(event == cv::EVENT_RBUTTONDOWN)
    {
      Running=!Running;
      std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
    }
  }
}
//-------------------------------------------------------------------------------------------

// return if continue
bool HandleKeyEvent()
{
  // keyboard interface:
  char c(cv::waitKey(1));
  if(c=='\x1b'||c=='q') return false;
  if(c=='[') {--NRotate90; std::cerr<<"NRotate90= "<<NRotate90<<std::endl;}
  if(c==']') {++NRotate90; std::cerr<<"NRotate90= "<<NRotate90<<std::endl;}
  else if(c=='r')
  {
    ColDetector.Reset();
    FlowColDetector.Reset();
  }
  else if(c=='l')
  {
    if(CDIdx>=0)
      ColDetector.LoadColors(CDIdx, ColorFilesBase+DefaultFileNames[CDIdx]);
    else
      FlowColDetector.LoadColors(0, ColorFilesBase+DefaultFlowFileName);
  }
  else if(c=='s')
  {
    if(CDIdx>=0)
      ColDetector.SaveColors(CDIdx, ColorFilesBase+DefaultFileNames[CDIdx]);
    else
      FlowColDetector.SaveColors(0, ColorFilesBase+DefaultFlowFileName);
  }
  else if((c>='1' && c<='7') || c=='0')
  {
    int old_cd_idx(CDIdx);
    switch(c)
    {
    case '1':  CDIdx= 0; break;
    case '2':  CDIdx= 1; break;
    case '3':  CDIdx= 2; break;
    case '4':  CDIdx= 3; break;
    case '5':  CDIdx= 4; break;
    case '6':  CDIdx= 5; break;
    case '7':  CDIdx= 6; break;
    case '0':  CDIdx= -1; break;
    }
    if(CDIdx>=NumDetectors)
      CDIdx= old_cd_idx;
    else
      std::cerr<<"###Selected: "<<(CDIdx+1)<<std::endl;
  }
  else if(c=='W')
  {
    VideoOut.Switch();
  }
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  else if(c=='m' || c=='M')
  {
    if(c=='m')  ++VizMode;  else  --VizMode;
    if(VizMode>5)  VizMode= 0;
    if(VizMode<0)  VizMode= 5;
    std::cerr<<"VizMode: "<<VizMode<<std::endl;
  }
  else if(c=='f')
  {
    ++FlowMaskMode;
    if(FlowMaskMode>2)  FlowMaskMode= 0;
    std::cerr<<"FlowMaskMode: "<<FlowMaskMode<<std::endl;
  }

  return true;
}
//-------------------------------------------------------------------------------------------

void DrawExternalViz(cv::Mat &disp_img)
{
  for(std::vector<lfd_vision::ColDetVizPrimitive>::const_iterator itr(VizObjs.begin()),itr_end(VizObjs.end());
      itr!=itr_end; ++itr)
  {
    cv::Scalar col= CV_RGB(itr->color.r,itr->color.g,itr->color.b);
    const double &lw= itr->line_width;
    switch(itr->type)
    {
    case lfd_vision::ColDetVizPrimitive::LINE :
      cv::line(disp_img, cv::Point2d(itr->param[0],itr->param[1]), cv::Point2d(itr->param[2],itr->param[3]), col, lw);
      break;
    case lfd_vision::ColDetVizPrimitive::CIRCLE :
      cv::circle(disp_img, cv::Point2d(itr->param[0],itr->param[1]), itr->param[2], col, lw);
      break;
    case lfd_vision::ColDetVizPrimitive::RECTANGLE :
      cv::rectangle(disp_img, cv::Rect(itr->param[0],itr->param[1],itr->param[2],itr->param[3]), col, lw);
      break;
    default:
      std::cerr<<"Unknown type:"<<itr->type<<std::endl;
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------

void ColDetVizCallback(const lfd_vision::ColDetViz &msg)
{
  VizObjs= msg.objects;
}
//-------------------------------------------------------------------------------------------

bool ResetAmount(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resetting base amount..."<<std::endl;
  ColDetector.Reset();
  return true;
}
//-------------------------------------------------------------------------------------------

bool Pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Paused..."<<std::endl;
  Running= false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resumed..."<<std::endl;
  Running= true;
  return true;
}
//-------------------------------------------------------------------------------------------

void RemoveMovingObjectFlow(TFlowFinder &flow_finder, const TMovingObjectDetector &mov_detector)
{
  std::list<TFlowElement> &flow(flow_finder.RefFlowElements());
  for(std::list<TFlowElement>::iterator itr(flow.begin()),itr_end(flow.end());
      itr!=itr_end; /*not increment itr*/)
  {
    if(mov_detector.IsMoving(cv::Point2f(itr->X,itr->Y)))
      itr= flow.erase(itr);
    else
      ++itr;
  }
}
//-------------------------------------------------------------------------------------------

void MaskFlow(TFlowFinder &flow_finder, const cv::Mat &mask_img)
{
  std::list<TFlowElement> &flow(flow_finder.RefFlowElements());
  for(std::list<TFlowElement>::iterator itr(flow.begin()),itr_end(flow.end());
      itr!=itr_end; /*not increment itr*/)
  {
    if(mask_img.at<unsigned char>(itr->Y,itr->X))
      ++itr;
    else
      itr= flow.erase(itr);
  }
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle node("~");
  int camera(0);
  int mode(2);  // Ratio computation mode (1 or 2, 1:deprecated).
  int pub_mode(0);  // 0: original, 1: only ColDetSensor message, 2: both
  double block_area_min(10.0);
  std::string vout_base("/tmp/vout");
  node.param("camera",camera,0);
  node.param("mode",mode,mode);
  node.param("pub_mode",pub_mode,pub_mode);
  node.param("viz_mode",VizMode,VizMode);
  node.param("flow_mask_mode",FlowMaskMode,FlowMaskMode);
  node.param("num_detectors",NumDetectors,NumDetectors);
  node.param("block_area_min",block_area_min,block_area_min);
  node.param("color_files_base",ColorFilesBase,std::string(""));
  node.param("rotate90n",NRotate90,NRotate90);
  node.param("vout_base",vout_base,vout_base);

  cv::VideoCapture cap(camera); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
  {
    std::cerr<<"Cannot open: "<<camera<<std::endl;
    return -1;
  }
  std::cerr<<"Camera opened"<<std::endl;

  VideoOut.SetfilePrefix(vout_base);

  ColDetector.Setup(NumDetectors);
  for(int i(0); i<NumDetectors; ++i)
    ColDetector.LoadColors(i, ColorFilesBase+DefaultFileNames[i]);
  ColDetector.SetBlockAreaMin(block_area_min);
  FlowColDetector.Setup(1);
  FlowColDetector.LoadColors(0, ColorFilesBase+DefaultFlowFileName);
  CDIdx= 0;

  FlowFinder.SetOptFlowWinSize(cv::Size(3,3));
  FlowFinder.SetOptFlowSpdThreshold(5.0);
  FlowFinder.SetErodeDilate(1);
  FlowFinder.SetAmountRange(/*min=*/3.0, /*max=*/3000.0);
  FlowFinder.SetSpeedRange(/*min=*/3.0, /*max=*/-1.0);

  // Moving object detector using LK optical flow.
  MovDetector.SetMaxFeatCount(1200);
  MovDetector.SetResetCount(10);
  MovDetector.SetMinMovingFlow(1.0);
  MovDetector.SetFlowRangeGain(20.0);
  MovDetector.SetMaxFlowRange(70.0);

  std::vector<ros::Publisher> ratio_pubs(NumDetectors);
  std::vector<ros::Publisher> mxy_pubs(NumDetectors);
  ros::Publisher flow_pub;
  ros::Publisher sensor_pub;

  // pub_mode: // 0: original, 1: only ColDetSensor message, 2: both
  if(pub_mode==0 || pub_mode==2)
  {
    ratio_pubs[0]= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);
    for(int i(1); i<NumDetectors; ++i)
      ratio_pubs[i]= node.advertise<std_msgs::Float64>(ToString("/color_occupied_ratio", i+1), 1);
    mxy_pubs[0]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy", 1);
    for(int i(1); i<NumDetectors; ++i)
      mxy_pubs[i]= node.advertise<std_msgs::Int32MultiArray>(ToString("/color_middle_xy", i+1), 1);
    flow_pub= node.advertise<std_msgs::Float64MultiArray>("/flow_speed_angle", 1);
  }
  if(pub_mode==1 || pub_mode==2)
  {
    sensor_pub= node.advertise<lfd_vision::ColDetSensor>("sensor", 1);
  }

  ros::Subscriber sub_viz= node.subscribe("viz", 1, &ColDetVizCallback);

  ros::ServiceServer srv_reset= node.advertiseService("reset", &ResetAmount);
  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);

  cv::namedWindow("color_detector",1);
  cv::Mat frame, disp_img, flow_mask_img;
  ColDetector.SetCameraWindow(frame);
  FlowColDetector.SetCameraWindow(frame);

  cv::setMouseCallback("color_detector", OnMouse);

  int show_fps(0);

  // ros::Rate loop_rate(5);  // 5 Hz
  for(int f(0);ros::ok();++f)
  {
    if(Running)
    {
      cap >> frame; // get a new frame from camera
      if(NRotate90!=0)  Rotate90N(frame,frame,NRotate90);

      // Visualization setup
      // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only, 5: 0.5*camera + flow + flow-mask
      if(VizMode==0 || VizMode==1)
      {
        frame.copyTo(disp_img);
      }
      else if(VizMode==2)
      {
        frame.copyTo(disp_img);
        disp_img*= 0.5;
      }
      else if(VizMode==3)
      {
        frame.copyTo(disp_img);
        disp_img*= 0.25;
      }
      else if(VizMode==4)
      {
        frame.copyTo(disp_img);  // TODO: make this efficient (not need to copy)
        disp_img.setTo(cv::Scalar(0,0,0));
      }
      else if(VizMode==5)
      {
        frame.copyTo(disp_img);
        disp_img*= 0.5;
      }

      // Color detection from image
      ColDetector.Detect(frame, mode, /*verbose=*/false);
      if(VizMode!=0 && VizMode!=5)
      {
        ColDetector.Draw(disp_img);
        if(CDIdx>=0)
          cv::rectangle(disp_img, ColDetector.Bound(CDIdx), CV_RGB(0,255,0), 2);
      }

      // For flow mask:
      // 0: none, 1: removing moving objects, 2: mask with colors
      if(FlowMaskMode==1)
      {
        // Moving objects detection from image
        MovDetector.Step(frame);
        if(VizMode==5)  MovDetector.Draw(disp_img);
      }
      else if(FlowMaskMode==2)
      {
        // Flow color detection from image
        flow_mask_img= FlowColDetector.Detector(0).Detect(frame);
        if(VizMode==5)  frame.copyTo(disp_img, flow_mask_img);
      }

      // Flow detection from image
      FlowFinder.Update(frame);
      // Remove detected flows of moving objects (e.g. robot hand)
      if(FlowMaskMode==1)  RemoveMovingObjectFlow(FlowFinder, MovDetector);
      else if(FlowMaskMode==2)  MaskFlow(FlowFinder, flow_mask_img);
      if(VizMode!=0)
        FlowFinder.DrawFlow(disp_img, CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/3);

      // Compute average flow
      cv::Vec2d avr_xy(0.0,0.0), avr_vel(0.0,0.0), avr_spddir(0.0,0.0);
      CalcFlowAverage(FlowFinder, avr_xy, avr_vel, avr_spddir);

      if(VizMode!=0 && avr_spddir[0]>20.0)
        cv::line(disp_img, cv::Point2d(avr_xy-0.5*avr_vel), cv::Point2d(avr_xy+0.5*avr_vel), CV_RGB(255,128,0), 5);

      if(VizMode!=0)
        DrawExternalViz(disp_img);

      // Send ROS topics
      // pub_mode: // 0: original, 1: only ColDetSensor message, 2: both
      if(pub_mode==0 || pub_mode==2)
      {
        std_msgs::Float64  ratio_msg;
        std_msgs::Int32MultiArray mxy_msg;
        for(int i(0),i_end(ColDetector.Size()); i<i_end; ++i)
        {
          ratio_msg.data= ColDetector.Ratio(i);
          ratio_pubs[i].publish(ratio_msg);

          mxy_msg.data.resize(2);
          mxy_msg.data[0]= ColDetector.CenterX(i);
          mxy_msg.data[1]= ColDetector.CenterY(i);
          mxy_pubs[i].publish(mxy_msg);
        }

        std_msgs::Float64MultiArray flow_msg;
        flow_msg.data.resize(2);
        flow_msg.data[0]= avr_spddir[0];
        flow_msg.data[1]= avr_spddir[1];
        flow_pub.publish(flow_msg);
      }
      if(pub_mode==1 || pub_mode==2)
      {
        lfd_vision::ColDetSensor  sensor_msg;

        sensor_msg.num_cols= ColDetector.Size();
        sensor_msg.col_filled_ratio= ColDetector.DataRatio();
        sensor_msg.col_center_xy= ColDetector.DataCenterXY();
        sensor_msg.col_area= ColDetector.DataArea();
        sensor_msg.col_bound= ColDetector.DataBound();

        sensor_msg.nums_blocks= ColDetector.NumsBlocks();
        sensor_msg.blocks_area.resize(ColDetector.BlocksArea().size());
        std::copy(ColDetector.BlocksArea().begin(),ColDetector.BlocksArea().end(), sensor_msg.blocks_area.begin());
        sensor_msg.blocks_center_xy.resize(ColDetector.BlocksCenterXY().size());
        std::copy(ColDetector.BlocksCenterXY().begin(),ColDetector.BlocksCenterXY().end(), sensor_msg.blocks_center_xy.begin());

        const std::list<TFlowElement> &flow(FlowFinder.FlowElements());
        sensor_msg.num_flows= flow.size();
        sensor_msg.flows_xy     .resize(2*flow.size());
        sensor_msg.flows_vxy    .resize(2*flow.size());
        sensor_msg.flows_spddir .resize(2*flow.size());
        sensor_msg.flows_amount .resize(flow.size());
        int i(0);
        for(std::list<TFlowElement>::const_iterator itr(flow.begin()),itr_end(flow.end());
            itr!=itr_end; ++itr,++i)
        {
          sensor_msg.flows_xy[2*i+0]= itr->X;
          sensor_msg.flows_xy[2*i+1]= itr->Y;
          sensor_msg.flows_vxy[2*i+0]= itr->VX;
          sensor_msg.flows_vxy[2*i+1]= itr->VY;
          sensor_msg.flows_spddir[2*i+0]= itr->Speed;
          sensor_msg.flows_spddir[2*i+1]= itr->Angle;
          sensor_msg.flows_amount[i]= itr->Amount;
        }

        sensor_msg.flow_avr_xy.resize(2);
        sensor_msg.flow_avr_vxy.resize(2);
        sensor_msg.flow_avr_spddir.resize(2);
        sensor_msg.flow_avr_xy[0]= avr_xy[0];
        sensor_msg.flow_avr_xy[1]= avr_xy[1];
        sensor_msg.flow_avr_vxy[0]= avr_vel[0];
        sensor_msg.flow_avr_vxy[1]= avr_vel[1];
        sensor_msg.flow_avr_spddir[0]= avr_spddir[0];
        sensor_msg.flow_avr_spddir[1]= avr_spddir[1];

        sensor_pub.publish(sensor_msg);
      }

      std::cerr<<"ratio:";
      for(int i(0); i<ColDetector.Size(); ++i)
        std::cerr<<" "<<ColDetector.Ratio(i);
      std::cerr<<"\t spd,angle: "<<avr_spddir<<std::endl;

      VideoOut.Step(disp_img);
      VideoOut.VizRec(disp_img);
      cv::imshow("color_detector", disp_img);

      if(show_fps==0)  {std::cerr<<"FPS: "<<VideoOut.FPS()<<std::endl;  show_fps=VideoOut.FPS()*4;}
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }

    if(!HandleKeyEvent())  break;

    ros::spinOnce();
  }


  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
