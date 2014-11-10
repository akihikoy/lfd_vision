//-------------------------------------------------------------------------------------------
/*! \file    color_detector_node.cpp
    \brief   Color detector node.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jun.05, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/color_detector.h"
#include "pr2_lfd_vision/flow_finder.h"
#include "pr2_lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
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
const char *DefaultFileNames[]={
    "default_colors1.dat",
    "default_colors2.dat",
    "default_colors3.dat",
    "default_colors4.dat",
    "default_colors5.dat",
    "default_colors6.dat",
    "default_colors7.dat"};
std::string ColorFilesBase= "";
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

static int  cd_idx(0);
static bool running(true);

void OnMouseCamera(int event, int x, int y, int flags, void *p_col_detector)
{
  TMultipleColorDetector *col_detector(reinterpret_cast<TMultipleColorDetector*>(p_col_detector));
  col_detector->CameraWindowMouseCallback(cd_idx, event, x, y, flags);
}
//-------------------------------------------------------------------------------------------

void OnMouseMask(int event, int x, int y, int flags, void *p_col_detector)
{
  TMultipleColorDetector *col_detector(reinterpret_cast<TMultipleColorDetector*>(p_col_detector));
  col_detector->MaskWindowMouseCallback(cd_idx, event, x, y, flags);

  if(event == cv::EVENT_RBUTTONDOWN)
  {
    running=!running;
    std::cerr<<(running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle node("~");
  int camera(0);
  int mode(1);
  int num_detectors(1);
  int rotate90n(0);
  std::string vout_base;
  node.param("camera",camera,0);
  node.param("mode",mode,2);
  node.param("num_detectors",num_detectors,1);
  node.param("color_files_base",ColorFilesBase,std::string(""));
  node.param("rotate90n",rotate90n,0);
  node.param("vout_base",vout_base,std::string("/tmp/vout_"));

  TMultipleColorDetector col_detector;

  cv::VideoCapture cap(camera); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
  {
    std::cerr<<"Cannot open: "<<camera<<std::endl;
    return -1;
  }
  std::cerr<<"Camera opened"<<std::endl;

  col_detector.Setup(num_detectors);
  for(int i(0); i<num_detectors; ++i)
    col_detector.LoadColors(i, ColorFilesBase+DefaultFileNames[i]);
  cd_idx= 0;

  TFlowFinder flow_finder;
  flow_finder.SetOptFlowWinSize(cv::Size(3,3));
  flow_finder.SetOptFlowSpdThreshold(5.0);
  flow_finder.SetErodeDilate(1);
  flow_finder.SetAmountRange(/*min=*/3.0, /*max=*/3000.0);
  flow_finder.SetSpeedRange(/*min=*/3.0, /*max=*/-1.0);

  // ros::Publisher ratio_pub= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);
  std::vector<ros::Publisher> ratio_pubs(num_detectors);
  ratio_pubs[0]= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);
  for(int i(1); i<num_detectors; ++i)
    ratio_pubs[i]= node.advertise<std_msgs::Float64>(ToString("/color_occupied_ratio", i+1), 1);

  std::vector<ros::Publisher> mxy_pubs(num_detectors);
  mxy_pubs[0]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy", 1);
  for(int i(1); i<num_detectors; ++i)
    mxy_pubs[i]= node.advertise<std_msgs::Int32MultiArray>(ToString("/color_middle_xy", i+1), 1);

  ros::Publisher flow_pub;
  flow_pub= node.advertise<std_msgs::Float64MultiArray>("/flow_speed_angle", 1);

  cv::namedWindow("camera",1);
  cv::namedWindow("mask_img",1);
  cv::Mat frame, disp_img;
  col_detector.SetCameraWindow(frame);

  cv::setMouseCallback("camera", OnMouseCamera, &col_detector);
  cv::setMouseCallback("mask_img", OnMouseMask, &col_detector);

  cv::VideoWriter vout_camera, vout_mask;
  double time_prev= GetCurrentTime(), fps(10.0), fps_alpha(0.05);
  int    show_fps(0);

  // ros::Rate loop_rate(5);  // 5 Hz
  for(int f(0);ros::ok();++f)
  {
    if(running)
    {
      cap >> frame; // get a new frame from camera
      if(rotate90n!=0)  Rotate90N(frame,frame,rotate90n);
      cv::imshow("camera", frame);

      col_detector.Detect(frame, mode, /*verbose=*/false);
      col_detector.Draw(disp_img);

      flow_finder.Update(frame);
      flow_finder.DrawFlow(disp_img, CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/3);
      // for(int i(0);i<flow_finder.FlowElements().size();++i) std::cerr<<f<<" "<<flow_finder.FlowElements()[i]<<std::endl;

      std_msgs::Float64  ratio_msg;
      std_msgs::Int32MultiArray mxy_msg;
      for(int i(0); i<col_detector.Size(); ++i)
      {
        ratio_msg.data= col_detector.Ratio(i);
        ratio_pubs[i].publish(ratio_msg);

        mxy_msg.data.resize(2);
        mxy_msg.data[0]= col_detector.MedianX(i);
        mxy_msg.data[1]= col_detector.MedianY(i);
        mxy_pubs[i].publish(mxy_msg);
      }

      // TEST: Compute average speed, angle
      double sum_amt(0.0);
      cv::Vec2d avr_xy(0.0,0.0), avr_pvel(0.0,0.0), avr_vel(0.0,0.0);
      for(int i(0);i<flow_finder.FlowElements().size();++i)
      {
        double amt= flow_finder.FlowElements()[i].Amount;
        sum_amt+= amt;
        avr_xy[0]+= amt*flow_finder.FlowElements()[i].X;
        avr_xy[1]+= amt*flow_finder.FlowElements()[i].Y;
        avr_pvel[0]+= amt*flow_finder.FlowElements()[i].Speed;
        avr_pvel[1]+= amt*std::fabs(flow_finder.FlowElements()[i].Angle);
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
      if(avr_pvel[0]>20.0)
        cv::line(disp_img, cv::Point2d(avr_xy-0.5*avr_vel), cv::Point2d(avr_xy+0.5*avr_vel), CV_RGB(255,128,0), 5);
      // std::cerr<<"spd,angle: "<<avr_pvel<<std::endl;
      std_msgs::Float64MultiArray flow_msg;
      flow_msg.data.resize(2);
      flow_msg.data[0]= avr_pvel[0];
      flow_msg.data[1]= avr_pvel[1];
      flow_pub.publish(flow_msg);

      std::cerr<<"ratio:";
      for(int i(0); i<col_detector.Size(); ++i)
        std::cerr<<" "<<col_detector.Ratio(i);
      std::cerr<<"\t spd,angle: "<<avr_pvel<<std::endl;

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
    }

    // keyboard interface:
    int c(cv::waitKey(1));
    if(c=='\x1b'||c=='q') break;
    if(c=='[') {--rotate90n; std::cerr<<"rotate90n= "<<rotate90n<<std::endl;}
    if(c==']') {++rotate90n; std::cerr<<"rotate90n= "<<rotate90n<<std::endl;}
    else if(c=='r')
    {
      col_detector.Reset();
    }
    else if(c=='l')
    {
      col_detector.LoadColors(cd_idx, ColorFilesBase+DefaultFileNames[cd_idx]);
    }
    else if(c=='s')
    {
      col_detector.SaveColors(cd_idx, ColorFilesBase+DefaultFileNames[cd_idx]);
    }
    else if(c>='1' && c<='7')
    {
      int old_cd_idx(cd_idx);
      switch(c)
      {
      case '1':  cd_idx= 0; break;
      case '2':  cd_idx= 1; break;
      case '3':  cd_idx= 2; break;
      case '4':  cd_idx= 3; break;
      case '5':  cd_idx= 4; break;
      case '6':  cd_idx= 5; break;
      case '7':  cd_idx= 6; break;
      }
      if(cd_idx>=num_detectors)
        cd_idx= old_cd_idx;
      else
        std::cerr<<"###Selected: "<<(cd_idx+1)<<std::endl;
    }
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
  }


  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
