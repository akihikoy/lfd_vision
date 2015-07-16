//-------------------------------------------------------------------------------------------
/*! \file    test_col_flow.cpp
    \brief   Flow detection using color detection.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jul.15, 2015
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/color_detector.h"
#include "pr2_lfd_vision/flow_finder.h"
#include "pr2_lfd_vision/mov_detector.h"
#include "pr2_lfd_vision/vision_util.h"
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
//-------------------------------------------------------------------------------------------
namespace trick
{

TMultipleColorDetector FlowColDetector;
TFlowFinder FlowFinder;
int CDIdx(0);
bool Running(true);
// std::string ColFileName("xdefault_colors0.dat");
const char *DefaultFlowFileName= "ydefault_colors0.dat";
std::string ColorFilesBase= "";
double FPS(10.0);
int VizMode(2);  // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only, 5: 0.5*camera + flow + flow-mask
int FlowMaskMode(0);  // 0: none, 1: removing moving objects, 2: mask with colors

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
    FlowColDetector.CameraWindowMouseCallback(0, event, x, y, flags);
  }
  else
  {
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
  else if(c=='r')
  {
    FlowColDetector.Reset();
  }
  else if(c=='l')
  {
    FlowColDetector.LoadColors(0, ColorFilesBase+DefaultFlowFileName);
  }
  else if(c=='s')
  {
    FlowColDetector.SaveColors(0, ColorFilesBase+DefaultFlowFileName);
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
  cv::VideoCapture cap(0); // open the default camera
  if(argc==2)
  {
    cap.release();
    cap.open(atoi(argv[1]));
  }
  if(!cap.isOpened())  // check if we succeeded
  {
    std::cerr<<"no camera!"<<std::endl;
    return -1;
  }
  std::cerr<<"camera opened"<<std::endl;

  FlowColDetector.Setup(1);
  for(int i(0); i<1; ++i)
    FlowColDetector.LoadColors(i, ColorFilesBase+DefaultFlowFileName);
  // FlowColDetector.SetBlockAreaMin(/*block_area_min=*/10.0);
  FlowColDetector.RefDetector(0).SetDilationsErosions(5);
  CDIdx= 0;

  FlowFinder.SetOptFlowWinSize(cv::Size(3,3));
  FlowFinder.SetOptFlowSpdThreshold(5.0);
  FlowFinder.SetErodeDilate(1);
  FlowFinder.SetAmountRange(/*min=*/3.0, /*max=*/3000.0);
  FlowFinder.SetSpeedRange(/*min=*/3.0, /*max=*/-1.0);

  cv::namedWindow("color_detector",1);
  cv::Mat frame, disp_img;
  cv::Mat flow_mask_img;
  FlowColDetector.SetCameraWindow(frame);

  cv::setMouseCallback("color_detector", OnMouse);

  double time_prev= GetCurrentTime(), fps_alpha(0.05);
  int    show_fps(0);

  // ros::Rate loop_rate(5);  // 5 Hz
  for(int f(0);true;++f)
  {
    if(Running)
    {
      cap >> frame; // get a new frame from camera
      // if(frame_old.empty())  frame.copyTo(frame_old);
      // frame_diff= (frame - frame_old)*5.0;
      // cv::absdiff(frame_old, frame, frame_diff);  frame_diff*= 5.0;

      // Visualization setup
      // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only
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

      // For flow mask:
      // 0: none, 1: removing moving objects, 2: mask with colors
      if(FlowMaskMode==2)
      {
        // Color detection from image
        // flow_mask_img= FlowColDetector.Detector(0).Detect(frame);
        flow_mask_img= FlowColDetector.Detector(0).Detect(frame);
        // masked_img.setTo(cv::Scalar(0,0,0));
        // frame.copyTo(masked_img, flow_mask_img);
        // std::cerr<<flow_mask_img.type()<<","<<CV_8UC1<<std::endl;
        // FlowColDetector.Detect(frame, mode, /*verbose=*/false);
        // if(VizMode!=0)
        // {
          // FlowColDetector.Draw(disp_img);
          // cv::rectangle(disp_img, FlowColDetector.Bound(0), CV_RGB(0,255,0), 2);
        // }
        if(VizMode==5)
        {
          // masked_img.copyTo(disp_img);
          // disp_img+= masked_img;
          frame.copyTo(disp_img, flow_mask_img);
        }
      }

      // Flow detection from image
      FlowFinder.Update(frame);
      // FlowFinder.Update(masked_img);
      // Remove detected flows of moving objects (e.g. robot hand)
      // if(FlowMaskMode==1)  RemoveMovingObjectFlow(FlowFinder, MovDetector);
      if(FlowMaskMode==2)  MaskFlow(FlowFinder, flow_mask_img);
      if(VizMode!=0)
        FlowFinder.DrawFlow(disp_img, CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/3);

      // Compute average flow
      cv::Vec2d avr_xy(0.0,0.0), avr_vel(0.0,0.0), avr_spddir(0.0,0.0);
      CalcFlowAverage(FlowFinder, avr_xy, avr_vel, avr_spddir);

      if(VizMode!=0 && avr_spddir[0]>20.0)
        cv::line(disp_img, cv::Point2d(avr_xy-0.5*avr_vel), cv::Point2d(avr_xy+0.5*avr_vel), CV_RGB(255,128,0), 5);


      cv::imshow("color_detector", disp_img);
      // frame.copyTo(frame_old);

      // update fps
      FPS= fps_alpha*(1.0/(GetCurrentTime()-time_prev)) + (1.0-fps_alpha)*FPS;
      time_prev= GetCurrentTime();
      if(show_fps==0)  {std::cerr<<"FPS: "<<FPS<<std::endl;  show_fps=20;}
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }

    if(!HandleKeyEvent())  break;
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
