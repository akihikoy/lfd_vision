//-------------------------------------------------------------------------------------------
/*! \file    usb_stereo_node.cpp
    \brief   USB stereo integration with color and flow detections.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Apr.04, 2016

    cf. color_detector_node.cpp
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/color_detector.h"
#include "lfd_vision/flow_finder.h"
#include "lfd_vision/usb_stereo.h"
#include "lfd_vision/geom_util.h"
#include "lfd_vision/vision_util.h"
#include "lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include "lfd_vision/ColDetSensor.h"
#include "lfd_vision/ColDetViz.h"
#include "lfd_vision/ColDetVizPrimitive.h"
#include "lfd_vision/ROI_3DProj.h"
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
#include <boost/thread.hpp>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
const char *ColFileNames[]={
    "default_colors1.dat",
    "default_colors2.dat",
    "default_colors3.dat",
    "default_colors4.dat",
    "default_colors5.dat",
    "default_colors6.dat",
    "default_colors7.dat"};
std::string ColorFilesBase[]={"x","x2"};

int  CameraIdx(0), CDIdx(0);  // Focused camera index, color index in ColDetector
bool Running(true), Shutdown(false);
TMultipleColorDetector ColDetector[2];
TFlowFinder FlowFinder[2];
TStereo Stereo;
// TStereo StereoF;

int NumColDetectors(1);
int NRotate90(0);
TEasyVideoOut VideoOut[2];
int VizMode[]= {2,2};  // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only

std::string ImgWin("1100111");  // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2

std::vector<lfd_vision::ColDetVizPrimitive> VizObjs[2];  // External visualization requests.

ros::Publisher CloudPub, FlowCloudPub;
cv::Mat Frame[2];
boost::mutex MutCamCapture;

lfd_vision::ROI_3DProj ROIColDet, ROIStereoF;
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
  if(event!=0)  CameraIdx= *(reinterpret_cast<int*>(data));

  if(flags==cv::EVENT_FLAG_SHIFTKEY)
  {
    ColDetector[CameraIdx].CameraWindowMouseCallback(CDIdx, event, x, y, flags);
  }
  else
  {
    ColDetector[CameraIdx].MaskWindowMouseCallback(CDIdx, event, x, y, flags);

    if(event == cv::EVENT_RBUTTONDOWN)
    {
      Running=!Running;
      std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
    }
  }
}
/*
  Right click: pause/resume
*/
void OnMouseSimple(int event, int x, int y, int flags, void *data)
{
  if(flags!=0)  return;
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
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
    for(int j(0);j<2;++j)
      ColDetector[j].Reset();
  }
  else if(c=='l')
  {
    ColDetector[CameraIdx].LoadColors(CDIdx, ColorFilesBase[CameraIdx]+ColFileNames[CDIdx]);
  }
  else if(c=='s')
  {
    ColDetector[CameraIdx].SaveColors(CDIdx, ColorFilesBase[CameraIdx]+ColFileNames[CDIdx]);
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
    if(CDIdx>=NumColDetectors)
      CDIdx= old_cd_idx;
    else
      std::cerr<<"###Selected: "<<(CDIdx+1)<<std::endl;
  }
  else if(c=='W')
  {
    for(int j(0);j<2;++j)
      VideoOut[j].Switch();
  }
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  else if(c=='m' || c=='M')
  {
    if(c=='m')  ++VizMode[CameraIdx];  else  --VizMode[CameraIdx];
    if(VizMode[CameraIdx]>4)  VizMode[CameraIdx]= 0;
    if(VizMode[CameraIdx]<0)  VizMode[CameraIdx]= 4;
    std::cerr<<"VizMode["<<CameraIdx<<"]: "<<VizMode[CameraIdx]<<std::endl;
  }

  return true;
}
//-------------------------------------------------------------------------------------------

void DrawExternalViz(int cam_idx, cv::Mat &disp_img)
{
  for(std::vector<lfd_vision::ColDetVizPrimitive>::const_iterator itr(VizObjs[cam_idx].begin()),itr_end(VizObjs[cam_idx].end());
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

void ProjectROIToMask(const lfd_vision::ROI_3DProj &roi,
    const cv::Size &size1, const cv::Size &size2, const TCameraParams &cam_params,
    cv::Mat &mask1, cv::Mat &mask2)
{
  mask1.create(size1, CV_8U);
  mask2.create(size2, CV_8U);
  if(roi.type==roi.NONE)
  {
    mask1.setTo(255);
    mask2.setTo(255);
  }
  else if(roi.type==roi.POLYGON)
  {
    cv::Mat points3d(roi.param.size()/3,3,CV_32F), points2d1, points2d2;
    for(int i(0),i_end(roi.param.size()/3); i<i_end; ++i)
    {
      points3d.at<float>(i,0)= roi.param[3*i];
      points3d.at<float>(i,1)= roi.param[3*i+1];
      points3d.at<float>(i,2)= roi.param[3*i+2];
    }
// // Refine 3D points.  FIXME: This is inaccurate. Modify each line, not each point.
// for(int r(0),rows(points3d.rows); r<rows; ++r)
  // if(points3d.at<float>(r,2)<0.0)  points3d.at<float>(r,2)= 0.0;
    cv::Vec3f rvec1(0.0,0.0,0.0), tvec1(0.0,0.0,0.0);
    cv::projectPoints(points3d, rvec1, tvec1, cam_params.K1, cam_params.D1, points2d1);
    cv::projectPoints(points3d, cam_params.R, cam_params.T, cam_params.K2, cam_params.D2, points2d2);
    points2d1.convertTo(points2d1, CV_32S);
    points2d2.convertTo(points2d2, CV_32S);
    for(int r(0),rows(points2d1.rows); r<rows; ++r)
    {
      if(points2d1.at<int>(r,0)<-size1.width)   points2d1.at<int>(r,0)= -size1.width;
      if(points2d1.at<int>(r,0)>2*size1.width)  points2d1.at<int>(r,0)= 2*size1.width;
      if(points2d1.at<int>(r,1)<-size1.height)   points2d1.at<int>(r,1)= -size1.height;
      if(points2d1.at<int>(r,1)>2*size1.height)  points2d1.at<int>(r,1)= 2*size1.height;
    }
    for(int r(0),rows(points2d2.rows); r<rows; ++r)
    {
      if(points2d2.at<int>(r,0)<-size2.width)   points2d2.at<int>(r,0)= -size2.width;
      if(points2d2.at<int>(r,0)>2*size2.width)  points2d2.at<int>(r,0)= 2*size2.width;
      if(points2d2.at<int>(r,1)<-size2.height)   points2d2.at<int>(r,1)= -size2.height;
      if(points2d2.at<int>(r,1)>2*size2.height)  points2d2.at<int>(r,1)= 2*size2.height;
    }

    mask1.setTo(0);
    mask2.setTo(0);
    std::vector<cv::Mat> ppt1(1),ppt2(1);
    ppt1[0]= points2d1;
    ppt2[0]= points2d2;
    cv::fillPoly(mask1, ppt1, cv::Scalar(255));
    cv::fillPoly(mask2, ppt2, cv::Scalar(255));
  }
  else if(roi.type==roi.CONE)
  {
    int N= 16;
    cv::Mat points3d(1+N,3,CV_32F), rot, points2d1, points2d2, hull1, hull2;
    points3d.at<float>(0,0)= 0.0;
    points3d.at<float>(0,1)= 0.0;
    points3d.at<float>(0,2)= 0.0;
    float rad= roi.param[7]*std::tan(roi.param[6]);
    for(int i(0); i<N; ++i)
    {
      float th=(float)i*M_PI*2.0/(float)(N);
      points3d.at<float>(1+i,0)= rad*std::cos(th);
      points3d.at<float>(1+i,1)= rad*std::sin(th);
      points3d.at<float>(1+i,2)= 0.0;
    }
    float v1[]={0.,0.,-1.}, v2[]={roi.param[3],roi.param[4],roi.param[5]};
    cv::Vec3f axis_angle;
    GetAxisAngle(v1, v2, axis_angle);
    cv::Rodrigues(axis_angle, rot);
// std::cerr<<"P-1 "<<axis_angle<<std::endl;
// std::cerr<<"P-1 "<<rot<<std::endl;
    points3d= points3d*rot.t();
    points3d.at<float>(0,0)+= roi.param[0];
    points3d.at<float>(0,1)+= roi.param[1];
    points3d.at<float>(0,2)+= roi.param[2];
    cv::Mat_<float> center= points3d.row(0)+roi.param[7]*cv::Mat(cv::Vec3f(roi.param[3],roi.param[4],roi.param[5])).t();
    for(int i(0); i<N; ++i)
      points3d.row(1+i)+= center;
  // // Refine 3D points.  FIXME: This is inaccurate. Modify each line, not each point.
  // float min_z(0.5);
  // for(int r(0),rows(points3d.rows); r<rows; ++r)
    // if(points3d.at<float>(r,2)<min_z)  points3d.at<float>(r,2)= min_z;
// std::cerr<<"P0 "<<points3d<<std::endl;
    // cv::Vec3f rvec1(0.0,0.0,0.0), tvec1(0.0,0.0,0.0);
    // cv::projectPoints(points3d, rvec1, tvec1, cam_params.K1, cam_params.D1, points2d1);
    // cv::projectPoints(points3d, cam_params.R, cam_params.T, cam_params.K2, cam_params.D2, points2d2);
    ProjectPointsToRectifiedImg(points3d, cam_params.P1, points2d1);
    ProjectPointsToRectifiedImg(points3d, cam_params.P2, points2d2);
// std::cerr<<"P0.4 "<<points2d1<<std::endl;
// std::cerr<<"P0.4 "<<points2d2<<std::endl;
    points2d1.convertTo(points2d1, CV_32SC1);
    points2d2.convertTo(points2d2, CV_32SC1);
// std::cerr<<"P0.5 "<<points2d1<<std::endl;
// std::cerr<<"P0.5 "<<points2d2<<std::endl;
    // for(int r(0),rows(points2d1.rows); r<rows; ++r)
    // {
      // if(points2d1.at<int>(r,0)<-size1.width)   points2d1.at<int>(r,0)= -size1.width;
      // if(points2d1.at<int>(r,0)>2*size1.width)  points2d1.at<int>(r,0)= 2*size1.width;
      // if(points2d1.at<int>(r,1)<-size1.height)   points2d1.at<int>(r,1)= -size1.height;
      // if(points2d1.at<int>(r,1)>2*size1.height)  points2d1.at<int>(r,1)= 2*size1.height;
    // }
    // for(int r(0),rows(points2d2.rows); r<rows; ++r)
    // {
      // if(points2d2.at<int>(r,0)<-size2.width)   points2d2.at<int>(r,0)= -size2.width;
      // if(points2d2.at<int>(r,0)>2*size2.width)  points2d2.at<int>(r,0)= 2*size2.width;
      // if(points2d2.at<int>(r,1)<-size2.height)   points2d2.at<int>(r,1)= -size2.height;
      // if(points2d2.at<int>(r,1)>2*size2.height)  points2d2.at<int>(r,1)= 2*size2.height;
    // }
    // ver.2
    // cv::vector<int> idx_valid;
    // idx_valid.reserve(points2d1.rows);
    // for(int r(0),rows(points2d1.rows); r<rows; ++r)
    // {
      // if(  points2d1.at<int>(r,0)>=-size1.width  && points2d1.at<int>(r,0)<2*size1.width
        // && points2d1.at<int>(r,1)>=-size1.height && points2d1.at<int>(r,1)<2*size1.height
        // && points2d2.at<int>(r,0)>=-size2.width  && points2d2.at<int>(r,0)<2*size2.width
        // && points2d2.at<int>(r,1)>=-size2.height && points2d2.at<int>(r,1)<2*size2.height )
        // idx_valid.push_back(r);
    // }
    // ExtractRows(points2d1,idx_valid,points2d1);
    // ExtractRows(points2d2,idx_valid,points2d2);

// std::cerr<<"P1 "<<points2d1.rows<<", "<<points2d1.cols<<std::endl;
// std::cerr<<"P1 "<<points2d2.rows<<", "<<points2d2.cols<<std::endl;
// std::cerr<<"P1 "<<points2d1<<std::endl;
// std::cerr<<"P1 "<<points2d2<<std::endl;
    cv::convexHull(points2d1, hull1);
    cv::convexHull(points2d2, hull2);
// std::cerr<<"P2 "<<hull1.rows<<", "<<hull1.cols<<std::endl;
// std::cerr<<"P2 "<<hull2.rows<<", "<<hull2.cols<<std::endl;
    mask1.setTo(0);
    mask2.setTo(0);
// std::cerr<<"P3 "<<mask1.rows<<", "<<mask1.cols<<std::endl;
// std::cerr<<"P3 "<<mask2.rows<<", "<<mask2.cols<<std::endl;
// std::cerr<<"P3 "<<hull1<<std::endl;
// std::cerr<<"P3 "<<hull2<<std::endl;
    std::vector<cv::Mat> ppt1(1),ppt2(1);
    ppt1[0]= hull1;
    ppt2[0]= hull2;
    cv::fillPoly(mask1, ppt1, cv::Scalar(255));
    cv::fillPoly(mask2, ppt2, cv::Scalar(255));
// std::cerr<<"P4 "<<mask1.rows<<", "<<mask1.cols<<std::endl;
// std::cerr<<"P4 "<<mask2.rows<<", "<<mask2.cols<<std::endl;
// cv::imshow("mask1", mask1);
// cv::imshow("mask2", mask2);
  }
}
//-------------------------------------------------------------------------------------------

class TColDetVizCallback
{
private:
  int cam_idx_;
public:
  TColDetVizCallback(int cam_idx) : cam_idx_(cam_idx) {}
  void operator()(const lfd_vision::ColDetVizConstPtr &msg)
  {
    VizObjs[cam_idx_]= msg->objects;
  }
};
//-------------------------------------------------------------------------------------------

void ROICallback(const lfd_vision::ROI_3DProjPtr &msg)
{
  if(msg->target=="col_det")        ROIColDet= *msg;
  else if(msg->target=="stereo_f")  ROIStereoF= *msg;
}
//-------------------------------------------------------------------------------------------

bool ResetAmount(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resetting base amount..."<<std::endl;
  for(int j(0); j<2; ++j)
    ColDetector[j].Reset();
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

// Get point cloud by stereo
void ExecStereo()
{
  cv::Mat frame[2];
  cv::Mat disparity;
  while(!Shutdown)
  {
    if(Running)
    {
      {
        boost::mutex::scoped_lock lock(MutCamCapture);
        for(int j(0);j<2;++j)  Frame[j].copyTo(frame[j]);
      }
      Stereo.Proc(frame[0],frame[1]);
      // Stereo.StereoParams().GrayScale= false;
      // Stereo.StereoParams().StereoMethod= TStereoSGBMParams::smBM;
      // Stereo.Proc(FlowFinder[0].FlowMask(),FlowFinder[1].FlowMask());
      cv::normalize(Stereo.Disparity(), disparity, 0, 255, CV_MINMAX, CV_8U);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
      if(ImgWin[2]=='1')  cv::imshow("stereo_frame_l", Stereo.FrameL());
      if(ImgWin[3]=='1')  cv::imshow("stereo_frame_r", Stereo.FrameR());
      if(ImgWin[4]=='1')  cv::imshow("stereo_disparity", disparity);

      // Publish as point cloud.
      Stereo.ReprojectTo3D();
      sensor_msgs::PointCloud2 cloud_msg;
      ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(cloud_msg,
          ConvertXYZImageToPointCloud(Stereo.XYZ(),Stereo.RGB()),
          /*frame_id=*/"usb_stereo");
      CloudPub.publish(cloud_msg);
      usleep(50*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

#if 0
// Get point cloud of flow by stereo
void FlowStereo(int we, int wd)
{
  cv::Mat frame1, frame2, disparity;

  FlowFinder[0].FlowMask().copyTo(frame1);
  FlowFinder[1].FlowMask().copyTo(frame2);
  frame1*= 200;
  frame2*= 200;
  cv::erode(frame1,frame1,cv::Mat(),cv::Point(-1,-1), we);
  cv::dilate(frame1,frame1,cv::Mat(),cv::Point(-1,-1), wd);
  cv::erode(frame2,frame2,cv::Mat(),cv::Point(-1,-1), we);
  cv::dilate(frame2,frame2,cv::Mat(),cv::Point(-1,-1), wd);
  // cv::Mat mask1(frame1), mask2(frame2);
  // Frame[0].copyTo(frame1, mask1);
  // Frame[1].copyTo(frame2, mask2);

  StereoF.Proc(frame1, frame2);
  cv::normalize(StereoF.Disparity(), disparity, 0, 255, CV_MINMAX, CV_8U);
  cv::imshow("stereof_frame_l", StereoF.FrameL());
  cv::imshow("stereof_frame_r", StereoF.FrameR());
  cv::imshow("stereof_disparity", disparity);

  // Publish as point cloud.
  StereoF.ReprojectTo3D();
  sensor_msgs::PointCloud2 cloud_msg;
  ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(cloud_msg,
      ConvertXYZImageToPointCloud(StereoF.XYZ(),StereoF.RGB()),
      /*frame_id=*/"usb_stereo");
  FlowCloudPub.publish(cloud_msg);
}
#endif
//-------------------------------------------------------------------------------------------

// Get point cloud of flow by flow-stereo algorithm
struct TFlowStereo2
{
  int We;
  int Wd;
  int XFilter, YFilter;
  int XStep, YStep;
  int ThMatch;

  // Filter kernel:
  cv::Mat Kernel;

  // Temporary containers:
  cv::Mat frame1, frame2, mask1, mask2;
  cv::Mat seg1, seg2, tmp;
  std::vector<int> matched;
  std::vector<cv::Point2f> points1,points2;
  cv::Mat points4d;
  cv::Mat frame1c,frame2c;  // For visualization

  TFlowStereo2()
    {
      We= 2;
      Wd= 3;
      XFilter= 1;
      YFilter= 32;
      XStep= 1;
      YStep= 16;
      ThMatch= 16;
    }
  void Init()
    {
      Kernel.create(cv::Size(XFilter,YFilter),CV_32F);
      Kernel= cv::Mat::ones(Kernel.size(),CV_32F)/(float)(Kernel.rows*Kernel.cols);
    }
  void operator()()
    {
      ProjectROIToMask(ROIStereoF, FlowFinder[0].FlowMask().size(), FlowFinder[1].FlowMask().size(), Stereo.CameraParams(), mask1, mask2);
      frame1.setTo(0);
      frame2.setTo(0);
      FlowFinder[0].FlowMask().copyTo(frame1, mask1);
      FlowFinder[1].FlowMask().copyTo(frame2, mask2);
      frame1*= 200;
      frame2*= 200;

      // Remove noise, make remaining pixels bigger:
      cv::erode(frame1,frame1,cv::Mat(),cv::Point(-1,-1), We);
      cv::dilate(frame1,frame1,cv::Mat(),cv::Point(-1,-1), Wd);
      cv::erode(frame2,frame2,cv::Mat(),cv::Point(-1,-1), We);
      cv::dilate(frame2,frame2,cv::Mat(),cv::Point(-1,-1), Wd);
      // Vertical filter to make detecting flow easier:
      cv::filter2D(frame1, frame1, /*ddepth=*/-1, Kernel);
      cv::filter2D(frame2, frame2, /*ddepth=*/-1, Kernel);

      matched.resize(frame1.rows);
      for(int y(0),y_end(std::min(frame1.rows,frame2.rows)-YStep); y<y_end; y+=YStep)
      {
        cv::Mat seg1(frame1,cv::Rect(0,y,frame1.cols,1));
        cv::Mat seg2(frame2,cv::Rect(0,y,frame2.cols,1));
        int dx(0);
        double match(0.0), max_match(0.0), x_match(0);
        for(int x(0),x_end(frame1.cols); x<x_end; x+=XStep)
        {
          dx= frame1.cols-x;
          cv::bitwise_and(seg1(cv::Rect(x,0,dx,1)), seg2(cv::Rect(0,0,dx,1)), tmp);
          match= cv::sum(tmp)[0];
          if(match>max_match)  {x_match=x; max_match=match;}
        }
        if(max_match>ThMatch)
          for(int y2(y);y2<y+YStep;++y2)  matched[y2]= x_match;
        else
          for(int y2(y);y2<y+YStep;++y2)  matched[y2]= -1;
        // std::cerr<<" "<<matched[y];
      }
      // std::cerr<<std::endl;

      points1.clear();
      points2.clear();
      cv::cvtColor(frame1, frame1c, CV_GRAY2BGR);
      cv::cvtColor(frame2, frame2c, CV_GRAY2BGR);
      frame1c/=2;
      frame2c/=2;
      for(int y(0),y_end(matched.size()); y<y_end; y+=YStep)
      {
        int dx= matched[y];
        if(dx>=0)
        {
          for(int x(dx),x_end(frame1.cols); x<x_end; x+=XStep)
          {
            if(frame1.at<unsigned char>(y,x)>10 && frame2.at<unsigned char>(y,x-dx)>10)
            {
              for(int y2(y);y2<y+YStep;++y2)
              {
                frame1c.at<cv::Vec3b>(y2,x)+= cv::Vec3b(x%128,0,128-(x%128));
                frame2c.at<cv::Vec3b>(y2,x-dx)+= cv::Vec3b(x%128,0,128-(x%128));
              }
              points1.push_back(cv::Point2f(x,y));
              points2.push_back(cv::Point2f(x-dx,y));
            }
          }
        }
      }

      if(points1.size()>0)
      {
        cv::triangulatePoints(Stereo.CameraParams().P1, Stereo.CameraParams().P2,
            points1, points2, points4d);
        //*DBG*/std::cerr<<"points4d[0]="<<points4d.col(0)<<std::endl;
      }
      else
        points4d= cv::Mat::zeros(0,0,CV_32F);

      // cv::normalize(disparity, disparity, 0, 255, CV_MINMAX, CV_8U);
      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
      if(ImgWin[5]=='1')  cv::imshow("stereof_frame_l", frame1c);
      if(ImgWin[6]=='1')  cv::imshow("stereof_frame_r", frame2c);
      // cv::imshow("stereof_disparity", disparity);

      // Publish as point cloud.
      sensor_msgs::PointCloud2 cloud_msg;
      ConvertPointCloudToROSMsg<pcl::PointXYZ>(cloud_msg,
          Convert4DPointsToPointCloud(points4d),
          /*frame_id=*/"usb_stereo");
      FlowCloudPub.publish(cloud_msg);
    }
};
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "usb_stereo_node");
  ros::NodeHandle node("~");
  int camera[]= {0,1};
  int cap_width(640), cap_height(480);
  double block_area_min(10.0);

  int    ff_ofl_win(3);  // FlowFinder.OptFlowWinSize
  double ff_ofl_spd_min(2.0);  // FlowFinder.OptFlowSpdThreshold
  int    ff_er_dl(1);  // FlowFinder.ErodeDilate
  double ff_amt_min(1.0), ff_amt_max(3000.0);  // FlowFinder.AmountRange
  double ff_spd_min(1.0), ff_spd_max(-1.0);  // FlowFinder.SpeedRange
  int    ff_mask_flen(5);  // FlowFinder.FlowMaskFilterLen

  std::string stereo_param_yaml("config/ext_usbcam1_stereo3.yaml");
  int stereo_f_We(2);
  int stereo_f_Wd(3);
  int stereo_f_XFilter(1), stereo_f_YFilter(32);
  int stereo_f_XStep(1), stereo_f_YStep(16);
  int stereo_f_ThMatch(16);

  std::string vout_base("/tmp/vout");

  node.param("camera1",camera[0],camera[0]);
  node.param("camera2",camera[1],camera[1]);
  node.param("cap_width",cap_width,cap_width);
  node.param("cap_height",cap_height,cap_height);
  node.param("viz_mode1",VizMode[0],VizMode[0]);
  node.param("viz_mode2",VizMode[1],VizMode[1]);
  node.param("num_detectors",NumColDetectors,NumColDetectors);
  node.param("block_area_min",block_area_min,block_area_min);
  node.param("color_files_base1",ColorFilesBase[0],ColorFilesBase[0]);
  node.param("color_files_base2",ColorFilesBase[1],ColorFilesBase[1]);
  node.param("rotate90n",NRotate90,NRotate90);
  node.param("img_win",ImgWin,ImgWin);

  node.param("ff_ofl_win",ff_ofl_win,ff_ofl_win);
  node.param("ff_ofl_spd_min",ff_ofl_spd_min,ff_ofl_spd_min);
  node.param("ff_er_dl",ff_er_dl,ff_er_dl);
  node.param("ff_amt_min",ff_amt_min,ff_amt_min);
  node.param("ff_amt_max",ff_amt_max,ff_amt_max);
  node.param("ff_spd_min",ff_spd_min,ff_spd_min);
  node.param("ff_spd_max",ff_spd_max,ff_spd_max);
  node.param("ff_mask_flen",ff_mask_flen,ff_mask_flen);

  node.param("stereo_param_yaml",stereo_param_yaml,stereo_param_yaml);
  node.param("stereo_f_We",stereo_f_We,stereo_f_We);
  node.param("stereo_f_Wd",stereo_f_Wd,stereo_f_Wd);
  node.param("stereo_f_XFilter",stereo_f_XFilter,stereo_f_XFilter);
  node.param("stereo_f_YFilter",stereo_f_YFilter,stereo_f_YFilter);
  node.param("stereo_f_XStep",stereo_f_XStep,stereo_f_XStep);
  node.param("stereo_f_YStep",stereo_f_YStep,stereo_f_YStep);
  node.param("stereo_f_ThMatch",stereo_f_ThMatch,stereo_f_ThMatch);

  node.param("vout_base",vout_base,vout_base);

  ROIColDet.type= ROIColDet.NONE;
  ROIStereoF.type= ROIStereoF.NONE;

  cv::VideoCapture cap1(camera[0]), cap2(camera[1]);
  if(!cap1.isOpened() || !cap2.isOpened())  // check if we succeeded
  {
    std::cerr<<"Cannot open one of/both: "<<camera[0]<<", "<<camera[1]<<std::endl;
    return -1;
  }
  std::cerr<<"Camera opened"<<std::endl;

  cv::Size img_size(cap_width,cap_height);
  cap1.set(CV_CAP_PROP_FRAME_WIDTH, cap_width);
  cap1.set(CV_CAP_PROP_FRAME_HEIGHT, cap_height);
  cap2.set(CV_CAP_PROP_FRAME_WIDTH, cap_width);
  cap2.set(CV_CAP_PROP_FRAME_HEIGHT, cap_height);

  for(int j(0);j<2;++j)
    VideoOut[j].SetfilePrefix(vout_base);

  for(int j(0); j<2; ++j)
  {
    ColDetector[j].Setup(NumColDetectors);
    for(int i(0); i<NumColDetectors; ++i)
      ColDetector[j].LoadColors(i, ColorFilesBase[j]+ColFileNames[i]);
    ColDetector[j].SetBlockAreaMin(block_area_min);
  }
  CDIdx= 0;

  for(int j(0); j<2; ++j)
  {
    // 0: Full, 1: FlowMask only
    // FlowFinder[j].SetProcType(1);
    FlowFinder[j].SetOptFlowWinSize(cv::Size(ff_ofl_win,ff_ofl_win));
    FlowFinder[j].SetOptFlowSpdThreshold(ff_ofl_spd_min);
    FlowFinder[j].SetErodeDilate(ff_er_dl);
    FlowFinder[j].SetAmountRange(/*min=*/ff_amt_min, /*max=*/ff_amt_max);
    FlowFinder[j].SetSpeedRange(/*min=*/ff_spd_min, /*max=*/ff_spd_max);
    FlowFinder[j].SetFlowMaskFilterLen(ff_mask_flen);
  }

  Stereo.LoadCameraParametersFromYAML(stereo_param_yaml);
  Stereo.SetImageSize(img_size);
  Stereo.SetRecommendedStereoParams();
  Stereo.Init();
  // StereoF.LoadCameraParametersFromYAML(stereo_param_yaml);
  // StereoF.SetImageSize(img_size);
  // StereoF.SetRecommendedStereoParams();
  // StereoF.StereoParams().StereoMethod= TStereoSGBMParams::smBM;
  // StereoF.Init();
  TFlowStereo2 stereo_f;
  stereo_f.We= stereo_f_We;
  stereo_f.Wd= stereo_f_Wd;
  stereo_f.XFilter= stereo_f_XFilter;
  stereo_f.YFilter= stereo_f_YFilter;
  stereo_f.XStep= stereo_f_XStep;
  stereo_f.YStep= stereo_f_YStep;
  stereo_f.ThMatch= stereo_f_ThMatch;
  stereo_f.Init();

  ros::Publisher sensor_pub[2];
  sensor_pub[0]= node.advertise<lfd_vision::ColDetSensor>("sensor1", 1);
  sensor_pub[1]= node.advertise<lfd_vision::ColDetSensor>("sensor2", 1);
  CloudPub= node.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
  FlowCloudPub= node.advertise<sensor_msgs::PointCloud2>("flow_cloud", 1);

  ros::Subscriber sub_viz1= node.subscribe<lfd_vision::ColDetViz>("viz1", 1, TColDetVizCallback(0));
  ros::Subscriber sub_viz2= node.subscribe<lfd_vision::ColDetViz>("viz2", 1, TColDetVizCallback(1));
  ros::Subscriber sub_roi= node.subscribe("roi", 1, &ROICallback);

  ros::ServiceServer srv_reset= node.advertiseService("reset", &ResetAmount);
  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);

  cv::Mat frame[2], disp_img[2], mask[2];
  for(int j(0);j<2;++j)
    ColDetector[j].SetCameraWindow(frame[j]);

  // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
  int camera_indexes[]= {0,1};
  if(ImgWin[0]=='1')  cv::namedWindow("color_detector1",1);
  if(ImgWin[0]=='1')  cv::setMouseCallback("color_detector1", OnMouse, &camera_indexes[0]);
  if(ImgWin[1]=='1')  cv::namedWindow("color_detector2",1);
  if(ImgWin[1]=='1')  cv::setMouseCallback("color_detector2", OnMouse, &camera_indexes[1]);

  if(ImgWin[2]=='1')  cv::namedWindow("stereo_frame_l",1);
  if(ImgWin[2]=='1')  cv::setMouseCallback("stereo_frame_l", OnMouseSimple);
  if(ImgWin[3]=='1')  cv::namedWindow("stereo_frame_r",1);
  if(ImgWin[3]=='1')  cv::setMouseCallback("stereo_frame_r", OnMouseSimple);
  if(ImgWin[4]=='1')  cv::namedWindow("stereo_disparity",1);
  if(ImgWin[4]=='1')  cv::setMouseCallback("stereo_disparity", OnMouseSimple);

  if(ImgWin[5]=='1')  cv::namedWindow("stereof_frame_l",1);
  if(ImgWin[5]=='1')  cv::setMouseCallback("stereof_frame_l", OnMouseSimple);
  if(ImgWin[6]=='1')  cv::namedWindow("stereof_frame_r",1);
  if(ImgWin[6]=='1')  cv::setMouseCallback("stereof_frame_r", OnMouseSimple);
  // cv::namedWindow("stereof_disparity",1);
  // cv::setMouseCallback("stereof_disparity", OnMouseSimple);

  int show_fps(0);

  // Dummy capture.
  cap1 >> Frame[0];
  cap2 >> Frame[1];

  boost::thread th_stereo(&ExecStereo);

  // ros::Rate loop_rate(5);  // 5 Hz
  for(int f(0);ros::ok();++f)
  {
    if(Running)
    {
      ProjectROIToMask(ROIColDet, img_size, img_size, Stereo.CameraParams(), mask[0], mask[1]);
      frame[0].setTo(0);
      frame[1].setTo(0);

      cap1 >> Frame[0]; // get a new frame from camera
      cap2 >> Frame[1]; // get a new frame from camera
      {
        boost::mutex::scoped_lock lock(MutCamCapture);
        for(int j(0);j<2;++j)  Frame[j].copyTo(frame[j], mask[j]);
      }

      for(int cam_idx(0); cam_idx<2; ++cam_idx)
      {
        if(NRotate90!=0)  Rotate90N(frame[cam_idx],frame[cam_idx],NRotate90);

        // Visualization setup
        // 0: camera only, 1: camera + detected, 2: 0.5*camera + detected, 3: 0.25*camera + detected, 4: detected only
        if(VizMode[cam_idx]==0 || VizMode[cam_idx]==1)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);
        }
        else if(VizMode[cam_idx]==2)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);
          disp_img[cam_idx]*= 0.5;
        }
        else if(VizMode[cam_idx]==3)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);
          disp_img[cam_idx]*= 0.25;
        }
        else if(VizMode[cam_idx]==4)
        {
          frame[cam_idx].copyTo(disp_img[cam_idx]);  // TODO: make this efficient (not need to copy)
          disp_img[cam_idx].setTo(cv::Scalar(0,0,0));
        }

        // Color detection from image
        ColDetector[cam_idx].Detect(frame[cam_idx], /*mode=*/2, /*verbose=*/false);
        if(VizMode[cam_idx]!=0)
        {
          ColDetector[cam_idx].Draw(disp_img[cam_idx]);
          if(CDIdx>=0)
            cv::rectangle(disp_img[cam_idx], ColDetector[cam_idx].Bound(CDIdx), CV_RGB(0,255,0), 2);
        }

        // Flow detection from image
        FlowFinder[cam_idx].Update(frame[cam_idx]);
        if(VizMode[cam_idx]!=0)
          FlowFinder[cam_idx].DrawFlow(disp_img[cam_idx], CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/3);

        if(VizMode[cam_idx]!=0)
          DrawExternalViz(cam_idx, disp_img[cam_idx]);

        // TEST: show FlowMask only.
        // FlowFinder[cam_idx].FlowMask().copyTo(disp_img[cam_idx]);
        // disp_img[cam_idx]*= 200;

        // Send ROS topics
        {
          lfd_vision::ColDetSensor  sensor_msg;

          sensor_msg.num_cols= ColDetector[cam_idx].Size();
          sensor_msg.col_filled_ratio= ColDetector[cam_idx].DataRatio();
          sensor_msg.col_center_xy= ColDetector[cam_idx].DataCenterXY();
          sensor_msg.col_area= ColDetector[cam_idx].DataArea();
          sensor_msg.col_bound= ColDetector[cam_idx].DataBound();

          sensor_msg.nums_blocks= ColDetector[cam_idx].NumsBlocks();
          sensor_msg.blocks_area.resize(ColDetector[cam_idx].BlocksArea().size());
          std::copy(ColDetector[cam_idx].BlocksArea().begin(),ColDetector[cam_idx].BlocksArea().end(), sensor_msg.blocks_area.begin());
          sensor_msg.blocks_center_xy.resize(ColDetector[cam_idx].BlocksCenterXY().size());
          std::copy(ColDetector[cam_idx].BlocksCenterXY().begin(),ColDetector[cam_idx].BlocksCenterXY().end(), sensor_msg.blocks_center_xy.begin());

          const std::list<TFlowElement> &flow(FlowFinder[cam_idx].FlowElements());
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

          // DEPRECATED: sensor_msg.{flow_avr_xy,flow_avr_vxy,flow_avr_spddir}

          sensor_pub[cam_idx].publish(sensor_msg);
        }

        // VideoOut[cam_idx].Step(disp_img[cam_idx]);
        // VideoOut[cam_idx].VizRec(disp_img[cam_idx]);
      }  // for cam_idx

      std::cerr<<"ratio:";
      for(int cam_idx(0); cam_idx<2; ++cam_idx)
      {
        if(cam_idx>0)  std::cerr<<" | ";
        for(int i(0); i<ColDetector[cam_idx].Size(); ++i)
          std::cerr<<" "<<ColDetector[cam_idx].Ratio(i);
      }
      std::cerr<<std::endl;

      // TEST
      // FlowFinder[0].FlowMask().copyTo(disp_img[0]);
      // FlowFinder[1].FlowMask().copyTo(disp_img[1]);
      // disp_img[0]*= 200;
      // disp_img[1]*= 200;
      // Stereo.Rectify(disp_img[0],disp_img[1]);
      // FlowStereo(/*we*/2, /*wd*/3);
      stereo_f();

      VideoOut[0].Step(disp_img[0]);
      VideoOut[0].VizRec(disp_img[0]);
      VideoOut[1].Step(disp_img[1]);
      VideoOut[1].VizRec(disp_img[1]);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
      if(ImgWin[0]=='1')  cv::imshow("color_detector1", disp_img[0]);
      if(ImgWin[1]=='1')  cv::imshow("color_detector2", disp_img[1]);

      if(show_fps==0)
      {
        std::cerr<<"FPS: "<<VideoOut[0].FPS()<<", "<<VideoOut[1].FPS()<<std::endl;
        show_fps=VideoOut[0].FPS()*4;
      }
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }

    if(!HandleKeyEvent())  break;

    ros::spinOnce();
  }
  Shutdown= true;
  th_stereo.join();


  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
