//-------------------------------------------------------------------------------------------
/*! \file    visual_skin_node.cpp
    \brief   Computer vision for visual skin.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.10, 2016
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/usb_stereo.h"
#include "lfd_vision/blob_tracker.h"
#include "lfd_vision/geom_util.h"
#include "lfd_vision/vision_util.h"
#include "lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include "lfd_vision/BlobMoves.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
bool Running(true), Shutdown(false), DoCalibrate(false);
std::string BlobCalibPrefix("blob_");
std::vector<TCameraInfo> CamInfo;
std::vector<TStereoInfo> StereoInfo;
std::vector<TStereo> Stereo;  // Standard stereo
std::vector<TStereo> StereoB;  // Stereo for blob
std::vector<TBlobTracker> BlobTracker;
std::vector<TCameraRectifier> SingleCamRectifier;
std::vector<boost::function<void(cv::Mat&)> > CamRectifier;  // Functions to rectify camera images.
void DummyRectify(cv::Mat&) {}  // Do nothing function

std::vector<TEasyVideoOut> VideoOut;

std::vector<ros::Publisher> CloudPub;
std::vector<ros::Publisher> BlobPub;
std::vector<cv::Mat> Frame;
std::vector<long>    CapTime;
std::vector<boost::shared_ptr<boost::mutex> > MutCamCapture;
struct TIMShowStuff
{
  boost::shared_ptr<boost::mutex> Mutex;
  cv::Mat Frame;
};
std::map<std::string, TIMShowStuff> IMShowStuff;
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
  else if(c=='W')
  {
    for(int j(0),j_end(VideoOut.size());j<j_end;++j)
      VideoOut[j].Switch();
  }
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  else if(c=='c')
  {
    DoCalibrate= true;
  }
  else if(c=='s')
  {
    for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
    {
      BlobTracker[j].SaveCalib(BlobCalibPrefix+CamInfo[j].Name+".yaml");
    }
  }
  else if(c=='l')
  {
    for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
    {
      if(FileExists(BlobCalibPrefix+CamInfo[j].Name+".yaml"))
        BlobTracker[j].LoadCalib(BlobCalibPrefix+CamInfo[j].Name+".yaml");
    }
  }
  // else if(c=='m' || c=='M')
  // {
    // if(c=='m')  ++VizMode[CameraIdx];  else  --VizMode[CameraIdx];
    // if(VizMode[CameraIdx]>4)  VizMode[CameraIdx]= 0;
    // if(VizMode[CameraIdx]<0)  VizMode[CameraIdx]= 4;
    // std::cerr<<"VizMode["<<CameraIdx<<"]: "<<VizMode[CameraIdx]<<std::endl;
  // }

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
void ExecStereo(int i_stereo)
{
  TStereo &stereo(Stereo[i_stereo]);
  TStereoInfo &info(StereoInfo[i_stereo]);
  cv::Mat frame[2];
  cv::Mat disparity;
  while(!Shutdown)
  {
    if(Running)
    {
      {
        boost::mutex::scoped_lock lock1(*MutCamCapture[info.CamL]);
        boost::mutex::scoped_lock lock2(*MutCamCapture[info.CamR]);
        Frame[info.CamL].copyTo(frame[0]);
        Frame[info.CamR].copyTo(frame[1]);
      }
      stereo.Proc(frame[0],frame[1]);
      cv::normalize(stereo.Disparity(), disparity, 0, 255, CV_MINMAX, CV_8U);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
      // if(ImgWin[2]=='1')  cv::imshow("stereo_frame_l", stereo.FrameL());
      // if(ImgWin[3]=='1')  cv::imshow("stereo_frame_r", stereo.FrameR());
      // if(ImgWin[4]=='1')  cv::imshow("stereo_disparity", disparity);
      {
        // cv::imshow(info.Name, disparity);
        boost::mutex::scoped_lock lock(*IMShowStuff[info.Name].Mutex);
        disparity.copyTo(IMShowStuff[info.Name].Frame);
      }

      // Publish as point cloud.
      stereo.ReprojectTo3D();
      sensor_msgs::PointCloud2 cloud_msg;
      ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(cloud_msg,
          ConvertXYZImageToPointCloud(stereo.XYZ(),stereo.RGB()),
          /*frame_id=*/CamInfo[info.CamL].Name);
      CloudPub[i_stereo].publish(cloud_msg);
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ExecBlobTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TBlobTracker &tracker(BlobTracker[i_cam]);
  cv::Mat frame;
  long t_cap(0);
  while(!Shutdown)
  {
    if(Running)
    {
      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }

      {
        boost::mutex::scoped_lock lock(*MutCamCapture[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      CamRectifier[i_cam](frame);
      tracker.Step(frame);
      tracker.Draw(frame);

      VideoOut[i_cam].Step(frame);
      VideoOut[i_cam].VizRec(frame);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
      // if(ImgWin[0]=='1')
      {
        // cv::imshow(info.Name, frame);
        boost::mutex::scoped_lock lock(*IMShowStuff[info.Name].Mutex);
        frame.copyTo(IMShowStuff[info.Name].Frame);
      }

      // Publish as BlobMoves
      {
        const std::vector<TPointMove> &data(tracker.Data());
        lfd_vision::BlobMoves blob_moves;
        blob_moves.camera_index= i_cam;
        blob_moves.camera_name= info.Name;
        blob_moves.width= info.Width;
        blob_moves.height= info.Height;
        blob_moves.data.resize(data.size());
        int i(0);
        for(std::vector<TPointMove>::const_iterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr,++i)
        {
          lfd_vision::BlobMove &m(blob_moves.data[i]);
          m.Pox= itr->Po.x;
          m.Poy= itr->Po.y;
          m.So = itr->So;
          m.DPx= itr->DP.x;
          m.DPy= itr->DP.y;
          m.DS = itr->DS;
        }
        BlobPub[i_cam].publish(blob_moves);
      }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  ros::init(argc, argv, "visual_skin_node");
  ros::NodeHandle node("~");
  std::string pkg_dir(".");
  std::string cam_config("config/usbcam4g1.yaml");
  std::string stereo_config("config/usbcam4g1.yaml");
  std::string blobtrack_config("config/usbcam4g1.yaml");
  std::string blob_calib_prefix("blob_");
  std::string vout_base("/tmp/vout");

  node.param("pkg_dir",pkg_dir,pkg_dir);
  node.param("cam_config",cam_config,cam_config);
  node.param("stereo_config",stereo_config,stereo_config);
  node.param("blobtrack_config",blobtrack_config,blobtrack_config);
  node.param("blob_calib_prefix",blob_calib_prefix,blob_calib_prefix);
  node.param("vout_base",vout_base,vout_base);
  std::cerr<<"pkg_dir: "<<pkg_dir<<std::endl;
  std::cerr<<"cam_config: "<<cam_config<<std::endl;
  std::cerr<<"stereo_config: "<<stereo_config<<std::endl;
  std::cerr<<"blobtrack_config: "<<blobtrack_config<<std::endl;
  std::cerr<<"blob_calib_prefix: "<<blob_calib_prefix<<std::endl;

  std::vector<TBlobTrackerParams> blobtrack_info;
  ReadFromYAML(CamInfo, pkg_dir+"/"+cam_config);
  ReadFromYAML(StereoInfo, pkg_dir+"/"+stereo_config);
  ReadFromYAML(blobtrack_info, pkg_dir+"/"+blobtrack_config);
  BlobCalibPrefix= pkg_dir+"/"+blob_calib_prefix;

  std::vector<cv::VideoCapture> cap(CamInfo.size());
  SingleCamRectifier.resize(CamInfo.size());
  CamRectifier.resize(CamInfo.size());
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    const TCameraInfo &info(CamInfo[i_cam]);
    const std::string &fourcc(info.PixelFormat);
    cap[i_cam].open(info.DevID); cap[i_cam].release();  // DEBUG:TEST
    cap[i_cam].open(info.DevID);
    if(!cap[i_cam].isOpened())
    {
      std::cerr<<"Failed to open camera: "<<info.DevID<<std::endl;
      return -1;
    }
    if(fourcc.size()>0)  cap[i_cam].set(CV_CAP_PROP_FOURCC,CV_FOURCC(fourcc[0],fourcc[1],fourcc[2],fourcc[3]));
    cap[i_cam].set(CV_CAP_PROP_FRAME_WIDTH, info.Width);
    cap[i_cam].set(CV_CAP_PROP_FRAME_HEIGHT, info.Height);
    MutCamCapture.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));

    if(info.Rectification)
    {
      // Setup rectification
      // NOTE: The rectification of StereoInfo overwrites this rectification.
      cv::Size size_in(info.Width,info.Height), size_out(info.Width,info.Height);
      SingleCamRectifier[i_cam].Setup(info.K, info.D, info.R, size_in, info.Alpha, size_out);
      CamRectifier[i_cam]= boost::bind(&TCameraRectifier::Rectify, SingleCamRectifier[i_cam], _1);
    }
    else
    {
      CamRectifier[i_cam]= &DummyRectify;
    }
  }
  std::cerr<<"Opened camera(s)"<<std::endl;

  VideoOut.resize(CamInfo.size());
  for(int j(0),j_end(VideoOut.size());j<j_end;++j)
    VideoOut[j].SetfilePrefix(vout_base);

  Stereo.resize(StereoInfo.size());
  StereoB.resize(StereoInfo.size());
  for(int j(0),j_end(Stereo.size());j<j_end;++j)
  {
    const TStereoInfo &info(StereoInfo[j]);
    Stereo[j].LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    Stereo[j].SetImageSize(
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height),
        cv::Size(info.Width,info.Height) );
    Stereo[j].SetRecommendedStereoParams();
    Stereo[j].LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    Stereo[j].Init();
    cv::namedWindow(info.Name,1);
    cv::setMouseCallback(info.Name, OnMouseSimple);
    IMShowStuff[info.Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);

    StereoB[j].LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    StereoB[j].SetImageSize(
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height),
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height) );
    StereoB[j].SetRecommendedStereoParams();
    StereoB[j].LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    StereoB[j].Init();
    CamRectifier[info.CamL]= boost::bind(&TStereo::RectifyL, StereoB[j], _1, /*gray_scale=*/false);
    CamRectifier[info.CamR]= boost::bind(&TStereo::RectifyR, StereoB[j], _1, /*gray_scale=*/false);
  }

  BlobTracker.resize(CamInfo.size());
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    BlobTracker[j].Params()= blobtrack_info[j];
    BlobTracker[j].Init();
    if(FileExists(BlobCalibPrefix+CamInfo[j].Name+".yaml"))
      BlobTracker[j].LoadCalib(BlobCalibPrefix+CamInfo[j].Name+".yaml");
    cv::namedWindow(CamInfo[j].Name,1);
    cv::setMouseCallback(CamInfo[j].Name, OnMouseSimple);
    IMShowStuff[CamInfo[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
  }

  CloudPub.resize(Stereo.size());
  for(int j(0),j_end(Stereo.size());j<j_end;++j)
    CloudPub[j]= node.advertise<sensor_msgs::PointCloud2>(std::string("point_cloud_")+StereoInfo[j].Name, 1);

  BlobPub.resize(BlobTracker.size());
  for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
    BlobPub[j]= node.advertise<lfd_vision::BlobMoves>(std::string("blob_moves_")+CamInfo[j].Name, 1);

  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);

  Frame.resize(CamInfo.size());
  CapTime.resize(CamInfo.size());


  int show_fps(0);

  // Dummy capture.
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    cap[i_cam] >> Frame[i_cam];
    CapTime[i_cam]= GetCurrentTimeL();
  }

  std::vector<boost::shared_ptr<boost::thread> > th_blobtrack;
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    th_blobtrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecBlobTrack,j))));

  std::vector<boost::shared_ptr<boost::thread> > th_stereo;
  for(int j(0),j_end(StereoInfo.size());j<j_end;++j)
    th_stereo.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecStereo,j))));

  // ros::Rate loop_rate(5);  // 5 Hz
  for(int f(0);ros::ok();++f)
  {
    if(Running)
    {
      // Capture from cameras:
      for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
      {
        boost::mutex::scoped_lock lock(*MutCamCapture[i_cam]);
        cap[i_cam] >> Frame[i_cam];
        Rotate90N(Frame[i_cam],Frame[i_cam],CamInfo[i_cam].NRotate90);
        CapTime[i_cam]= GetCurrentTimeL();
      }

      // Show windows
      for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
      {
        boost::mutex::scoped_lock lock(*itr->second.Mutex);
        if(itr->second.Frame.total()>0)
          cv::imshow(itr->first, itr->second.Frame);
      }

      // Handle calibration request
      if(DoCalibrate && BlobTracker.size()>0)
      {
        Running= false;
        std::vector<std::vector<cv::Mat> > frames(CamInfo.size());
        cv::Mat frame;
        for(int i(0); i<BlobTracker[0].Params().NCalibPoints; ++i)
        {
          for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
          {
            boost::mutex::scoped_lock lock(*MutCamCapture[i_cam]);
            cap[i_cam] >> frame;
            Rotate90N(frame,frame,CamInfo[i_cam].NRotate90);
            CamRectifier[i_cam](frame);
            frames[i_cam].push_back(frame.clone());
          }
        }
        for(int j(0),j_end(BlobTracker.size()); j<j_end; ++j)
          BlobTracker[j].Calibrate(frames[j]);
        Running= true;
        DoCalibrate= false;
      }

      // usleep(10*1000);
      if(show_fps==0)
      {
        std::cerr<<"FPS: "<<VideoOut[0].FPS()<<std::endl;
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
  for(int j(0),j_end(th_blobtrack.size());j<j_end;++j)
    th_blobtrack[j]->join();
  for(int j(0),j_end(th_stereo.size());j<j_end;++j)
    th_stereo[j]->join();

  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
