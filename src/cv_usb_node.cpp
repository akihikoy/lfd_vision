//-------------------------------------------------------------------------------------------
/*! \file    cv_usb_node.cpp
    \brief   USB camera node for multiple cameras with OpenCV.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.08, 2016
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdio>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "cv_usb_node");
  ros::NodeHandle node("~");
  std::string config_file("config/usb_cams4g.yaml");

  if(argc>1)  config_file= argv[1];

  std::vector<TCameraInfo> cam_info;
  ReadFromYAML(cam_info, config_file);

  std::vector<cv::VideoCapture> cap(cam_info.size());
  for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
  {
    std::string &fourcc(cam_info[i_cam].PixelFormat);
    cap[i_cam].open(cam_info[i_cam].DevID);
    if(!cap[i_cam].isOpened())
    {
      std::cerr<<"Failed to open camera: "<<cam_info[i_cam].DevID<<std::endl;
      return -1;
    }
    cap[i_cam].set(CV_CAP_PROP_FOURCC,CV_FOURCC(fourcc[0],fourcc[1],fourcc[2],fourcc[3]));
    cap[i_cam].set(CV_CAP_PROP_FRAME_WIDTH, cam_info[i_cam].Width);
    cap[i_cam].set(CV_CAP_PROP_FRAME_HEIGHT, cam_info[i_cam].Height);
    cv::namedWindow(cam_info[i_cam].Name,1);
  }
  std::cerr<<"Opened camera(s)"<<std::endl;

  image_transport::ImageTransport imgtr(node);
  std::vector<image_transport::Publisher> pub;
  typedef boost::shared_ptr<camera_info_manager::CameraInfoManager> CamInfoMngrPtr;
  std::vector<CamInfoMngrPtr> info_manager;
  for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
  {
    pub.push_back(imgtr.advertise(cam_info[i_cam].Name+"/image_raw", 1));
    info_manager.push_back(CamInfoMngrPtr(new camera_info_manager::CameraInfoManager(ros::NodeHandle("~/"+cam_info[i_cam].Name), cam_info[i_cam].Name, /*camera_info_url=*/"")));
  }

  std::vector<cv::Mat> frame(cam_info.size());
  for(;ros::ok();)
  {
    // Capture from cameras:
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
      cap[i_cam] >> frame[i_cam];

    // Image processing:
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
      Rotate90N(frame[i_cam],frame[i_cam],cam_info[i_cam].NRotate90);

    // Publish images:
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
      pub[i_cam].publish( cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame[i_cam]).toImageMsg() );

    // Display images:
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
      cv::imshow(cam_info[i_cam].Name, frame[i_cam]);

    int c(cv::waitKey(10));
    if(c=='\x1b'||c=='q') break;
    // usleep(10000);
  }
  return 0;
}
//-------------------------------------------------------------------------------------------
