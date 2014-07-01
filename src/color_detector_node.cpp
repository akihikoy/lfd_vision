//-------------------------------------------------------------------------------------------
/*! \file    color_detector_node.cpp
    \brief   certain c++ source file
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Jun.05, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_trick/color_detector.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace trick
{
int NonzeroBase(0);
const char *DefaultFileName("default_colors.dat");
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

static std::vector<cv::Vec3b>  detect_colors;
static cv::Vec3s  col_radius(-3,5,5);
static TColorDetector  col_detector;
bool running(true);
void OnMouseCamera(int event, int x, int y, int, void *vpimg)
{
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    detect_colors.clear();
    col_detector.SetupColors(detect_colors, col_radius);
    NonzeroBase= 0;
    return;
  }
  if(event == cv::EVENT_LBUTTONDBLCLK)
  {
    NonzeroBase= 0;
    return;
  }

  if(event != cv::EVENT_LBUTTONDOWN)
    return;

  cv::Mat *pimg(reinterpret_cast<cv::Mat*>(vpimg));
  cv::Mat original(1,1,pimg->type()), converted;
  original.at<cv::Vec3b>(0,0)= pimg->at<cv::Vec3b>(y,x);  // WARNING: be careful about the order of y and x
  cv::cvtColor(original, converted, col_detector.ColorCode());
  std::cout<< "BGR: "<<original.at<cv::Vec3b>(0,0)<<"  HSV: "<<converted.at<cv::Vec3b>(0,0)<<std::endl;
  detect_colors.push_back(converted.at<cv::Vec3b>(0,0));
  col_detector.SetupColors(detect_colors, col_radius);
  NonzeroBase= 0;
}
void OnMouseMask(int event, int x, int y, int, void*)
{
  if(event == cv::EVENT_LBUTTONDBLCLK)
  {
    NonzeroBase= 0;
    return;
  }
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    running=!running;
    std::cerr<<(running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
}

void SaveColors(const char *file_name)
{
  std::cerr<<"Save colors into "<<file_name<<std::endl;
  std::ofstream ofs(file_name);
  for(std::vector<cv::Vec3b>::const_iterator itr(detect_colors.begin()),last(detect_colors.end()); itr!=last; ++itr)
    ofs<<int((*itr)[0])<<" "<<int((*itr)[1])<<" "<<int((*itr)[2])<<std::endl;
}

void LoadColors(const char *file_name)
{
  detect_colors.clear();
  std::ifstream ifs(file_name);
  if(!ifs)  return;
  std::cerr<<"Load colors from "<<file_name<<std::endl;
  std::string line;
  while(std::getline(ifs,line,'\n'))
  {
    std::stringstream ss(line);
    int ci[3];
    cv::Vec3b c;
    ss>>ci[0]>>ci[1]>>ci[2];
    c[0]=ci[0]; c[1]=ci[1]; c[2]=ci[2];
    detect_colors.push_back(c);
  }
  col_detector.SetupColors(detect_colors, col_radius);
}

int main(int argc, char**argv)
{
  LoadColors(DefaultFileName);

  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle node("~");
  int camera(0);
  int mode(1);
  node.param("camera",camera,0);
  node.param("mode",mode,1);

  ros::Publisher ratio_pub= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);

  cv::VideoCapture cap(camera); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
  {
    std::cerr<<"Cannot open: "<<camera<<std::endl;
    return -1;
  }
  std::cerr<<"Camera opened"<<std::endl;

  cv::namedWindow("camera",1);
  // cv::namedWindow("detected",1);
  cv::namedWindow("mask_img",1);
  cv::Mat frame, mask_img, detected;

  // std::vector<cv::Vec3b>  detect_colors;
  // detect_colors.push_back(cv::Vec3b(200,100,200));
  cv::setMouseCallback("camera", OnMouseCamera, &frame);
  cv::setMouseCallback("mask_img", OnMouseMask);

  // ros::Rate loop_rate(5);  // 5 Hz
  while(ros::ok())
  {
    if(running)
    {
      cap >> frame; // get a new frame from camera
      cv::imshow("camera", frame);
      mask_img= col_detector.Detect(frame);
      cv::imshow("mask_img", mask_img);

      int nonzero= cv::countNonZero(mask_img), diff(0);
      double ratio(0.0);
      switch(mode)
      {
      case 1:
        ratio= double(nonzero)/double(mask_img.total());
        std::cout<<"nonzero: \t"<<nonzero<<" / \t"<<mask_img.total()
            <<"  \t"<<ratio<<std::endl;
        break;
      case 2:
        if(NonzeroBase==0)
        {
          NonzeroBase= (nonzero>0 ? nonzero : 1);
          std::cerr<<"Reset NonzeroBase: "<<NonzeroBase<<std::endl;
        }
        diff= NonzeroBase-nonzero;
        if(diff<0)  diff= 0;
        ratio= double(diff)/double(NonzeroBase);
        std::cout<<"diff: \t"<<diff<<"  \t"<<ratio<<std::endl;
        break;
      default:
        std::cout<<"Invalid mode:"<<mode<<std::endl;
      }

      std_msgs::Float64  ratio_msg;
      ratio_msg.data= ratio;
      ratio_pub.publish(ratio_msg);

      // Apply the mask image
      // detected.create(frame.rows, frame.cols, CV_8UC3);
      // detected= cv::Scalar(0,0,0);
      // frame.copyTo(detected, mask_img);
      // cv::imshow("detected", detected);
    }
    else
    {
      usleep(200*1000);
    }
    int c(cv::waitKey(1));
    if(c=='\x1b'||c=='q') break;
    else if(c=='r')
    {
      NonzeroBase= 0;
      // cap >> frame;
      // mask_img= col_detector.Detect(frame);
      // int nonzero= cv::countNonZero(mask_img);
      // NonzeroBase= (nonzero>0 ? nonzero : 1);
    }
    else if(c=='l')
    {
      LoadColors(DefaultFileName);
      NonzeroBase= 0;

      // cap >> frame;
      // mask_img= col_detector.Detect(frame);
      // int nonzero= cv::countNonZero(mask_img);
      // NonzeroBase= (nonzero>0 ? nonzero : 1);
    }
    else if(c=='s')
    {
      SaveColors(DefaultFileName);
    }
    else if(c==' ')
    {
      running=!running;
      std::cerr<<(running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
    }

    ros::spinOnce();
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
