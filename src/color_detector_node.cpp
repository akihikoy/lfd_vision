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

static std::vector<cv::Vec3b>  detect_colors;
static cv::Vec3s  col_radius(-3,5,5);
static TColorDetector  col_detector;
void OnMouse(int event, int x, int y, int, void *vpimg)
{
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    detect_colors.clear();
    col_detector.SetupColors(detect_colors, col_radius);
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
}


int main(int argc, char**argv)
{
  ros::init(argc, argv, "minimum_node");
  ros::NodeHandle node;

  ros::Publisher ratio_pub= node.advertise<std_msgs::Float64>("color_occupied_ratio", 10);

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

  cv::namedWindow("camera",1);
  // cv::namedWindow("detected",1);
  cv::namedWindow("mask_img",1);
  cv::Mat frame, mask_img, detected;

  // std::vector<cv::Vec3b>  detect_colors;
  // detect_colors.push_back(cv::Vec3b(200,100,200));
  cv::setMouseCallback("camera", OnMouse, &frame);

  // ros::Rate loop_rate(10);  // 10 Hz
  while(ros::ok())
  {
    cap >> frame; // get a new frame from camera
    cv::imshow("camera", frame);
    mask_img= col_detector.Detect(frame);

    int nonzero= cv::countNonZero(mask_img);
    double nonzero_ratio= double(nonzero)/double(mask_img.total());
    std::cout<<"nonzero: \t"<<nonzero<<" / \t"<<mask_img.total()
        <<"  \t"<<nonzero_ratio<<std::endl;
    cv::imshow("mask_img", mask_img);

    std_msgs::Float64  ratio_msg;
    ratio_msg.data= nonzero_ratio;
    ratio_pub.publish(ratio_msg);

    // Apply the mask image
    // detected.create(frame.rows, frame.cols, CV_8UC3);
    // detected= cv::Scalar(0,0,0);
    // frame.copyTo(detected, mask_img);
    // cv::imshow("detected", detected);

    int c(cv::waitKey(1));
    if(c=='\x1b'||c=='q') break;

    ros::spinOnce();
    // loop_rate.sleep();
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
