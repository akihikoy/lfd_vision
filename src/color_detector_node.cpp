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
#include <std_msgs/Int32MultiArray.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
//-------------------------------------------------------------------------------------------
namespace trick
{
int NonzeroBase[]= {0,0,0,0,0,0,0};
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

static std::vector<std::vector<cv::Vec3b> >  detect_colors;
static cv::Vec3s  col_radius(-3,5,5);
static int  cd_idx(0);
static std::vector<TColorDetector>  col_detectors;
static bool running(true);
static cv::Vec3b  disp_colors[]= {
    cv::Vec3b(255,255,255),
    cv::Vec3b(255,0,0),
    cv::Vec3b(0,255,0),
    cv::Vec3b(0,0,255),
    cv::Vec3b(255,255,0),
    cv::Vec3b(255,0,255),
    cv::Vec3b(0,255,255)};

void OnMouseCamera(int event, int x, int y, int, void *vpimg)
{
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    detect_colors[cd_idx].clear();
    col_detectors[cd_idx].SetupColors(detect_colors[cd_idx], col_radius);
    NonzeroBase[cd_idx]= 0;
    return;
  }
  if(event == cv::EVENT_LBUTTONDBLCLK)
  {
    for(int i(0); i<col_detectors.size(); ++i)
      NonzeroBase[i]= 0;
    return;
  }

  if(event != cv::EVENT_LBUTTONDOWN)
    return;

  cv::Mat *pimg(reinterpret_cast<cv::Mat*>(vpimg));
  cv::Mat original(1,1,pimg->type()), converted;
  original.at<cv::Vec3b>(0,0)= pimg->at<cv::Vec3b>(y,x);  // WARNING: be careful about the order of y and x
  cv::cvtColor(original, converted, col_detectors[cd_idx].ColorCode());
  std::cout<< "BGR: "<<original.at<cv::Vec3b>(0,0)<<"  HSV: "<<converted.at<cv::Vec3b>(0,0)<<std::endl;
  detect_colors[cd_idx].push_back(converted.at<cv::Vec3b>(0,0));
  col_detectors[cd_idx].SetupColors(detect_colors[cd_idx], col_radius);
  NonzeroBase[cd_idx]= 0;
}
void OnMouseMask(int event, int x, int y, int, void*)
{
  if(event == cv::EVENT_LBUTTONDBLCLK)
  {
    for(int i(0); i<col_detectors.size(); ++i)
      NonzeroBase[i]= 0;
    return;
  }
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    running=!running;
    std::cerr<<(running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
}

void SaveColors(const char *file_suffix)
{
  std::string file_name= ColorFilesBase+std::string(file_suffix);
  std::cerr<<"###Save colors into "<<file_name<<std::endl;
  std::ofstream ofs(file_name.c_str());
  for(std::vector<cv::Vec3b>::const_iterator itr(detect_colors[cd_idx].begin()),last(detect_colors[cd_idx].end()); itr!=last; ++itr)
    ofs<<int((*itr)[0])<<" "<<int((*itr)[1])<<" "<<int((*itr)[2])<<std::endl;
}

void LoadColors(const char *file_suffix)
{
  std::string file_name= ColorFilesBase+std::string(file_suffix);
  detect_colors[cd_idx].clear();
  std::ifstream ifs(file_name.c_str());
  if(!ifs)  return;
  std::cerr<<"###Load colors from "<<file_name<<std::endl;
  std::string line;
  while(std::getline(ifs,line,'\n'))
  {
    std::stringstream ss(line);
    int ci[3];
    cv::Vec3b c;
    ss>>ci[0]>>ci[1]>>ci[2];
    c[0]=ci[0]; c[1]=ci[1]; c[2]=ci[2];
    detect_colors[cd_idx].push_back(c);
  }
  col_detectors[cd_idx].SetupColors(detect_colors[cd_idx], col_radius);
}

// Get median position of nonzero pixels
void GetMedian(const cv::Mat &src, int &x_med, int &y_med)
{
  assert(src.type()==CV_8UC1);
  int nonzero= cv::countNonZero(src);
  if(nonzero==0)  return;
  std::vector<int> arrayx(nonzero), arrayy(nonzero);
  int counter(0);
  for(int x(0); x<src.cols; ++x)
  {
    for(int y(0); y<src.rows; ++y)
    {
      if(src.at<unsigned char>(y,x) != 0)
      {
        arrayx[counter]= x;
        arrayy[counter]= y;
        ++counter;
      }
    }
  }
  std::sort(arrayx.begin(),arrayx.end());
  std::sort(arrayy.begin(),arrayy.end());
  x_med= arrayx[arrayx.size()/2];
  y_med= arrayy[arrayy.size()/2];
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle node("~");
  int camera(0);
  int mode(1);
  int num_detectors(1);
  node.param("camera",camera,0);
  node.param("mode",mode,2);
  node.param("num_detectors",num_detectors,1);
  node.param("color_files_base",ColorFilesBase,std::string(""));

  cv::VideoCapture cap(camera); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
  {
    std::cerr<<"Cannot open: "<<camera<<std::endl;
    return -1;
  }
  std::cerr<<"Camera opened"<<std::endl;

  col_detectors.resize(num_detectors);
  detect_colors.resize(num_detectors);
  for(int i(0); i<num_detectors; ++i)
  {
    cd_idx= i;
    LoadColors(DefaultFileNames[i]);
    col_detectors[i].SetupColors(detect_colors[i], col_radius);
  }
  cd_idx= 0;


  // ros::Publisher ratio_pub= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);
  std::vector<ros::Publisher> ratio_pubs(num_detectors);
  ratio_pubs[0]= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);
  if(num_detectors>=2)  ratio_pubs[1]= node.advertise<std_msgs::Float64>("/color_occupied_ratio2", 1);
  if(num_detectors>=3)  ratio_pubs[2]= node.advertise<std_msgs::Float64>("/color_occupied_ratio3", 1);
  if(num_detectors>=4)  ratio_pubs[3]= node.advertise<std_msgs::Float64>("/color_occupied_ratio4", 1);
  if(num_detectors>=5)  ratio_pubs[4]= node.advertise<std_msgs::Float64>("/color_occupied_ratio5", 1);
  if(num_detectors>=6)  ratio_pubs[5]= node.advertise<std_msgs::Float64>("/color_occupied_ratio6", 1);
  if(num_detectors>=7)  ratio_pubs[6]= node.advertise<std_msgs::Float64>("/color_occupied_ratio7", 1);

  std::vector<ros::Publisher> mxy_pubs(num_detectors);
  mxy_pubs[0]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy", 1);
  if(num_detectors>=2)  mxy_pubs[1]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy2", 1);
  if(num_detectors>=3)  mxy_pubs[2]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy3", 1);
  if(num_detectors>=4)  mxy_pubs[3]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy4", 1);
  if(num_detectors>=5)  mxy_pubs[4]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy5", 1);
  if(num_detectors>=6)  mxy_pubs[5]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy6", 1);
  if(num_detectors>=7)  mxy_pubs[6]= node.advertise<std_msgs::Int32MultiArray>("/color_middle_xy7", 1);

  std_msgs::Float64  ratio_msg;
  std_msgs::Int32MultiArray mxy_msg;


  cv::namedWindow("camera",1);
  // cv::namedWindow("detected",1);
  cv::namedWindow("mask_img",1);
  cv::Mat frame, detected;
  std::vector<cv::Mat>  mask_imgs(num_detectors);

  cv::Mat disp_imgs[3], disp_img;

  cv::setMouseCallback("camera", OnMouseCamera, &frame);
  cv::setMouseCallback("mask_img", OnMouseMask);

  // ros::Rate loop_rate(5);  // 5 Hz
  while(ros::ok())
  {
    if(running)
    {
      cap >> frame; // get a new frame from camera
      cv::imshow("camera", frame);

      if(num_detectors>=2)
      {
        disp_imgs[0]= cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
        disp_imgs[1]= cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
        disp_imgs[2]= cv::Mat::zeros(frame.rows, frame.cols, CV_8U);
        for(int i(0); i<num_detectors; ++i)
        {
          mask_imgs[i]= col_detectors[i].Detect(frame);
          disp_imgs[0]+= double(disp_colors[i][0])/255.0 * mask_imgs[i];
          disp_imgs[1]+= double(disp_colors[i][1])/255.0 * mask_imgs[i];
          disp_imgs[2]+= double(disp_colors[i][2])/255.0 * mask_imgs[i];
        }
        cv::merge(disp_imgs, 3, disp_img);
      }
      else
      {
        mask_imgs[0]= col_detectors[0].Detect(frame);
        disp_img= mask_imgs[0];
      }

      for(int i(0); i<num_detectors; ++i)
      {
        int nonzero= cv::countNonZero(mask_imgs[i]), diff(0);
        double ratio(0.0);
        switch(mode)
        {
        case 1:
          ratio= double(nonzero)/double(mask_imgs[i].total());
          std::cout<<"nonzero ("<<i<<"): \t"<<nonzero<<" / \t"<<mask_imgs[i].total()
              <<"  \t"<<ratio<<std::endl;
          break;
        case 2:
          if(NonzeroBase[i]==0)
          {
            NonzeroBase[i]= (nonzero>0 ? nonzero : 1);
            std::cerr<<"###Reset NonzeroBase: "<<NonzeroBase[i]<<std::endl;
          }
          diff= NonzeroBase[i]-nonzero;
          if(diff<0)  diff= 0;
          ratio= double(diff)/double(NonzeroBase[i]);
          std::cout<<"diff ("<<i<<"): \t"<<diff<<"  \t"<<ratio<<std::endl;
          break;
        default:
          std::cerr<<"Invalid mode:"<<mode<<std::endl;
        }

        int x_med(0), y_med(0);
        GetMedian(mask_imgs[i],x_med,y_med);

        ratio_msg.data= ratio;
        ratio_pubs[i].publish(ratio_msg);

        mxy_msg.data.resize(2);
        mxy_msg.data[0]= x_med;
        mxy_msg.data[1]= y_med;
        mxy_pubs[i].publish(mxy_msg);

        cv::circle(disp_img,cv::Point(x_med,y_med),3,0.5*cv::Scalar(disp_colors[i]));
      }

      cv::imshow("mask_img", disp_img);

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
      for(int i(0); i<num_detectors; ++i)
        NonzeroBase[i]= 0;
    }
    else if(c=='l')
    {
      LoadColors(DefaultFileNames[cd_idx]);
      NonzeroBase[cd_idx]= 0;
    }
    else if(c=='s')
    {
      SaveColors(DefaultFileNames[cd_idx]);
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
