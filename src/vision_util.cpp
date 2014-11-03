//-------------------------------------------------------------------------------------------
/*! \file    vision_util.cpp
    \brief   Basic vision utilities.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.29, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

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
//-------------------------------------------------------------------------------------------

bool OpenVideoOut(cv::VideoWriter &vout, const char *file_name, int fps, const cv::Size &size)
{
  int codec= CV_FOURCC('P','I','M','1');
  vout.open(file_name, codec, fps, size, true);

  if (!vout.isOpened())
  {
    std::cout<<"###Failed to open the output video: "<<file_name<<std::endl;
    return false;
  }
  std::cerr<<"###Opened video output: "<<file_name<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

// Rotate 90, 180, 270 degrees
void Rotate90N(const cv::Mat &src, cv::Mat &dst, int N)
{
  if(src.data!=dst.data)  src.copyTo(dst);
  for(int i((N%4+4)%4); i>0; --i)
  {
    cv::transpose(dst, dst);
    cv::flip(dst, dst, /*horizontal*/1);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

