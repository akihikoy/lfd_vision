//-------------------------------------------------------------------------------------------
/*! \file    vision_util.h
    \brief   Basic vision utilities.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.29, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef vision_util_h
#define vision_util_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <sys/time.h>  // gettimeofday
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

inline std::string ToString(const std::string &prefix, int num, const std::string &posix="")
{
  std::stringstream ss;
  ss<<prefix<<num<<posix;
  return ss.str();
}
//-------------------------------------------------------------------------------------------

inline double GetCurrentTime(void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
  // return ros::Time::now().toSec();
}
//-------------------------------------------------------------------------------------------

/*! \brief check the filename exists */
inline bool FileExists(const std::string &filename)
{
  bool res(false);
  std::ifstream ifs (filename.c_str());
  res = ifs.is_open();
  ifs.close();
  return res;
}
//-------------------------------------------------------------------------------------------

// Get median position of nonzero pixels
void GetMedian(const cv::Mat &src, int &x_med, int &y_med);
//-------------------------------------------------------------------------------------------

bool OpenVideoOut(cv::VideoWriter &vout, const char *file_name, int fps, const cv::Size &size);
//-------------------------------------------------------------------------------------------

// Rotate 90, 180, 270 degrees
void Rotate90N(const cv::Mat &src, cv::Mat &dst, int N);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // vision_util_h
//-------------------------------------------------------------------------------------------
