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

inline float Dist(const cv::Point2f &p, const cv::Point2f &q)
{
  cv::Point2f d= p-q;
  return cv::sqrt(d.x*d.x + d.y*d.y);
}
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

inline long GetCurrentTimeL(void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return time.tv_sec*1e6l + time.tv_usec;
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

// Extract rows of src and store to dst (works for &dst==&src)
void ExtractRows(const cv::Mat &src, const cv::vector<int> &idx, cv::Mat &dst);
//-------------------------------------------------------------------------------------------

// Find the largest contour and return info. bin_src should be a binary image.
bool FindLargestContour(const cv::Mat &bin_src,
    double *area=NULL,
    cv::Point2d *center=NULL,
    cv::Rect *bound=NULL,
    std::vector<cv::Point> *contour=NULL);
//-------------------------------------------------------------------------------------------

// Rotate 90, 180, 270 degrees
void Rotate90N(const cv::Mat &src, cv::Mat &dst, int N);
//-------------------------------------------------------------------------------------------

// Project points 3D onto a rectified image.
void ProjectPointsToRectifiedImg(const cv::Mat &points3d, const cv::Mat &P, cv::Mat &points2d);
//-------------------------------------------------------------------------------------------

inline void DrawCrossOnCenter(cv::Mat &img, int size, const cv::Scalar &col, int thickness=1)
{
  int hsize(size/2);
  cv::line(img, cv::Point(img.cols/2-hsize,img.rows/2), cv::Point(img.cols/2+hsize,img.rows/2), col, thickness);
  cv::line(img, cv::Point(img.cols/2,img.rows/2-hsize), cv::Point(img.cols/2,img.rows/2+hsize), col, thickness);
}
//-------------------------------------------------------------------------------------------

bool OpenVideoOut(cv::VideoWriter &vout, const char *file_name, int fps, const cv::Size &size);
//-------------------------------------------------------------------------------------------

/* Easy video output tool that automatically measures FPS.
    Filename is automatically decided with a sequential index.

  Example:
    TEasyVideoOut vout;
    vout.SetfilePrefix("/tmp/vout");
    while(...)
    {
      ...
      vout.Step(frame);
      vout.VizRec(frame);
      // display frame here
      if(key=="W")  vout.Switch();
    }
*/
class TEasyVideoOut
{
public:
  TEasyVideoOut(const double init_fps=10.0);

  // Automatically switch Rec/Stop.
  void Switch()
    {
      if(!writer_.isOpened())  Rec();
      else  Stop();
    }

  // Start recording.
  void Rec();

  // Stop recording.
  void Stop();

  // Writing frame(during recording)/updating FPS,image size.
  void Step(const cv::Mat &frame);

  /* Visualize a recording mark (red circle), effective only during recording.
      pos: position (0: left top (default), 1: left bottom, 2: right bottom, 3: right top)
  */
  void VizRec(cv::Mat &frame, int pos=0, int rad=5, int margin=3) const;

  bool IsRecording() const {return writer_.isOpened();}
  const double& FPS() const {return fps_;}

  void SetfilePrefix(const std::string &v)  {file_prefix_= v;}
  void SetfileSuffix(const std::string &v)  {file_suffix_= v;}

private:
  std::string file_prefix_, file_suffix_;
  cv::VideoWriter writer_;
  cv::Size img_size_;
  double fps_;
  double time_prev_, fps_alpha_;
};
//-------------------------------------------------------------------------------------------

struct TCameraInfo
{
  int DevID;
  int Width, Height;
  std::string PixelFormat;
  int NRotate90;
  std::string Name;
};
//-------------------------------------------------------------------------------------------
void Print(const std::vector<TCameraInfo> &cam_info);
void WriteToYAML(const std::vector<TCameraInfo> &cam_info, const std::string &file_name);
void ReadFromYAML(std::vector<TCameraInfo> &cam_info, const std::string &file_name);
//-------------------------------------------------------------------------------------------

struct TStereoInfo
{
  std::string Name;
  int CamL, CamR;
  int Width, Height;
  std::string StereoParam;
  std::string LensType;
};
//-------------------------------------------------------------------------------------------
void Print(const std::vector<TStereoInfo> &cam_info);
void WriteToYAML(const std::vector<TStereoInfo> &cam_info, const std::string &file_name);
void ReadFromYAML(std::vector<TStereoInfo> &cam_info, const std::string &file_name);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // vision_util_h
//-------------------------------------------------------------------------------------------
