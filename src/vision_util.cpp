//-------------------------------------------------------------------------------------------
/*! \file    vision_util.cpp
    \brief   Basic vision utilities.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.29, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iomanip>
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

// Extract rows of src and store to dst (works for &dst==&src)
void ExtractRows(const cv::Mat &src, const cv::vector<int> &idx, cv::Mat &dst)
{
  cv::Mat buf(src);
  dst.create(idx.size(),buf.cols,buf.type());
  int r(0);
  for(cv::vector<int>::const_iterator itr(idx.begin()),itr_end(idx.end()); itr!=itr_end; ++itr,++r)
    buf.row(*itr).copyTo(dst.row(r));
}
//-------------------------------------------------------------------------------------------

// Find the largest contour and return info. bin_src should be a binary image.
// WARNING: bin_src is modified.
bool FindLargestContour(const cv::Mat &bin_src,
    double *area,
    cv::Point2d *center,
    cv::Rect *bound,
    std::vector<cv::Point> *contour)
{
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(bin_src,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  if(contours.size()==0)  return false;
  double a(0.0),a_max(0.0), i_max(0);
  for(int i(0),i_end(contours.size()); i<i_end; ++i)
  {
    a= cv::contourArea(contours[i],false);
    if(a>a_max)  {a_max= a;  i_max= i;}
  }
  std::vector<cv::Point> &cnt(contours[i_max]);
  if(area!=NULL)
    *area= a_max;
  if(center!=NULL)
  {
    cv::Moments mu= cv::moments(cnt);
    *center= cv::Point2d(mu.m10/mu.m00, mu.m01/mu.m00);
  }
  if(bound!=NULL)
    *bound= cv::boundingRect(cnt);
  if(contour!=NULL)
    *contour= cnt;
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

// Project points 3D onto a rectified image.
void ProjectPointsToRectifiedImg(const cv::Mat &points3d, const cv::Mat &P, cv::Mat &points2d)
{
  cv::Mat P2;
  P.convertTo(P2,points3d.type());
  // cv::Mat points3dh, points2dh;
  // cv::convertPointsToHomogeneous(points3d, points3dh);
  // points2dh= points3dh*P2.t();
  cv::Mat points2dh= points3d*P2(cv::Range(0,3),cv::Range(0,3)).t();
  cv::Mat p3= P2.col(3).t();
  for(int r(0),rows(points2dh.rows);r<rows;++r)
    points2dh.row(r)+= p3;
  cv::convertPointsFromHomogeneous(points2dh, points2d);
}
//-------------------------------------------------------------------------------------------

bool OpenVideoOut(cv::VideoWriter &vout, const char *file_name, int fps, const cv::Size &size)
{
  // int codec= CV_FOURCC('P','I','M','1');  // mpeg1video
  // int codec= CV_FOURCC('X','2','6','4');  // x264?
  int codec= CV_FOURCC('m','p','4','v');  // mpeg4 (Simple Profile)
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

TEasyVideoOut::TEasyVideoOut(const double init_fps)
  :
    file_prefix_ ("/tmp/video"),
    file_suffix_ (".avi"),
    img_size_ (0,0),
    fps_ (init_fps),
    time_prev_ (-1.0),
    fps_alpha_ (0.05)
{
}
//-------------------------------------------------------------------------------------------

// Start recording.
void TEasyVideoOut::Rec()
{
  if(!writer_.isOpened())
  {
    int i(0);
    std::string file_name;
    do
    {
      std::stringstream ss;
      ss<<file_prefix_<<std::setfill('0')<<std::setw(4)<<i<<file_suffix_;
      file_name= ss.str();
      ++i;
    } while(FileExists(file_name));
    OpenVideoOut(writer_, file_name.c_str(), fps_, img_size_);
  }
}
//-------------------------------------------------------------------------------------------

// Stop recording.
void TEasyVideoOut::Stop()
{
  if(writer_.isOpened())
  {
    writer_.release();
    std::cerr<<"###Finished: video output"<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

// Writing frame(during recording)/updating FPS,image size
void TEasyVideoOut::Step(const cv::Mat &frame)
{
  img_size_= cv::Size(frame.cols, frame.rows);

  // update fps
  if(time_prev_<0.0)
  {
    time_prev_= GetCurrentTime();
  }
  else
  {
    double new_fps= 1.0/(GetCurrentTime()-time_prev_);
    if(new_fps>fps_/20.0 && new_fps<fps_*20.0)  // Removing outliers (e.g. pause/resume)
      fps_= fps_alpha_*new_fps + (1.0-fps_alpha_)*fps_;
    time_prev_= GetCurrentTime();
  }

  if(writer_.isOpened())
  {
    if(frame.depth()==CV_8U && frame.channels()==3)
      writer_<<frame;
    else
    {
      // If frame is [0...1] float type matrix:
      cv::Mat frame2,frame3;
      if(frame.depth()!=CV_8U)
        cv::Mat(frame*255.0).convertTo(frame2, CV_8UC(frame.channels()));
      else
        frame2= frame;
      if(frame2.channels()!=3)
      {
        cv::Mat in[]= {frame2, frame2, frame2};
        cv::merge(in, 3, frame3);
      }
      else
        frame3= frame2;
      writer_<<frame3;
    }
  }
}
//-------------------------------------------------------------------------------------------

/* Visualize a recording mark (red circle), effective only during recording.
    pos: position (0: left top (default), 1: left bottom, 2: right bottom, 3: right top)
*/
void TEasyVideoOut::VizRec(cv::Mat &frame, int pos, int rad, int margin) const
{
  if(writer_.isOpened())
  {
    cv::Point2d pt((rad+margin), (rad+margin));
    switch(pos)
    {
    case 0:  pt= cv::Point2d((rad+margin),(rad+margin)); break;
    case 1:  pt= cv::Point2d((rad+margin),frame.rows-(rad+margin)); break;
    case 2:  pt= cv::Point2d(frame.cols-(rad+margin),frame.rows-(rad+margin)); break;
    case 3:  pt= cv::Point2d(frame.cols-(rad+margin),(rad+margin)); break;
    }
    cv::circle(frame, pt, rad, cv::Scalar(0,0,255), -1);
  }
}
//-------------------------------------------------------------------------------------------


void Print(const std::vector<TCameraInfo> &cam_info)
{
  int i(0);
  for(std::vector<TCameraInfo>::const_iterator itr(cam_info.begin()),itr_end(cam_info.end()); itr!=itr_end; ++itr,++i)
  {
    std::cout<<"No. "<<i<<std::endl;
    #define PROC_VAR(x)  std::cout<<"  "#x": "<<itr->x<<std::endl;
    PROC_VAR(DevID       );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(PixelFormat );
    PROC_VAR(NRotate90   );
    PROC_VAR(Name        );
    #undef PROC_VAR
  }
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TCameraInfo> &cam_info, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"CameraInfo"<<"[";
  for(std::vector<TCameraInfo>::const_iterator itr(cam_info.begin()),itr_end(cam_info.end()); itr!=itr_end; ++itr)
  {
    #define PROC_VAR(x)  fs<<#x<<itr->x;
    fs<<"{";
    PROC_VAR(DevID       );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(PixelFormat );
    PROC_VAR(NRotate90   );
    PROC_VAR(Name        );
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TCameraInfo> &cam_info, const std::string &file_name)
{
  cam_info.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["CameraInfo"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TCameraInfo cf;
    #define PROC_VAR(x)  (*itr)[#x]>>cf.x;
    PROC_VAR(DevID       );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(PixelFormat );
    PROC_VAR(NRotate90   );
    PROC_VAR(Name        );
    #undef PROC_VAR
    cam_info.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


void Print(const std::vector<TStereoInfo> &stereo_info)
{
  int i(0);
  for(std::vector<TStereoInfo>::const_iterator itr(stereo_info.begin()),itr_end(stereo_info.end()); itr!=itr_end; ++itr,++i)
  {
    std::cout<<"No. "<<i<<std::endl;
    #define PROC_VAR(x)  std::cout<<"  "#x": "<<itr->x<<std::endl;
    PROC_VAR(Name        );
    PROC_VAR(Name        );
    PROC_VAR(CamL        );
    PROC_VAR(CamR        );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(StereoParam );
    PROC_VAR(LensType    );
    #undef PROC_VAR
  }
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TStereoInfo> &stereo_info, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"StereoInfo"<<"[";
  for(std::vector<TStereoInfo>::const_iterator itr(stereo_info.begin()),itr_end(stereo_info.end()); itr!=itr_end; ++itr)
  {
    #define PROC_VAR(x)  fs<<#x<<itr->x;
    fs<<"{";
    PROC_VAR(Name        );
    PROC_VAR(Name        );
    PROC_VAR(CamL        );
    PROC_VAR(CamR        );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(StereoParam );
    PROC_VAR(LensType    );
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TStereoInfo> &stereo_info, const std::string &file_name)
{
  stereo_info.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["StereoInfo"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TStereoInfo cf;
    #define PROC_VAR(x)  (*itr)[#x]>>cf.x;
    PROC_VAR(Name        );
    PROC_VAR(Name        );
    PROC_VAR(CamL        );
    PROC_VAR(CamR        );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(StereoParam );
    PROC_VAR(LensType    );
    #undef PROC_VAR
    stereo_info.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

