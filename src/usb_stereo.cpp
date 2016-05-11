//-------------------------------------------------------------------------------------------
/*! \file    usb_stereo.cpp
    \brief   USB stereo processing.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Apr.04, 2016
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/usb_stereo.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

//===========================================================================================
// Basic stereo class.
// class TStereo
//===========================================================================================

// Load camera parameters from a YAML file.
bool TStereo::LoadCameraParametersFromYAML(const std::string &file_name)
{
  //From stereo camera calibration:
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
    std::cerr<<"Failed to open file"<<std::endl;
    return false;
  }
  fs["D1"] >> cp_.D1;
  fs["K1"] >> cp_.K1;
  fs["P1"] >> cp_.P1;
  fs["R1"] >> cp_.R1;
  fs["D2"] >> cp_.D2;
  fs["K2"] >> cp_.K2;
  fs["P2"] >> cp_.P2;
  fs["R2"] >> cp_.R2;
  fs["R"] >> cp_.R;
  fs["T"] >> cp_.T;
  return true;
}
//-------------------------------------------------------------------------------------------

// img_size_in_,img_size_out_ must be set before using this.
void TStereo::SetRecommendedStereoParams()
{
  int n_disp= (((img_size_out_.width/8) + 15) & -16)+16*4;
  int min_disp= std::max(0, n_disp-16*8);
  int w_size= 5;

  sp_.preset                = cv::StereoBM::BASIC_PRESET;
  sp_.minDisparity          = min_disp;
  sp_.numberOfDisparities   = n_disp;
  sp_.SADWindowSize         = w_size;
  sp_.preFilterCap          = 0;
  sp_.uniquenessRatio       = 0;
  sp_.P1                    = 8*3*w_size*w_size;
  sp_.P2                    = 32*3*w_size*w_size;
  sp_.speckleWindowSize     = 0;
  sp_.speckleRange          = 0;
  sp_.disp12MaxDiff         = 0;
  sp_.fullDP                = false;

  sp_.StereoMethod          = TStereoParams::smSGBM;
  sp_.GrayScale             = true;
  sp_.LensType              = TStereoParams::ltBasic;
}
//-------------------------------------------------------------------------------------------

// Initialize stereo operation.
void TStereo::Init()
{
  // Although we have the most of camera parameters, run stereoRectify to get Q.
  if(sp_.LensType==TStereoParams::ltBasic)
  {
    cv::stereoRectify(cp_.K1, cp_.D1, cp_.K2, cp_.D2, img_size_in_, cp_.R, cp_.T.t(),
        cp_.R1, cp_.R2, cp_.P1, cp_.P2, cp_.Q,
        /*flags=*/cv::CALIB_ZERO_DISPARITY, /*alpha=*/0.0,
        img_size_out_/*, Rect* validPixROI1=0, Rect* validPixROI2=0*/);
        // FIXME: alpha is important to be tuned!!!
    std::cerr<<"cp_.Q="<<cp_.Q<<std::endl;
    cv::initUndistortRectifyMap(cp_.K1, cp_.D1, cp_.R1, cp_.P1, img_size_out_, CV_16SC2, map11_, map12_);
    cv::initUndistortRectifyMap(cp_.K2, cp_.D2, cp_.R2, cp_.P2, img_size_out_, CV_16SC2, map21_, map22_);
  }
  else if(sp_.LensType==TStereoParams::ltFisheye)
  {
    cv::fisheye::stereoRectify(cp_.K1, cp_.D1, cp_.K2, cp_.D2, img_size_in_, cp_.R, cp_.T.t(),
        cp_.R1, cp_.R2, cp_.P1, cp_.P2, cp_.Q,
        /*flags=*/cv::CALIB_ZERO_DISPARITY, img_size_out_,
        /*balance=*/0.0, /*fov_scale=*/0.55);
        // FIXME: balance, fov_scale are important to be tuned!!!
    std::cerr<<"cp_.Q="<<cp_.Q<<std::endl;
    cv::fisheye::initUndistortRectifyMap(cp_.K1, cp_.D1, cp_.R1, cp_.P1, img_size_out_, CV_16SC2, map11_, map12_);
    cv::fisheye::initUndistortRectifyMap(cp_.K2, cp_.D2, cp_.R2, cp_.P2, img_size_out_, CV_16SC2, map21_, map22_);
  }

  if(sp_.StereoMethod==TStereoParams::smBM)
  {
    stereo_bm_.init(sp_.preset, sp_.numberOfDisparities, sp_.SADWindowSize);
  }
  else if(sp_.StereoMethod==TStereoParams::smSGBM)
  {
    stereo_sgbm_.minDisparity          = sp_.minDisparity          ;
    stereo_sgbm_.numberOfDisparities   = sp_.numberOfDisparities   ;
    stereo_sgbm_.SADWindowSize         = sp_.SADWindowSize         ;
    stereo_sgbm_.preFilterCap          = sp_.preFilterCap          ;
    stereo_sgbm_.uniquenessRatio       = sp_.uniquenessRatio       ;
    stereo_sgbm_.P1                    = sp_.P1                    ;
    stereo_sgbm_.P2                    = sp_.P2                    ;
    stereo_sgbm_.speckleWindowSize     = sp_.speckleWindowSize     ;
    stereo_sgbm_.speckleRange          = sp_.speckleRange          ;
    stereo_sgbm_.disp12MaxDiff         = sp_.disp12MaxDiff         ;
    stereo_sgbm_.fullDP                = sp_.fullDP                ;
  }
}
//-------------------------------------------------------------------------------------------

// Process stereo operation from left and right images.
void TStereo::Proc(const cv::Mat &frame_l, const cv::Mat &frame_r)
{
  frame_l.copyTo(rgb_);
  frame_l.copyTo(frame1_);
  frame_r.copyTo(frame2_);

  Rectify(frame1_, frame2_, sp_.GrayScale);

  // TODO:FIXME: REDUCE THE IMAGE SIZES...?

  if(sp_.StereoMethod==TStereoParams::smBM)
    stereo_bm_(frame1_, frame2_, disparity_);
  else if(sp_.StereoMethod==TStereoParams::smSGBM)
    stereo_sgbm_(frame1_, frame2_, disparity_);

  // cv::filterSpeckles(disparity_, /*newVal=*/0, /*maxSpeckleSize=*/10, /*maxDiff=*/16, buf_);
}
//-------------------------------------------------------------------------------------------

// Rectify images.
void TStereo::Rectify(cv::Mat &frame1, cv::Mat &frame2, bool gray_scale)
{
  RectifyL(frame1, gray_scale);
  RectifyR(frame2, gray_scale);
}
//-------------------------------------------------------------------------------------------

// Rectify left(1) image.
void TStereo::RectifyL(cv::Mat &frame1, bool gray_scale)
{
  if(gray_scale)
  {
    cv::Mat gray1;
    if(frame1.channels()==3)
    {
      cv::cvtColor(frame1, gray1, CV_BGR2GRAY);
      frame1= gray1;
    }
  }

  cv::Mat frame1r;
  cv::remap(frame1, frame1r, map11_, map12_, cv::INTER_LINEAR);
  frame1= frame1r;
}
//-------------------------------------------------------------------------------------------

// Rectify right(2) image.
void TStereo::RectifyR(cv::Mat &frame2, bool gray_scale)
{
  if(gray_scale)
  {
    cv::Mat gray2;
    if(frame2.channels()==3)
    {
      cv::cvtColor(frame2, gray2, CV_BGR2GRAY);
      frame2= gray2;
    }
  }

  cv::Mat frame2r;
  cv::remap(frame2, frame2r, map21_, map22_, cv::INTER_LINEAR);
  frame2= frame2r;
}
//-------------------------------------------------------------------------------------------


// Reproject disparity to 3D. Use after Proc.
void TStereo::ReprojectTo3D()
{
  disparity_.convertTo(disparity_f32_, CV_32F, 1.0/16.0);
  cv::reprojectImageTo3D(disparity_f32_, xyz_, cp_.Q/*, handleMissingValues=false, int ddepth=-1 */);
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

