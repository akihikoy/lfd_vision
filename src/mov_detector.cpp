//-------------------------------------------------------------------------------------------
/*! \file    mov_detector.cpp
    \brief   Moving object detector using LK optical flow.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jul.14, 2015
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/mov_detector.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


TMovingObjectDetector::TMovingObjectDetector()
  :
    term_criteria_(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03),
    win_size_(10,10),
    max_feat_count_(500),
    reset_count_(100),
    min_moving_flow_(1.0),
    flow_range_gain_(5.0),
    max_flow_range_(20.0),
    count_(0)
{
}
//-------------------------------------------------------------------------------------------

void TMovingObjectDetector::Step(const cv::Mat &frame)
{
  cv::cvtColor(frame, gray_, CV_BGR2GRAY);
  if(prev_gray_.empty())  gray_.copyTo(prev_gray_);

  if(points_[0].empty() || count_==0)
  {
    // Automatically detect feature points
    cv::goodFeaturesToTrack(prev_gray_, points_[0], max_feat_count_, 0.01, 10, cv::Mat(), 3, 0, 0.04);
    cv::cornerSubPix(prev_gray_, points_[0], win_size_, cv::Size(-1,-1), term_criteria_);
    count_= reset_count_;
  }
  if(!points_[0].empty())
  {
    // Do tracking
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_gray_, gray_, points_[0], points_[1], status_, err, win_size_, 3, term_criteria_, 0);
    // Keep only tracked points and compute flow vector
    size_t k(0);
    flow_.resize(points_[1].size());
    for(size_t i(0),i_end(points_[1].size()); i<i_end; ++i)
    {
      if(!status_[i])  continue;  // Corresponding feature not found
      points_[1][k]= points_[1][i];
      flow_[k]= std::pair<cv::Point2f,bool>(points_[1][i] - points_[0][i], false);
      if(cv::norm(flow_[k].first)>=min_moving_flow_)
        flow_[k].second= true;
      ++k;
    }
    points_[1].resize(k);
    flow_.resize(k);
  }
  std::swap(points_[1], points_[0]);
  cv::swap(prev_gray_, gray_);

  --count_;
}
//-------------------------------------------------------------------------------------------

void TMovingObjectDetector::Draw(cv::Mat &img) const
{
  const std::vector<cv::Point2f> &points(points_[0]);
  if(points.empty() || flow_.empty())  return;
  for(size_t i(0),i_end(points.size()); i<i_end; ++i)
  {
    cv::circle(img, points[i], 3, cv::Scalar(0,0,255), 1, 8);
    if(!flow_[i].second)  continue;  // Not moving
    float f= cv::norm(flow_[i].first);
    // if(f>50.0) continue;  // Outliers
    f*= flow_range_gain_;
    if(f>max_flow_range_)  f= max_flow_range_;
    cv::circle(img, points[i], f, cv::Scalar(0,255,0), 1, 8);
    cv::line(img, points[i], points[i]+flow_[i].first, cv::Scalar(255,128,0), 2, 8);
  }
}
//-------------------------------------------------------------------------------------------

// Check if a point pt is moving.
bool TMovingObjectDetector::IsMoving(const cv::Point2f &pt) const
{
  const std::vector<cv::Point2f> &points(points_[0]);
  if(points.empty() || flow_.empty())  return false;
  for(size_t i(0),i_end(points.size()); i<i_end; ++i)
  {
    if(!flow_[i].second)  continue;  // Not moving
    float f= cv::norm(flow_[i].first);
    // if(f>50.0) continue;  // Outliers
    f*= flow_range_gain_;
    if(f>max_flow_range_)  f= max_flow_range_;
    if(cv::norm(pt - points[i]) < f)  return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

