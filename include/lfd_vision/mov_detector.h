//-------------------------------------------------------------------------------------------
/*! \file    mov_detector.h
    \brief   Moving object detector using LK optical flow.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jul.14, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef mov_detector_h
#define mov_detector_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

class TMovingObjectDetector
{
public:
  TMovingObjectDetector();
  void Step(const cv::Mat &frame);
  void Draw(cv::Mat &img) const;

  // Check if a point pt is moving.
  bool IsMoving(const cv::Point2f &pt) const;

  void SetMaxFeatCount(int v)  {max_feat_count_= v;}
  void SetResetCount(int v)  {reset_count_= v;}

  void SetMinMovingFlow(const double &v)  {min_moving_flow_= v;}
  void SetFlowRangeGain(const double &v)  {flow_range_gain_= v;}
  void SetMaxFlowRange(const double &v)  {max_flow_range_= v;}

private:
  cv::TermCriteria term_criteria_;
  cv::Size win_size_;
  int max_feat_count_;
  int reset_count_;

  double min_moving_flow_;
  double flow_range_gain_;
  double max_flow_range_;

  cv::Mat gray_, prev_gray_;
  std::vector<cv::Point2f> points_[2];  // Feature points.
  std::vector<std::pair<cv::Point2f,bool> > flow_;  // Flow vector and if it's moving
  std::vector<uchar> status_;
  int count_;
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // mov_detector_h
//-------------------------------------------------------------------------------------------
