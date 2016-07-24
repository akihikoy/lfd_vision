//-------------------------------------------------------------------------------------------
/*! \file    flow_finder.h
    \brief   Detect flow using OpenCV.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.29, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef flow_finder_h
#define flow_finder_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <list>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

struct TFlowElement
{
  double X, Y;  // Flow center
  double VX, VY;  // Flow velocity (x,y)
  double Speed, Angle;  // Flow velocity (speed, angle)
  double Amount;  // Flow amount
  std::vector<cv::Point> *Contour;  // Contour of flow region
  TFlowElement() : Contour(NULL)  {}
};
std::ostream& operator<<(std::ostream &lhs, const TFlowElement &rhs);
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TFlowFinder
//===========================================================================================
{
public:
  TFlowFinder();

  void Update(const cv::Mat &frame);
  void UpdateProc1_FindFlow(cv::Mat &mask);
  void UpdateProc2_ContourAnalysis(const cv::Mat &mask);
  void DrawFlow(cv::Mat &frame, const cv::Scalar &color, const double &len=1.0, int thickness=3);

  void CalcOpticalFlow(const cv::Mat &prev, const cv::Mat &curr);
  // void GetAngleSpdImg(cv::Mat &img_angle, cv::Mat &img_spd);
  void GetSpdSqImg(cv::Mat &img_spd);

  // region of interest
  // set ROI?

  std::list<TFlowElement>& RefFlowElements()  {return flow_elmts_;}

  const std::list<TFlowElement>& FlowElements() const {return flow_elmts_;}
  const std::vector<std::vector<cv::Point> >& Contours() const {return contours_;}

  const cv::Mat& FlowMask() const {return flow_mask_;}

  // 0: Full, 1: FlowMask only
  void SetProcType(int pt)  {proc_type_= pt;}
  void SetOptFlowWinSize(const cv::Size &v)  {optflow_win_size_= v;}
  void SetOptFlowSpdThreshold(const double &v)  {optflow_spd_threshold_= v;}
  void SetErodeDilate(int n_itr)  {erode_dilate_= n_itr;}
  // Set range of flow-amount to be added (negative value does not affect)
  void SetAmountRange(const double &min=-1.0, const double &max=-1.0)
    {amount_min_=min; amount_max_=max;}
  // Set range of flow-speed to be added (negative value does not affect)
  void SetSpeedRange(const double &min=-1.0, const double &max=-1.0)
    {speed_min_=min; speed_max_=max;}
  // Filter length of FlowMask (0: no filter)
  void SetFlowMaskFilterLen(int l)  {flow_mask_filter_len_= l;}

  const int& FlowMaskFilterLen() const {return flow_mask_filter_len_;}

private:
  std::list<TFlowElement> flow_elmts_;

  // Parameters:
  int proc_type_;  // 0: Full, 1: FlowMask only
  cv::Size optflow_win_size_;
  double optflow_spd_threshold_;
  int    erode_dilate_;
  double amount_min_, amount_max_;
  double speed_min_, speed_max_;
  int flow_mask_filter_len_;  // Filter length of FlowMask (0: no filter)

  // Tmp variables:
  cv::Mat frame_, frame_old_;
  cv::Mat velx_, vely_;
  cv::Mat img_spd_/*, img_angle_*/;
  cv::Mat flow_mask_;  // Filtered flow-mask
  cv::Mat raw_flow_mask_;  // Non-filtered flow-mask

  // Filter for flow_mask_
  int idx_mask_;
  std::vector<cv::Mat> mask_queue_;

  std::vector<std::vector<cv::Point> > contours_;

  int rows_, cols_;

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
// Utility
//===========================================================================================

void CalcFlowAverage(const TFlowFinder &ff, cv::Vec2d &avr_xy, cv::Vec2d &avr_vel, cv::Vec2d &avr_spddir);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // flow_finder_h
//-------------------------------------------------------------------------------------------
