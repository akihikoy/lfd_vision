//-------------------------------------------------------------------------------------------
/*! \file    flow_finder.cpp
    \brief   certain c++ unit file
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.29, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/flow_finder.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

std::ostream& operator<<(std::ostream &lhs, const TFlowElement &rhs)
{
  lhs<<"["<<rhs.X<<", "<<rhs.Y<<"],"
      " d["<<rhs.VX<<", "<<rhs.VY<<"], "<<rhs.Amount<<","
      " ("<<(rhs.Contour!=NULL?rhs.Contour->size():0)<<" pts)";
  return lhs;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TFlowFinder
//===========================================================================================

TFlowFinder::TFlowFinder()
  :
    optflow_win_size_(3,3),
    optflow_spd_threshold_(5.0),
    erode_dilate_(1),
    amount_min_(3.0),
    amount_max_(-1.0),
    speed_min_(-1.0),
    speed_max_(-1.0)
{
}
//-------------------------------------------------------------------------------------------

void TFlowFinder::Update(const cv::Mat &frame)
{
  if(frame.channels()==3)
    cv::cvtColor(frame,frame_,CV_BGR2GRAY);
  else
    frame.convertTo(frame_,CV_8UC1);

  UpdateProc1_FindFlow();
  UpdateProc2_ContourAnalysis(mask_);
}
//-------------------------------------------------------------------------------------------

void TFlowFinder::UpdateProc1_FindFlow()
{
  if(frame_old_.size()!=frame_.size())  frame_old_= frame_.clone();

  // CalcOpticalFlow(frame_old_, frame_);
  CalcOpticalFlow(frame_, frame_old_);  // NOTE: Inverting the previous and current
  frame_old_= frame_.clone();

  GetAngleSpdImg(img_angle_, img_spd_);


  cv::threshold(img_spd_, mask_, /*thresh=*/optflow_spd_threshold_, /*maxval=*/1.0, cv::THRESH_BINARY);
  mask_.convertTo(mask_,CV_8UC1);
  if(erode_dilate_>0)
  {
    cv::erode(mask_,mask_,cv::Mat(),cv::Point(-1,-1), erode_dilate_);
    cv::dilate(mask_,mask_,cv::Mat(),cv::Point(-1,-1), erode_dilate_);
  }
}
//-------------------------------------------------------------------------------------------

void TFlowFinder::UpdateProc2_ContourAnalysis(const cv::Mat &mask)
{
  // Estimating flows by analyzing countour
  flow_elmts_.clear();
  contours_.clear();
  cv::Mat mask2= mask.clone();
  cv::findContours(mask2,contours_,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  for(int i(0); i<contours_.size(); ++i)
  {
    double avr_angle(0.0), avr_spd(0.0);
    cv::Point2d center;
    // Compute average angle and speed from optical flow image:
    // cv::Rect bound= cv::boundingRect(contours_[i]);
    // int num_points(0);
    // for (int px(bound.x); px<bound.x+bound.width; ++px)
    // {
      // for (int py(bound.y); py<bound.y+bound.height; ++py)
      // {
        // // if(cv::pointPolygonTest(contours_[i],cv::Point2f(px,py),/*measureDist=*/false)>=0.0)
        // {
          // avr_spd+= img_spd_.at<float>(py,px);
          // avr_angle+= img_angle_.at<float>(py,px);
          // ++num_points;
        // }
      // }
    // }
    // avr_spd/= double(num_points);
    // avr_angle/= double(num_points);
    // Compute average angle and speed from bounding box:
    cv::RotatedRect bound= cv::minAreaRect(contours_[i]);
    {
      cv::Point2f pts[4];
      bound.points(pts);
      center= 0.25*(pts[0]+pts[1]+pts[2]+pts[3]);
      cv::Vec2f v,v1(pts[1]-pts[0]),v2(pts[2]-pts[1]);
      if(cv::norm(v1)>cv::norm(v2))  v= v1;
      else  v= v2;
      avr_spd= cv::norm(v);
      avr_angle= std::atan2(v[1],v[0]);
    }
    // if(bound.size.height/bound.size.width>3.0)
    // {
      // /*Draw bount box*/{
        // cv::Point2f pts[4];
        // cv::Point ptsi[4];
        // bound.points(pts);
        // int n_pts(4);
        // for(int p(0);p<4;++p)  ptsi[p]= pts[p];
        // const cv::Point *ppts[4]= {&ptsi[0],&ptsi[1],&ptsi[2],&ptsi[3]};
        // cv::polylines(img_spd, ppts, &n_pts, /*ncontours=*/1, /*isClosed=*/true, cv::Scalar(1.0), /*thickness=*/1, /*lineType=*/8);
        // cv::polylines(img_angle, ppts, &n_pts, /*ncontours=*/1, /*isClosed=*/true, cv::Scalar(M_PI), /*thickness=*/1, /*lineType=*/8);
      // }
    // }

    // Moments, replace center by value from moments:
    cv::Moments mu= cv::moments(contours_[i]);
    if(std::fabs(mu.m00)>1.0e-6)
    {
      center= cv::Point2d(mu.m10/mu.m00, mu.m01/mu.m00);
    }

    TFlowElement flow;
    flow.X= center.x;
    flow.Y= center.y;
    flow.VX= avr_spd*std::cos(avr_angle);
    flow.VY= avr_spd*std::sin(avr_angle);
    flow.Speed= avr_spd;
    flow.Angle= avr_angle;
    flow.Amount= cv::contourArea(contours_[i]);
    flow.Contour= &contours_[i];

    bool add(true);
    if(amount_min_>=0.0 && flow.Amount<amount_min_)  add= false;
    else if(amount_max_>=0.0 && flow.Amount>amount_max_)  add= false;
    else if(speed_min_>=0.0 && flow.Speed<speed_min_)  add= false;
    else if(speed_max_>=0.0 && flow.Speed>speed_max_)  add= false;

    if(add)  flow_elmts_.push_back(flow);
  }
}
//-------------------------------------------------------------------------------------------

void TFlowFinder::DrawFlow(cv::Mat &frame, const cv::Scalar &color, const double &len, int thickness)
{
  cv::Scalar col2(color*0.5);
  cv::Scalar col3(color*0.8);
  for(std::list<TFlowElement>::const_iterator itr(flow_elmts_.begin()),itr_end(flow_elmts_.end());
      itr!=itr_end; ++itr)
  {
    // Draw contour:
    if(itr->Contour!=NULL && itr->Contour->size()>0)
    {
      const cv::Point *pts= (const cv::Point*) cv::Mat(*itr->Contour).data;
      int npts= itr->Contour->size();
      cv::fillPoly(frame, &pts, &npts, /*ncontours=*/1, col2, /*lineType=*/8);
      cv::polylines(frame, &pts, &npts, /*ncontours=*/1, /*isClosed=*/true, col3, /*thickness=*/1, /*lineType=*/8);
    }
    // Draw flow:
    cv::Point2d center(itr->X, itr->Y);
    cv::Point2d vel(itr->VX, itr->VY);
    cv::line(frame, center, center+len*vel, color, thickness, /*line_type=*/CV_AA, 0);
  }
}
//-------------------------------------------------------------------------------------------

void TFlowFinder::CalcOpticalFlow(const cv::Mat &prev, const cv::Mat &curr)
{
  cols_= curr.cols;
  rows_= curr.rows;
  velx_.create(rows_, cols_, CV_32FC1);
  vely_.create(rows_, cols_, CV_32FC1);
  velx_= cv::Scalar(0.0);
  vely_= cv::Scalar(0.0);

  CvMat prev2(prev), curr2(curr), velx2(velx_), vely2(vely_);

  // Using HS:
  // CvTermCriteria criteria= cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 64, 0.01);
  // cvCalcOpticalFlowHS(&prev2, &curr2, 0, &velx2, &vely2, 100.0, criteria);
  // Using LK:
  cvCalcOpticalFlowLK(&prev2, &curr2, optflow_win_size_, &velx2, &vely2);
}
//-------------------------------------------------------------------------------------------

void TFlowFinder::GetAngleSpdImg(cv::Mat &img_angle, cv::Mat &img_spd)
{
  img_spd.create(rows_, cols_, CV_32FC1);
  img_angle.create(rows_, cols_, CV_32FC1);

  typedef cv::MatIterator_<float> t_itr;
  t_itr ivx(velx_.begin<float>()), ivy(vely_.begin<float>());
  t_itr itr_d(img_spd.begin<float>()), itr_a(img_angle.begin<float>());
  t_itr itr_d_last(img_spd.end<float>());

  for(; itr_d!=itr_d_last; ++ivx,++ivy,++itr_d,++itr_a)
  {
    *itr_d= std::sqrt((*ivx)*(*ivx)+(*ivy)*(*ivy));
    *itr_a= std::atan2(*ivy,*ivx);
  }
}
//------------------------------------------------------------------------------


//===========================================================================================
// Utility
//===========================================================================================

// TEST: Compute average speed, angle
void CalcFlowAverage(const TFlowFinder &ff, cv::Vec2d &avr_xy, cv::Vec2d &avr_vel, cv::Vec2d &avr_spddir)
{
  double sum_amt(0.0);
  const std::list<TFlowElement> &flow(ff.FlowElements());
  for(std::list<TFlowElement>::const_iterator itr(flow.begin()),itr_end(flow.end());
      itr!=itr_end; ++itr)
  {
    double amt= itr->Amount;
    sum_amt+= amt;
    avr_xy[0]+= amt*itr->X;
    avr_xy[1]+= amt*itr->Y;
    avr_spddir[0]+= amt*itr->Speed;
    avr_spddir[1]+= amt*std::fabs(itr->Angle);
  }
  if(sum_amt>1.0e-6)
  {
    avr_xy/= sum_amt;
    avr_spddir/= sum_amt;
  }
  else
  {
    avr_xy= cv::Vec2d(0.0,0.0);
    avr_spddir= cv::Vec2d(0.0,0.0);
  }
  avr_vel= cv::Vec2d(avr_spddir[0]*std::cos(avr_spddir[1]), avr_spddir[0]*std::sin(avr_spddir[1]));
  // std::cerr<<"spd,angle: "<<avr_spddir<<std::endl;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

