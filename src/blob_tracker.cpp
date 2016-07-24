//-------------------------------------------------------------------------------------------
/*! \file    blob_tracker.cpp
    \brief   certain c++ unit file
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.10, 2016
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/blob_tracker.h"
#include "lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/imgproc/imgproc.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


void DrawPointMoves(cv::Mat &img, const std::vector<TPointMove> &move,
    const cv::Scalar &col1, const cv::Scalar &col2,
    const float &ds_emp,  // Emphasize (scale) ratio of DS
    const float &dp_emp  // Emphasize (scale) ratio of DP
  )
{
  // for(std::vector<TPointMove>::const_iterator m(move.begin()),m_end(move.end()); m!=m_end; ++m)
  // {
    // // cv::circle(img, m->Po, m->So, col1);
    // // cv::circle(img, m->Po, m->So+ds_emp*m->DS, col2, ds_emp*m->DS);
    // cv::circle(img, m->Po, m->So, col1, ds_emp*m->DS);
    // cv::line(img, m->Po, m->Po+dp_emp*m->DP, col2, 3);
  // }
  for(std::vector<TPointMove>::const_iterator m(move.begin()),m_end(move.end()); m!=m_end; ++m)
    cv::circle(img, m->Po, m->So, col1, ds_emp*m->DS);
  for(std::vector<TPointMove>::const_iterator m(move.begin()),m_end(move.end()); m!=m_end; ++m)
    cv::circle(img, m->Po+dp_emp*m->DP, m->So+ds_emp*m->DS, col2);
  for(std::vector<TPointMove>::const_iterator m(move.begin()),m_end(move.end()); m!=m_end; ++m)
    cv::line(img, m->Po, m->Po+dp_emp*m->DP, col2, 3);
}
//-------------------------------------------------------------------------------------------

// Track individual blobs.  prev: base, curr: current.
void TrackKeyPoints(
    const std::vector<cv::KeyPoint> &prev,
    const std::vector<cv::KeyPoint> &curr,
    std::vector<TPointMove> &move,
    const float &dist_min,  // Minimum distance change (i.e. sensitivity)
    const float &dist_max,  // Maximum distance change (too large might be noise)
    const float &ds_min,  // Minimum size change (i.e. sensitivity)
    const float &ds_max  // Maximum size change (too large might be noise)
    // const float &dd_max   // Maximum change of distance change (low pass filter)
  )
{
  std::vector<TPointMove> old_move(move);  // for filter
  // move.clear();
  // move.reserve(prev.size());
  move.resize(prev.size());
  typedef std::pair<int,float> TTracked;
  std::vector<TTracked> tracked;  // [idx of curr]=(idx of prev, dist)
  tracked.resize(curr.size(), TTracked(-1,0.0));
  int p_idx(0);
  for(std::vector<cv::KeyPoint>::const_iterator p(prev.begin()),p_end(prev.end()); p!=p_end; ++p,++p_idx)
  {
    float dp_min(dist_max);
    std::vector<cv::KeyPoint>::const_iterator c_min(curr.end());
    int c_idx(0), c_min_idx(-1);
    for(std::vector<cv::KeyPoint>::const_iterator c(curr.begin()),c_end(curr.end()); c!=c_end; ++c,++c_idx)
    {
      float dist= Dist(p->pt, c->pt);
      if(dist<dp_min)
      {
        dp_min= dist;
        c_min= c;
        c_min_idx= c_idx;
      }
    }
    // TPointMove m;
    TPointMove &m(move[p_idx]);
    m.Po= p->pt;
    m.So= p->size;
    cv::Point2f dp= c_min->pt - m.Po;
    float ds= c_min->size - m.So;
    float &dist(dp_min);  // norm of dp
    if(dist>=dist_min && dist<dist_max && ds>=ds_min && ds<ds_max
      && (tracked[c_min_idx].first<0 || tracked[c_min_idx].second<dist)
      /*&& (old_move.size()==0 || Dist(dp,old_move[p_idx].DP)<dd_max)*/ )
    {
      if(tracked[c_min_idx].first>=0)
      {
        move[tracked[c_min_idx].first].DP= cv::Point2f(0.0,0.0);
        move[tracked[c_min_idx].first].DS= 0.0;
      }
      m.DP= dp;
      m.DS= ds;
      tracked[c_min_idx]= TTracked(p_idx, dist);
    }
    else
    {
      m.DP= cv::Point2f(0.0,0.0);
      m.DS= 0.0;
    }
    // move.push_back(m);
  }
}
//-------------------------------------------------------------------------------------------

std::vector<cv::KeyPoint> CalibrateOrigin(
    const std::vector<std::vector<cv::KeyPoint> > &data,
    const float &dist_neighbor,  // Minimum distance to a neighbor blob.
    const float &dist_min,
    const float &dist_max,
    const float &ds_min,
    const float &ds_max
    // const float &dd_max
  )
{
  std::vector<cv::KeyPoint> origin;
  if(data.size()==0)  return origin;
  std::vector<TPointMove> move;
  origin= data[0];
  for(int i(0),i_end(origin.size()); i<i_end; ++i)
  {
    for(int j(i_end-1); j>i; --j)
    {
      float dist= Dist(origin[i].pt, origin[j].pt);
      if(dist<dist_neighbor)
      {
        origin.erase(origin.begin()+j);
        --i_end;
      }
    }
  }
  for(int i(1),i_end(data.size()); i<i_end; ++i)
  {
    TrackKeyPoints(origin, data[i], move, dist_min, dist_max, ds_min, ds_max/*, dd_max*/);
    for(int j(move.size()-1); j>=0; --j)
    {
      if(move[j].DP.x!=0.0 || move[j].DP.y!=0.0)
        origin.erase(origin.begin()+j);
    }
  }
  return origin;
}
//-------------------------------------------------------------------------------------------

TBlobTrackerParams::TBlobTrackerParams()
{
  SBDParams.filterByColor= 0;
  SBDParams.blobColor= 0;
  // Change thresholds
  SBDParams.minThreshold = 5;
  SBDParams.maxThreshold = 200;
  // Filter by Area.
  SBDParams.filterByArea = true;
  SBDParams.minArea = 40;
  // Filter by Circularity
  SBDParams.filterByCircularity = true;
  SBDParams.minCircularity = 0.10;
  // Filter by Convexity
  SBDParams.filterByConvexity = true;
  SBDParams.minConvexity = 0.87;
  // Filter by Inertia
  SBDParams.filterByInertia = true;
  SBDParams.minInertiaRatio = 0.01;

  // For blob detection:
  DistNeighbor= 20.0;
  // For blob tracking:
  DistMin= 2.0;
  DistMax= 10.0;
  DSMin= 0.0;
  DSMax= 10.0;
  // For visualization:
  DSEmp= 4.0;
  DPEmp= 10.0;

  // For calibration:
  NCalibPoints= 20;
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TBlobTrackerParams> &blob_params, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"BlobTracker"<<"[";
  for(std::vector<TBlobTrackerParams>::const_iterator itr(blob_params.begin()),itr_end(blob_params.end()); itr!=itr_end; ++itr)
  {
    fs<<"{";
    #define PROC_VAR(x,y)  fs<<#x"_"#y<<itr->x.y;
    PROC_VAR(SBDParams,filterByColor       );
    PROC_VAR(SBDParams,blobColor           );
    PROC_VAR(SBDParams,minThreshold        );
    PROC_VAR(SBDParams,maxThreshold        );
    PROC_VAR(SBDParams,filterByArea        );
    PROC_VAR(SBDParams,minArea             );
    PROC_VAR(SBDParams,filterByCircularity );
    PROC_VAR(SBDParams,minCircularity      );
    PROC_VAR(SBDParams,filterByConvexity   );
    PROC_VAR(SBDParams,minConvexity        );
    PROC_VAR(SBDParams,filterByInertia     );
    PROC_VAR(SBDParams,minInertiaRatio     );
    #undef PROC_VAR
    #define PROC_VAR(x)  fs<<#x<<itr->x;
    PROC_VAR(DistNeighbor );
    PROC_VAR(DistMin      );
    PROC_VAR(DistMax      );
    PROC_VAR(DSMin        );
    PROC_VAR(DSMax        );
    PROC_VAR(DSEmp        );
    PROC_VAR(DPEmp        );
    PROC_VAR(NCalibPoints );
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TBlobTrackerParams> &blob_params, const std::string &file_name)
{
  blob_params.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["BlobTracker"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TBlobTrackerParams cf;
    #define PROC_VAR(x,y)  if(!(*itr)[#x"_"#y].empty())  (*itr)[#x"_"#y]>>cf.x.y;
    PROC_VAR(SBDParams,filterByColor       );
    PROC_VAR(SBDParams,blobColor           );
    PROC_VAR(SBDParams,minThreshold        );
    PROC_VAR(SBDParams,maxThreshold        );
    PROC_VAR(SBDParams,filterByArea        );
    PROC_VAR(SBDParams,minArea             );
    PROC_VAR(SBDParams,filterByCircularity );
    PROC_VAR(SBDParams,minCircularity      );
    PROC_VAR(SBDParams,filterByConvexity   );
    PROC_VAR(SBDParams,minConvexity        );
    PROC_VAR(SBDParams,filterByInertia     );
    PROC_VAR(SBDParams,minInertiaRatio     );
    #undef PROC_VAR
    #define PROC_VAR(x)  if(!(*itr)[#x].empty())  (*itr)[#x]>>cf.x;
    PROC_VAR(DistNeighbor );
    PROC_VAR(DistMin      );
    PROC_VAR(DistMax      );
    PROC_VAR(DSMin        );
    PROC_VAR(DSMax        );
    PROC_VAR(DSEmp        );
    PROC_VAR(DPEmp        );
    PROC_VAR(NCalibPoints );
    #undef PROC_VAR
    blob_params.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// class TBlobTracker
//-------------------------------------------------------------------------------------------

void TBlobTracker::Init()
{
  detector_= new cv::SimpleBlobDetector(params_.SBDParams);
}
//-------------------------------------------------------------------------------------------

void TBlobTracker::Step(const cv::Mat &img)
{
  detector_->detect(img, keypoints_curr_);
  // cv::drawKeypoints(img, keypoints_curr_, img, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  if(keypoints_orig_.size()==0)  keypoints_orig_= keypoints_curr_;
  TrackKeyPoints(keypoints_orig_, keypoints_curr_, keypoints_move_,
      params_.DistMin, params_.DistMax, params_.DSMin, params_.DSMax);
}
//-------------------------------------------------------------------------------------------

void TBlobTracker::Draw(cv::Mat &img)
{
  DrawPointMoves(img, keypoints_move_, cv::Scalar(255,0,0), cv::Scalar(0,0,255));
}
//-------------------------------------------------------------------------------------------

void TBlobTracker::Calibrate(cv::VideoCapture &cap, boost::function<void(cv::Mat&)> modifier)
{
  std::cerr<<"Calibrating..."<<std::endl;
  // keypoints_orig_= keypoints_curr_;
  std::vector<std::vector<cv::KeyPoint> > data;
  cv::Mat frame;
  for(int i(0); i<params_.NCalibPoints; ++i)
  {
    cap >> frame; // get a new frame from camera
    if(modifier)  modifier(frame);
    detector_->detect(frame, keypoints_curr_);
    data.push_back(keypoints_curr_);
  }
  keypoints_orig_= CalibrateOrigin(data, params_.DistNeighbor,
      params_.DistMin, params_.DistMax, params_.DSMin, params_.DSMax);
}
//-------------------------------------------------------------------------------------------

void TBlobTracker::Calibrate(const std::vector<cv::Mat> &images)
{
  std::cerr<<"Calibrating..."<<std::endl;
  // keypoints_orig_= keypoints_curr_;
  std::vector<std::vector<cv::KeyPoint> > data;
  for(int i(0),i_end(images.size()); i<i_end; ++i)
  {
    detector_->detect(images[i], keypoints_curr_);
    data.push_back(keypoints_curr_);
  }
  keypoints_orig_= CalibrateOrigin(data, params_.DistNeighbor,
      params_.DistMin, params_.DistMax, params_.DSMin, params_.DSMax);
}
//-------------------------------------------------------------------------------------------

void TBlobTracker::SaveCalib(const std::string &file_name) const
{
  WriteToYAML(keypoints_orig_, file_name);
}
//-------------------------------------------------------------------------------------------

void TBlobTracker::LoadCalib(const std::string &file_name)
{
  ReadFromYAML(keypoints_orig_, file_name);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

