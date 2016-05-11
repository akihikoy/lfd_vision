//-------------------------------------------------------------------------------------------
/*! \file    blob_tracker.h
    \brief   Blob tracker.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.10, 2016
*/
//-------------------------------------------------------------------------------------------
#ifndef blob_tracker_h
#define blob_tracker_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/function.hpp>
#include <iostream>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

struct TPointMove
{
  cv::Point2f Po;  // Original position
  float So;        // Original size
  cv::Point2f DP;  // Displacement of position
  float DS;        // Displacement of size
};
void DrawPointMoves(cv::Mat &img, const std::vector<TPointMove> &move,
    const cv::Scalar &col1, const cv::Scalar &col2,
    const float &ds_emp=4.0,  // Emphasize (scale) ratio of DS to draw
    const float &dp_emp=10.0  // Emphasize (scale) ratio of DP to draw
  );
// Track individual blobs.  prev: base, curr: current.
void TrackKeyPoints(
    const std::vector<cv::KeyPoint> &prev,
    const std::vector<cv::KeyPoint> &curr,
    std::vector<TPointMove> &move,
    const float &dist_min,  // Minimum distance change (i.e. sensitivity)
    const float &dist_max,  // Maximum distance change (too large might be noise)
    const float &ds_min,  // Minimum size change (i.e. sensitivity)
    const float &ds_max  // Maximum size change (too large might be noise)
  );
std::vector<cv::KeyPoint> CalibrateOrigin(
    const std::vector<std::vector<cv::KeyPoint> > &data,
    const float &dist_neighbor,  // Minimum distance to a neighbor blob.
    const float &dist_min,
    const float &dist_max,
    const float &ds_min,
    const float &ds_max
    // const float &dd_max
  );
//-------------------------------------------------------------------------------------------

struct TBlobTrackerParams
{
  // For blob detection:
  cv::SimpleBlobDetector::Params SBDParams;
  float DistNeighbor;  // Minimum distance to a neighbor blob.
  // For blob tracking:
  float DistMin;  // Minimum distance change (i.e. sensitivity)
  float DistMax;  // Maximum distance change (too large might be noise)
  float DSMin;  // Minimum size change (i.e. sensitivity)
  float DSMax;  // Maximum size change (too large might be noise)
  // For visualization:
  float DSEmp;  // Emphasize (scale) ratio of DS to draw
  float DPEmp;  // Emphasize (scale) ratio of DP to draw
  // For calibration:
  int NCalibPoints;  // Number of points for calibration
};
void SetRecommendedBlobTrackerParams(TBlobTrackerParams &params);
//-------------------------------------------------------------------------------------------

class TBlobTracker
{
public:
  TBlobTracker()
    {SetRecommendedBlobTrackerParams(params_);}
  void Init();
  void Step(const cv::Mat &img);
  void Draw(cv::Mat &img);
  void Calibrate(cv::VideoCapture &cap, boost::function<void(cv::Mat&)> modifier=NULL);
  void Calibrate(const std::vector<cv::Mat> &images);

  TBlobTrackerParams& Params()  {return params_;}
  const TBlobTrackerParams& Params() const {return params_;}

private:
  TBlobTrackerParams params_;
  cv::Ptr<cv::SimpleBlobDetector> detector_;

  std::vector<cv::KeyPoint> keypoints_orig_, keypoints_curr_;
  std::vector<TPointMove> keypoints_move_;
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // blob_tracker_h
//-------------------------------------------------------------------------------------------
