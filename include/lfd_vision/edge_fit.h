//-------------------------------------------------------------------------------------------
/*! \file    edge_fit.h
    \brief   Fitting a given edge (set of points) to an image.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jun.28, 2016
*/
//-------------------------------------------------------------------------------------------
#ifndef edge_fit_h
#define edge_fit_h
//-------------------------------------------------------------------------------------------
#include "lfd_vision/vision_util.h"
#include "lfd_vision/optimizer.h"
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

struct TEdgeDetectParams
{
  int PreBlurSize;
  int PostBlurSize;

  TEdgeDetectParams();
};

struct TEdgeEvalParams
{
  float MinEdgeBrightness;  // If an edge brightness is smaller than this, it's considered as a non-edge point
  float MinMatchingRatio;  // If the ratio of mached edge points is smaller than this value, return infeasible

  TEdgeEvalParams();
};

struct TEdgeFitParams
{
  TEdgeDetectParams EdgeDetect;
  TEdgeEvalParams EdgeEval;
  TCMAESParams CMAES;
  double XMin[6];  // Lower bound of optimized parameters.
  double XMax[6];  // Upper bound of optimized parameters.
  double Sig0[6];  // Initial standard deviation.

  /*Note.
    Parameterization of pose:
      X: Displacement from a reference pose pref.
      X[0:3]: Displacement of x,y,z position.
      X[3:6]: Rotation vector = angle*axis(x,y,z).
  */

  cv::Mat LPoints3d;  // Nx3 matrix storing local edge points of a model
  cv::Mat P1, P2;

  double VizBlendRatio;  // Blend ratio of original frame (0: no original frame).

  TEdgeFitParams();
};

//-------------------------------------------------------------------------------------------


void DetectEdges(const cv::Mat &frame, cv::Mat &edges_x, cv::Mat &edges_y, const TEdgeDetectParams &params);
//-------------------------------------------------------------------------------------------


class TEdgeFit
{
public:
  /* Execute fitting. pose0 is used as a reference pose.
    The resulting pose is stored into pose.
    pose can be the same as pose0. */
  void Run(const cv::Mat &frame1, const cv::Mat &frame2, const double pose0[7], double pose[7]);

  // Detect edges (automatically called in Run).
  void DetectEdges2(const cv::Mat &frame1, const cv::Mat &frame2);

  // Visualize the result.  Should be called after Run or DetectEdges.
  void Viz(cv::Mat &disp1, cv::Mat &disp2, const double pose[7]) const;

  TEdgeFitParams& Params()  {return params_;}
  const TEdgeFitParams& Params() const {return params_;}

private:
  TEdgeFitParams params_;

  cv::Mat frame1_, frame2_;
  cv::Mat edges1_x_, edges1_y_;
  cv::Mat edges2_x_, edges2_y_;

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // edge_fit_h
//-------------------------------------------------------------------------------------------
