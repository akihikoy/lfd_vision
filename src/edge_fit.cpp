//-------------------------------------------------------------------------------------------
/*! \file    edge_fit.cpp
    \brief   Fitting a given edge (set of points) to an image.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jun.28, 2016
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/edge_fit.h"
#include "lfd_vision/optimizer.h"
#include "lfd_vision/geom_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

TEdgeDetectParams::TEdgeDetectParams()
  :
    PreBlurSize(3),
    PostBlurSize(21)
{
}

TEdgeEvalParams::TEdgeEvalParams()
  :
    MinEdgeBrightness(50.0),
    MinMatchingRatio(0.4)
{
}

TEdgeFitParams::TEdgeFitParams()
  :
    VizBlendRatio(0.25)
{
  CMAES.lambda= 100;
  CMAES.stopMaxFunEvals= 2000;
  CMAES.stopTolFun= 1.0e-6;
  CMAES.stopTolFunHist= -1.0;  // Disable
  // CMAES.diagonalCov= 1.0;
  CMAES.PrintLevel= 0;

  double bounds[2][6]= {
    {-0.1,-0.1,-0.1, -0.2,-0.2,-0.2},
    {+0.1,+0.1,+0.1, +0.2,+0.2,+0.2} };
  double sig0[6]= {0.02,0.02,0.02, 0.01,0.01,0.01};
  std::copy(bounds[0],bounds[0]+6,XMin);
  std::copy(bounds[1],bounds[1]+6,XMax);
  std::copy(sig0,sig0+6,Sig0);
}

//-------------------------------------------------------------------------------------------


template <typename t_value>
void TransformPoints(const cv::Mat &l_points3d, const double pose[7], cv::Mat &points3d)
{
  int N(l_points3d.rows);
  points3d.create(N,3,l_points3d.type());
  for(int i(0);i<N;++i)
  {
    double pd[3]= {(double)l_points3d.at<t_value>(i,0),(double)l_points3d.at<t_value>(i,1),(double)l_points3d.at<t_value>(i,2)};
    double p2d[3];
    TransformP(pose,pd,p2d);
    points3d.at<t_value>(i,0)= p2d[0];
    points3d.at<t_value>(i,1)= p2d[1];
    points3d.at<t_value>(i,2)= p2d[2];
  }
}
//-------------------------------------------------------------------------------------------

template <typename t_value>
void PointSeqToGradients(const cv::Mat &points2d, cv::Mat &grads2d)
{
  int N(points2d.rows);
  grads2d.create(N,2,points2d.type());
  if(N<=1)  return;
  for(int i(0);i<N;++i)
  {
    t_value p1[2]= {points2d.at<t_value>(i,0), points2d.at<t_value>(i,1)};
    t_value &gx(grads2d.at<t_value>(i,0)), &gy(grads2d.at<t_value>(i,1));
    if(i==0)
    {
      t_value p2[2]= {points2d.at<t_value>(i+1,0), points2d.at<t_value>(i+1,1)};
      gx= p2[0]-p1[0];
      gy= p2[1]-p1[1];
    }
    else if(i==N-1)
    {
      t_value p0[2]= {points2d.at<t_value>(i-1,0), points2d.at<t_value>(i-1,1)};
      gx= p1[0]-p0[0];
      gy= p1[1]-p0[1];
    }
    else
    {
      t_value p0[2]= {points2d.at<t_value>(i-1,0), points2d.at<t_value>(i-1,1)};
      t_value p2[2]= {points2d.at<t_value>(i+1,0), points2d.at<t_value>(i+1,1)};
      gx= p2[0]-p0[0];
      gy= p2[1]-p0[1];
    }
  }
}
//-------------------------------------------------------------------------------------------

template <typename t_value>
void DrawPoints(cv::Mat &img, const cv::Mat &points, const cv::Scalar &col)
{
  for(int r(0),r_end(points.rows);r<r_end;++r)
  {
    int p[2]= {(int)points.at<t_value>(r,0), (int)points.at<t_value>(r,1)};
    // img.at<cv::Vec3b>(p[1],p[0])= cv::Vec3b(col[0],col[1],col[2]);
    cv::circle(img, cv::Point(p[0],p[1]), 2, col);
  }
}
//-------------------------------------------------------------------------------------------

template <typename t_value>
void DrawPoints(cv::Mat &img, const cv::Mat &points, const cv::Mat &grads)
{
  for(int r(0),r_end(points.rows);r<r_end;++r)
  {
    int p[2]= {(int)points.at<t_value>(r,0), (int)points.at<t_value>(r,1)};
    int g[2]= {(int)(std::fabs(grads.at<t_value>(r,0))), (int)(std::fabs(grads.at<t_value>(r,1)))};
    float lg(std::sqrt(g[0]*g[0]+g[1]*g[1]));
    cv::Scalar col(0.0, 255*g[1]/lg, 255*g[0]/lg);
    // img.at<cv::Vec3b>(p[1],p[0])= cv::Vec3b(col[0],col[1],col[2]);
    cv::circle(img, cv::Point(p[0],p[1]), 2, col);
  }
}
//-------------------------------------------------------------------------------------------

void DetectEdges(const cv::Mat &frame, cv::Mat &edges_x, cv::Mat &edges_y, const TEdgeDetectParams &params)
{
  cv::Mat frame_gray;
  cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
  cv::blur(frame_gray, frame_gray, cv::Size(params.PreBlurSize,params.PreBlurSize));
  int scale(1), delta(0), ddepth(CV_16S);
  // Gradient X
  //cv::Scharr(frame_gray, edges_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel(frame_gray, edges_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  // cv::convertScaleAbs(edges_x, edges_x);
  // Gradient Y
  //cv::Scharr(frame_gray, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel(frame_gray, edges_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  // cv::convertScaleAbs(edges_y, edges_y);
// std::cerr<<"edges_x.type() "<<edges_x.type()<<" "<<CV_16S<<" "<<CV_8U<<std::endl;
  cv::convertScaleAbs(edges_x, edges_x);  // becomes CV_8U
  cv::convertScaleAbs(edges_y, edges_y);  // becomes CV_8U
  // cv::dilate(edges_x,edges_x,cv::Mat(),cv::Point(-1,-1), 2);
  // cv::dilate(edges_y,edges_y,cv::Mat(),cv::Point(-1,-1), 2);
// std::cerr<<"edges_x.type() "<<edges_x.type()<<" "<<CV_16S<<" "<<CV_8U<<std::endl;
  // int bw(9);
  // cv::blur(edges_x, edges_x, cv::Size(bw,bw));
  // cv::blur(edges_y, edges_y, cv::Size(bw,bw));
  int bw(params.PostBlurSize);
  cv::GaussianBlur(edges_x, edges_x, cv::Size(bw,bw), 0, 0, cv::BORDER_DEFAULT);
  cv::GaussianBlur(edges_y, edges_y, cv::Size(bw,bw), 0, 0, cv::BORDER_DEFAULT);
}
//-------------------------------------------------------------------------------------------

template <typename t_value>
double EvaluateEdgePoints(
    const cv::Mat &edges_x, const cv::Mat &edges_y,
    const cv::Mat &points2d, const cv::Mat &grads2d,
    bool &is_feasible, const TEdgeEvalParams &params)
{
  int rows(edges_x.rows), cols(edges_x.cols);
  int num(0);
  double sum(0);
  for(int r(0),r_end(points2d.rows);r<r_end;++r)
  {
    int p[2]= {(int)points2d.at<t_value>(r,0), (int)points2d.at<t_value>(r,1)};
    // NOTE: Since we applied convertScaleAbs to edges, we apply fabs to gradients
    t_value g[2]= {std::fabs(grads2d.at<t_value>(r,0)), std::fabs(grads2d.at<t_value>(r,1))};
    if(p[0]>=0 && p[0]<cols && p[1]>=0 && p[1]<rows)
    {
      // sum+= edges.at<unsigned char>(p[1],p[0]);
      // sum+= (edges.at<unsigned char>(p[1],p[0])>20 ? 1.0 : 0.0);
      // sum+= std::log(1+edges.at<unsigned char>(p[1],p[0]));
      // NOTE: signed short if convertScaleAbs is not used, unsigned char if convertScaleAbs is used
      // float ex(edges_x.at<signed short>(p[1],p[0])), ey(edges_y.at<signed short>(p[1],p[0]));
      float ex(edges_x.at<unsigned char>(p[1],p[0])), ey(edges_y.at<unsigned char>(p[1],p[0]));
      float le(std::sqrt(ex*ex+ey*ey)), lg(std::sqrt(g[0]*g[0]+g[1]*g[1]));
      if(le>params.MinEdgeBrightness && lg>1.0e-4)
      {
        // NOTE: ex and ey are flipped because horizontal edge detection (x) detects y-directional gradient
        float dot= (ey*g[0]+ex*g[1])/(le*lg);
        // sum+= dot*le;
        // sum+= dot+std::atan(le)/3.14;
        // sum+= 1.0;
        // sum+= le;
        // sum+= (le>100.0 && dot>0.8 ? 1.0 : 0.0);
        sum+= dot*std::log(le);
        // sum+= dot*std::atan(le);
        // sum+= dot;
        // std::cerr<<" | "<<g[0]<<" "<<g[1]<<" "<<ex<<" "<<ey<<" "<<" "<<lg<<" "<<le<<" "<<dot;
        ++num;
      }
    }
  }
  // std::cerr<<" * "<<num<<"/"<<points2d.rows<<endl;
  sum/= (double)points2d.rows;
  if((double)num/(double)points2d.rows<params.MinMatchingRatio)  is_feasible= false;
  else is_feasible= true;
  return sum;
}
//-------------------------------------------------------------------------------------------

/* Parameterization of pose:
  dp: Displacement from a reference pose pref.
  dp[0:3]: Displacement of x,y,z position.
  dp[3:6]: Rotation vector = angle*axis(x,y,z).
*/
static
void ParamToPose(const double pref[7], const double dp[6], double pose[7])
{
  pose[0]= pref[0]+dp[0];
  pose[1]= pref[1]+dp[1];
  pose[2]= pref[2]+dp[2];
  Eigen::Vector3d axis(dp+3);
  double angle(axis.norm());
  if(angle>1.0e-6)  axis/= angle;
  Eigen::Quaterniond q= Eigen::AngleAxisd(angle,axis) * QToEigMat(pref+3);
  EigMatToQ(q, pose+3);
}
//-------------------------------------------------------------------------------------------

struct TFObjEdgePointsParams2
{
  const cv::Mat &edges1_x;
  const cv::Mat &edges1_y;
  const cv::Mat &edges2_x;
  const cv::Mat &edges2_y;
  const cv::Mat &l_points3d;
  const double *pose0;
  const cv::Mat &P1;
  const cv::Mat &P2;
  const TEdgeEvalParams &edge_eval_params;
  TFObjEdgePointsParams2(
      const cv::Mat &v_edges1_x, const cv::Mat &v_edges1_y,
      const cv::Mat &v_edges2_x, const cv::Mat &v_edges2_y,
      const cv::Mat &v_l_points3d, const double *v_pose0,
      const cv::Mat &v_P1, const cv::Mat &v_P2,
      const TEdgeEvalParams &v_edge_eval_params)
    :
      edges1_x     (v_edges1_x   ),
      edges1_y     (v_edges1_y   ),
      edges2_x     (v_edges2_x   ),
      edges2_y     (v_edges2_y   ),
      l_points3d   (v_l_points3d ),
      pose0        (v_pose0      ),
      P1           (v_P1         ),
      P2           (v_P2         ),
      edge_eval_params(v_edge_eval_params)
    {}
};

static
double FObjEdgePoints2(const TFObjEdgePointsParams2 &p, const double *param, bool &is_feasible)
{
  double pose[7];
  ParamToPose(p.pose0,param,pose);
  cv::Mat points3d, points2d1, grads2d1, points2d2, grads2d2;
  TransformPoints<float>(p.l_points3d, pose, points3d);
  ProjectPointsToRectifiedImg(points3d, p.P1, points2d1);
  ProjectPointsToRectifiedImg(points3d, p.P2, points2d2);
  PointSeqToGradients<float>(points2d1, grads2d1);
  PointSeqToGradients<float>(points2d2, grads2d2);

  bool is_feasible1(false), is_feasible2(false);
  double e1= -EvaluateEdgePoints<float>(p.edges1_x, p.edges1_y, points2d1, grads2d1, is_feasible1, p.edge_eval_params);
  double e2= -EvaluateEdgePoints<float>(p.edges2_x, p.edges2_y, points2d2, grads2d2, is_feasible2, p.edge_eval_params);
  is_feasible= is_feasible1 && is_feasible2;
  return e1+e2;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// class TEdgeFit
//-------------------------------------------------------------------------------------------

// Detect edges (automatically called in Run).
void TEdgeFit::DetectEdges2(const cv::Mat &frame1, const cv::Mat &frame2)
{
  frame1_= frame1;  // just a reference
  frame2_= frame2;  // ditto
  DetectEdges(frame1_, edges1_x_, edges1_y_, params_.EdgeDetect);
  DetectEdges(frame2_, edges2_x_, edges2_y_, params_.EdgeDetect);
}
//-------------------------------------------------------------------------------------------

/* Execute fitting. pose0 is used as a reference pose.
  The resulting pose is stored into pose.
  pose can be the same as pose0. */
void TEdgeFit::Run(const cv::Mat &frame1, const cv::Mat &frame2, const double pose0[7], double pose[7], double *quality)
{
  DetectEdges2(frame1, frame2);

  TFObjEdgePointsParams2 fparams(edges1_x_, edges1_y_, edges2_x_, edges2_y_,
                                params_.LPoints3d, pose0, params_.P1, params_.P2, params_.EdgeEval);
  const int Dim(6);
  double x0[Dim]= {0.0,0.0,0.0, 0.0,0.0,0.0};
  double xres[Dim];
  MinimizeF(boost::bind(&FObjEdgePoints2, fparams, _1, _2),
      x0, params_.Sig0, Dim, params_.XMin, params_.XMax, /*bound_len=*/Dim, xres, params_.CMAES);

  // std::cout<<"\nxres=";
  // for(int d(0);d<Dim;++d)  std::cout<<" "<<xres[d];
  // bool is_feasible;
  // std::cout<<"; "<<FObjEdgePoints2(fparams,xres,is_feasible);
  // std::cout<<std::endl;
  ParamToPose(pose0,xres,pose);
  if(quality!=NULL)
  {
    bool is_feasible;
    *quality= -FObjEdgePoints2(fparams,xres,is_feasible);
    if(!is_feasible)  *quality= 0.0;
  }
}
//-------------------------------------------------------------------------------------------

// Visualize the result.  Should be called after Run.
void TEdgeFit::Viz(cv::Mat &disp1, cv::Mat &disp2, const double pose[7]) const
{
  cv::Mat points3d, points2d1, grads2d1, points2d2, grads2d2;
  if(pose!=NULL)
  {
    TransformPoints<float>(params_.LPoints3d, pose, points3d);
    ProjectPointsToRectifiedImg(points3d, params_.P1, points2d1);
    ProjectPointsToRectifiedImg(points3d, params_.P2, points2d2);
    PointSeqToGradients<float>(points2d1, grads2d1);
    PointSeqToGradients<float>(points2d2, grads2d2);
  }
  cv::Mat grads1[3]= {0.0*edges1_x_, edges1_x_, edges1_y_};
  cv::merge(grads1,3,disp1);
  cv::Mat grads2[3]= {0.0*edges2_x_, edges2_x_, edges2_y_};
  cv::merge(grads2,3,disp2);
  if(pose!=NULL)
  {
    DrawPoints<float>(disp1,points2d1,grads2d1);
    DrawPoints<float>(disp2,points2d2,grads2d2);
  }
  disp1= frame1_*params_.VizBlendRatio+disp1;
  disp2= frame2_*params_.VizBlendRatio+disp2;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

