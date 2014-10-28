//-------------------------------------------------------------------------------------------
/*! \file    flow_analyzer.h
    \brief   Flow analyzer.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.17, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef flow_analyzer_h
#define flow_analyzer_h
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <valarray>
#include <geometry_msgs/Pose.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------


// Convert x to geometry_msgs/Pose
inline geometry_msgs::Pose XToGPose(const double x[7])
{
  geometry_msgs::Pose pose;
  pose.position.x= x[0];
  pose.position.y= x[1];
  pose.position.z= x[2];
  pose.orientation.x= x[3];
  pose.orientation.y= x[4];
  pose.orientation.z= x[5];
  pose.orientation.w= x[6];
  return pose;
}
//-------------------------------------------------------------------------------------------

// Convert geometry_msgs/Pose to x
inline void GPoseToX(const geometry_msgs::Pose &pose, double x[7])
{
  x[0]= pose.position.x;
  x[1]= pose.position.y;
  x[2]= pose.position.z;
  x[3]= pose.orientation.x;
  x[4]= pose.orientation.y;
  x[5]= pose.orientation.z;
  x[6]= pose.orientation.w;
}
//-------------------------------------------------------------------------------------------

// Operation kind, should be the same as the definition in IndexedBBEquation.msg
enum TBBOperationKind
{
  okUnion=0,  //l | r
  okIntersection=1,  //l & r
  okDifference=2  //l - r
};
struct TBBEquation
{
  TBBOperationKind Op;
  int IdxL;  // Left hand side
  int IdxR;  // Right hand side
};
//-------------------------------------------------------------------------------------------

struct TBoundingBox
{
  bool Active;
  bool IsEquation;
  double X[7];  // x,y,z, qx,qy,qz,qw
  double Size[3];  // x-size,y-size,z-size
  TBBEquation Eq;
  TBoundingBox() : Active(false), IsEquation(false) {}
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TFlowAnalyzer
//===========================================================================================
{
public:
  TFlowAnalyzer();

  void InitBoundingBoxes(int n)  {bounding_boxes_.resize(n);}
  void SetBoundingBox(int idx, const TBoundingBox &bb)  {bounding_boxes_[idx]= bb;}
  TBoundingBox& RefBoundingBox(int idx)  {return bounding_boxes_[idx];}

  void AnalyzeBBs(
      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_orig,
      std::vector<int> &out_counts);

private:
  std::vector<TBoundingBox>  bounding_boxes_;
  std::vector<std::vector<int> > bb_clipped_;

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // flow_analyzer_h
//-------------------------------------------------------------------------------------------
