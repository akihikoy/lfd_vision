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
#include "lfd_vision/geom_util.h"
#include "lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <valarray>
//-------------------------------------------------------------------------------------------
namespace trick
{
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

struct TBoundingBox : TRotatedBoundingBox<double>
{
  bool Active;
  bool IsEquation;
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
