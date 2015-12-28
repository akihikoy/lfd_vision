//-------------------------------------------------------------------------------------------
/*! \file    flow_analyzer.cpp
    \brief   Flow analyzer.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.17, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/flow_analyzer.h"
#include "lfd_vision/geom_util.h"
//-------------------------------------------------------------------------------------------
// #include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/crop_box.h>
#include <algorithm>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class TFlowAnalyzer
//===========================================================================================
TFlowAnalyzer::TFlowAnalyzer()
{
  InitBoundingBoxes(10);
}
//-------------------------------------------------------------------------------------------

void TFlowAnalyzer::AnalyzeBBs(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_orig,
    std::vector<int> &out_counts)
{
  out_counts.resize(bounding_boxes_.size());
  bb_clipped_.resize(bounding_boxes_.size());

  int idx(0);
  for(std::vector<TBoundingBox>::const_iterator bb_itr(bounding_boxes_.begin()),bb_end(bounding_boxes_.end());
      bb_itr!=bb_end; ++bb_itr, ++idx)
  {
    if(!bb_itr->Active)
    {
      out_counts[idx]= 0;
      continue;
    }

    if(bb_itr->IsEquation)
      continue;

    // 1. Transform points by bb_itr->X^-1
    // 2. Extract points by min/max (bounding box) -0.5*bb_itr->Size, 0.5*bb_itr->Size
    Eigen::Affine3d Tbb,Tbb_i;
    Tbb= XToEigMat(bb_itr->X) * Eigen::Scaling(0.5*bb_itr->Size[0],0.5*bb_itr->Size[1],0.5*bb_itr->Size[2]);
    Tbb_i= Tbb.inverse();
    // pcl::transformPointCloud(*cloud_orig, *cloud_bb, Tbb_i.matrix().cast<float>());
    // pcl::BoxClipper3D<pcl::PointXYZ>  bb_clipper(Tbb_i.matrix().cast<float>());
    // bb_clipper.clipPointCloud3D(*cloud_orig, bb_clipped_[idx]);
    pcl::CropBox<pcl::PointXYZ>  bb_clipper;
    bb_clipper.setMin(Eigen::Vector4f(-1.0,-1.0,-1.0,0.0));
    bb_clipper.setMax(Eigen::Vector4f(1.0,1.0,1.0,0.0));
    bb_clipper.setTransform(Tbb_i.cast<float>());
    bb_clipper.setInputCloud(cloud_orig);
    bb_clipper.filter(bb_clipped_[idx]);

    // 3. Count the number
    out_counts[idx]= bb_clipped_[idx].size();

    // Sort for future use
    std::sort(bb_clipped_[idx].begin(), bb_clipped_[idx].end());
  }

  idx= 0;
  for(std::vector<TBoundingBox>::const_iterator bb_itr(bounding_boxes_.begin()),bb_end(bounding_boxes_.end());
      bb_itr!=bb_end; ++bb_itr, ++idx)
  {
    if(!bb_itr->Active)
      continue;
    if(!bb_itr->IsEquation)
      continue;
    const std::vector<int> &lhs(bb_clipped_[bb_itr->Eq.IdxL]), &rhs(bb_clipped_[bb_itr->Eq.IdxR]);
    bb_clipped_[idx].resize(lhs.size()+rhs.size());
    std::vector<int>::iterator last(bb_clipped_[idx].begin());
    switch(bb_itr->Eq.Op)
    {
    case okUnion:
      last= std::set_union(lhs.begin(),lhs.end(), rhs.begin(),rhs.end(), bb_clipped_[idx].begin());
      break;
    case okIntersection:
      last= std::set_intersection(lhs.begin(),lhs.end(), rhs.begin(),rhs.end(), bb_clipped_[idx].begin());
      break;
    case okDifference:
      last= std::set_difference(lhs.begin(),lhs.end(), rhs.begin(),rhs.end(), bb_clipped_[idx].begin());
      break;
    }
    bb_clipped_[idx].resize(last-bb_clipped_[idx].begin());  // This should be sorted

    // Count the number
    out_counts[idx]= bb_clipped_[idx].size();
  }

}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

