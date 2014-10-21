//-------------------------------------------------------------------------------------------
/*! \file    flow_analyzer.cpp
    \brief   Flow analyzer.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.17, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/flow_analyzer.h"
//-------------------------------------------------------------------------------------------
// #include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/crop_box.h>
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
  std::vector<int> bb_clipped;

  int idx(0);
  for(std::vector<TBoundingBox>::const_iterator bb_itr(bounding_boxes_.begin()),bb_end(bounding_boxes_.end());
      bb_itr!=bb_end; ++bb_itr, ++idx)
  {
    if(!bb_itr->Active)
    {
      out_counts[idx]= 0;
      continue;
    }

    // 1. Transform points by bb_itr->X^-1
    // 2. Extract points by min/max (bounding box) -0.5*bb_itr->Size, 0.5*bb_itr->Size
    Eigen::Affine3d Tbb,Tbb_i;
    Tbb= XToEigMat(bb_itr->X) * Eigen::Scaling(0.5*bb_itr->Size[0],0.5*bb_itr->Size[1],0.5*bb_itr->Size[2]);
    Tbb_i= Tbb.inverse();
    // pcl::transformPointCloud(*cloud_orig, *cloud_bb, Tbb_i.matrix().cast<float>());
    // pcl::BoxClipper3D<pcl::PointXYZ>  bb_clipper(Tbb_i.matrix().cast<float>());
    // bb_clipper.clipPointCloud3D(*cloud_orig, bb_clipped);
    pcl::CropBox<pcl::PointXYZ>  bb_clipper;
    bb_clipper.setMin(Eigen::Vector4f(-1.0,-1.0,-1.0,0.0));
    bb_clipper.setMax(Eigen::Vector4f(1.0,1.0,1.0,0.0));
    bb_clipper.setTransform(Tbb_i.cast<float>());
    bb_clipper.setInputCloud(cloud_orig);
    bb_clipper.filter(bb_clipped);

    // 3. Count the number
    out_counts[idx]= bb_clipped.size();
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

