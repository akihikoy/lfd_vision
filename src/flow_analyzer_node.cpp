//-------------------------------------------------------------------------------------------
/*! \file    flow_analyzer_node.cpp
    \brief   Flow analyzer.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.17, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/flow_analyzer.h"
#include "pr2_lfd_vision/Int32Array.h"
#include "pr2_lfd_vision/IndexedBoundingBox.h"
#include "pr2_lfd_vision/SetBoundingBox.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/passthrough.h>
#include <vector>
//-------------------------------------------------------------------------------------------
namespace trick
{

  // TFlowAnalyzer();

  // void InitBoundingBoxes(int n)  {bounding_boxes_.resize(n);}
  // void SetBoundingBox(int idx, const TBoundingBox &bb)  {bounding_boxes_[idx]= bb;}
  // TBoundingBox& RefBoundingBox(int idx)  {return bounding_boxes_[idx];}

  // void AnalyzeBBs(
      // pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_orig,
      // std::vector<int> &out_counts);


class TFlowAnalyzerNode
{
public:
  TFlowAnalyzerNode(ros::NodeHandle &node)
      :
        node_(node)
    {
      analyzer_.InitBoundingBoxes(20);

      pub_count_= node_.advertise<pr2_lfd_vision::Int32Array>("bb_counts", 1);

      srv_set_bb_= node_.advertiseService("set_bb", &TFlowAnalyzerNode::SetBoundingBox, this);

      sub_depth_= node_.subscribe("/depth_non_filtered", 1, &TFlowAnalyzerNode::PointCloudCallback, this);
      sub_indexed_bb_= node_.subscribe("indexed_bb", 1, &TFlowAnalyzerNode::IndexedBBCallback, this);
    }

  bool SetBoundingBox(pr2_lfd_vision::SetBoundingBox::Request &req, pr2_lfd_vision::SetBoundingBox::Response &res)
    {
      TBoundingBox bb;
      bb.Active= req.indexed_bb.active;
      GPoseToX(req.indexed_bb.pose, bb.X);
      if(req.indexed_bb.dimensions.size()==3)
      {
        for(int d(0);d<3;++d)  bb.Size[d]= req.indexed_bb.dimensions[d];
      }
      analyzer_.SetBoundingBox(req.indexed_bb.index, bb);
      res.success= true;  // TODO: check error
      return true;
    }

  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
      // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZI>);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromROSMsg(*msg, *cloud_src);
      // pcl::copyPointCloud(*cloud_src, *cloud_orig);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud_orig);

      // Pre-process
      // for(int i(0);i<19200;i+=45)  {std::cerr<<"p:"<<cloud_src->points[i].x<<"  "<<cloud_src->points[i].y<<"  "<<cloud_src->points[i].z<<"  "<<cloud_src->points[i].intensity<<std::endl;}
      // std::cerr<<"cloud_src->points.size():"<<cloud_src->points.size()<<std::endl;
      // std::cerr<<"cloud_orig->points.size():"<<cloud_orig->points.size()<<std::endl;
      // Build a passthrough filter to remove spurious NaNs
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud_orig);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-100.0, 100.0);
      pass.filter(*cloud_orig);
      // std::vector<int> indexes;
      // pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*cloud_orig, *cloud_orig, indexes);
      // std::cerr<<"cloud_orig->points.size():"<<cloud_orig->points.size()<<std::endl;

      // pcl::PointXYZ pt_min, pt_max;
      // pcl::getMinMax3D(*cloud_orig, pt_min, pt_max);
      // std::cerr<<"pt_min:"<<pt_min<<std::endl;
      // std::cerr<<"pt_max:"<<pt_max<<std::endl;

      pr2_lfd_vision::Int32Array  out_counts;
      analyzer_.AnalyzeBBs(cloud_orig, out_counts.data);
      for(size_t i(0);i<out_counts.data.size();++i)  std::cerr<<" "<<out_counts.data[i]; std::cerr<<std::endl;
      pub_count_.publish(out_counts);
    }

  void IndexedBBCallback(const pr2_lfd_vision::IndexedBoundingBoxConstPtr &msg)
    {
      TBoundingBox bb;
      bb.Active= msg->active;
      GPoseToX(msg->pose, bb.X);
      // std::cerr<<"msg->dimensions:"<<msg->dimensions.size()<<std::endl;
      if(msg->dimensions.size()==3)
      {
        for(int d(0);d<3;++d)  bb.Size[d]= msg->dimensions[d];
      }
      analyzer_.SetBoundingBox(msg->index, bb);
    }

private:
  TFlowAnalyzer analyzer_;

  ros::NodeHandle     &node_;
  ros::Publisher      pub_count_;
  ros::Subscriber     sub_depth_;
  ros::Subscriber     sub_indexed_bb_;
  ros::ServiceServer  srv_set_bb_;
};
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  ros::init(argc, argv, "flow_analyzer");
  ros::NodeHandle node("~");

  TFlowAnalyzerNode analyzer_node(node);

  ros::spin();
  return 0;
}
//-------------------------------------------------------------------------------------------
