//-------------------------------------------------------------------------------------------
/*! \file    flow_analyzer_node.cpp
    \brief   Flow analyzer.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.17, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/flow_analyzer.h"
#include "lfd_vision/sentis_m100.h"
#include "lfd_vision/flow_finder.h"
#include "lfd_vision/geom_util.h"
//-------------------------------------------------------------------------------------------
#include "lfd_vision/Int32Array.h"
#include "lfd_vision/IndexedBoundingBox.h"
#include "lfd_vision/SetBoundingBox.h"
#include "lfd_vision/SetBBEquation.h"
#include "lfd_vision/ReadRegister.h"
#include "lfd_vision/WriteRegister.h"
#include "lfd_vision/SetFrameRate.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // cvtColor
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/video/background_segm.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{

inline double GetCurrentTime(void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
  // return ros::Time::now().toSec();
}

class TFlowAnalyzerNode
{
public:
  TFlowAnalyzerNode(ros::NodeHandle &node, bool activate_callback=true)
      :
        node_(node)
    {
      analyzer_.InitBoundingBoxes(20);

      pub_count_= node_.advertise<lfd_vision::Int32Array>("bb_counts", 1);

      srv_set_bb_= node_.advertiseService("set_bb", &TFlowAnalyzerNode::SetBoundingBox, this);
      srv_set_bbeq_= node_.advertiseService("set_bbeq", &TFlowAnalyzerNode::SetBBEquation, this);

      if(activate_callback)
        sub_depth_= node_.subscribe("/depth_non_filtered", 1, &TFlowAnalyzerNode::PointCloudCallback, this);
      sub_indexed_bb_= node_.subscribe("indexed_bb", /*queue_size=*/10, &TFlowAnalyzerNode::IndexedBBCallback, this);
    }

  bool SetBoundingBox(lfd_vision::SetBoundingBox::Request &req, lfd_vision::SetBoundingBox::Response &res)
    {
      TBoundingBox bb;
      bb.Active= req.indexed_bb.active;
      bb.IsEquation= false;
      GPoseToX(req.indexed_bb.pose, bb.X);
      if(req.indexed_bb.dimensions.size()==3)
      {
        for(int d(0);d<3;++d)  bb.Size[d]= req.indexed_bb.dimensions[d];
      }
      analyzer_.SetBoundingBox(req.indexed_bb.index, bb);
      res.success= true;  // TODO: check error
      return true;
    }

  bool SetBBEquation(lfd_vision::SetBBEquation::Request &req, lfd_vision::SetBBEquation::Response &res)
    {
      TBoundingBox bb;
      bb.Active= req.indexed_bbeq.active;
      bb.IsEquation= true;
      bb.Eq.Op= static_cast<trick::TBBOperationKind>(req.indexed_bbeq.op);
      bb.Eq.IdxL= req.indexed_bbeq.idx_l;
      bb.Eq.IdxR= req.indexed_bbeq.idx_r;
      analyzer_.SetBoundingBox(req.indexed_bbeq.index, bb);
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

      AnalyzePointCloud(cloud_orig);
    }

  //! cloud_orig may change
  void AnalyzePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_orig)
    {
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

      lfd_vision::Int32Array  out_counts;
      analyzer_.AnalyzeBBs(cloud_orig, out_counts.data);
      for(size_t i(0);i<out_counts.data.size();++i)  std::cerr<<" "<<out_counts.data[i]; std::cerr<<std::endl;
      pub_count_.publish(out_counts);
    }

  void IndexedBBCallback(const lfd_vision::IndexedBoundingBoxConstPtr &msg)
    {
      TBoundingBox bb;
      bb.Active= msg->active;
      bb.IsEquation= false;
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
  ros::ServiceServer  srv_set_bbeq_;
};
//-------------------------------------------------------------------------------------------


class TSentisM100Node : public TSentisM100
{
public:
  TSentisM100Node(ros::NodeHandle &node)
      :
        TSentisM100(),
        node_(node)
    {
      srv_write_register_= node_.advertiseService("write_register", &TSentisM100Node::SrvWriteRegister, this);
      srv_read_register_= node_.advertiseService("read_register", &TSentisM100Node::SrvReadRegister, this);
      srv_set_frame_rate_= node_.advertiseService("set_frame_rate", &TSentisM100Node::SrvSetFrameRate, this);
    }

  bool SrvWriteRegister(lfd_vision::WriteRegister::Request &req, lfd_vision::WriteRegister::Response &res)
    {
      res.success= WriteRegister(req.address, req.value);
      return true;
    }

  bool SrvReadRegister(lfd_vision::ReadRegister::Request &req, lfd_vision::ReadRegister::Response &res)
    {
      res.value= ReadRegister(req.address);
      res.success= IsNoError("");
      return true;
    }

  bool SrvSetFrameRate(lfd_vision::SetFrameRate::Request &req, lfd_vision::SetFrameRate::Response &res)
    {
      res.success= SetFrameRate(req.frame_rate);
      return true;
    }

private:
  ros::NodeHandle     &node_;
  ros::ServiceServer  srv_write_register_;
  ros::ServiceServer  srv_read_register_;
  ros::ServiceServer  srv_set_frame_rate_;

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

  int init_fps, tcp_port, udp_port;
  std::string tcp_ip, udp_ip;
  node.param("init_fps",init_fps,1);
  node.param("tcp_ip",tcp_ip,std::string("192.168.0.10"));
  node.param("udp_ip",udp_ip,std::string("224.0.0.1"));
  node.param("tcp_port",tcp_port,10001);
  node.param("udp_port",udp_port,10002);

  /* Note: we set activate_callback because we do not use sentis_tof_m100 node,
    but use our own routine written below to observe using m100. */
  TFlowAnalyzerNode analyzer_node(node, /*activate_callback=*/false);
  // TFlowAnalyzerNode analyzer_node(node, /*activate_callback=*/true);

  TFlowFinder flow_finder;
  flow_finder.SetOptFlowWinSize(cv::Size(3,3));
  flow_finder.SetOptFlowSpdThreshold(3.0);
  flow_finder.SetErodeDilate(1);
  flow_finder.SetAmountRange(/*min=*/-1.0, /*max=*/-1.0);
  flow_finder.SetSpeedRange(/*min=*/-1.0, /*max=*/-1.0);
  // cv::BackgroundSubtractorMOG2 bkg_sbtr(/*int history=*/5, /*double varThreshold=*/5.0, /*bool detectShadows=*/true);

  cv::namedWindow("depth",1);
  cv::Mat img_depth, disp_img;
  img_depth.create(cv::Size(M100_IMAGE_WIDTH,M100_IMAGE_HEIGHT), CV_32FC1);

  ros::Publisher pub_cloud= node.advertise<sensor_msgs::PointCloud2>("/depth_non_filtered", 1);
  // ros::Publisher pub_cloud= node.advertise<pcl::PointCloud<pcl::PointXYZ> >("/depth_non_filtered", 1);
  TSentisM100Node tof_sensor(node);
  tof_sensor.Init(init_fps, /*data_format=*/XYZ_COORDS_DATA, tcp_ip.c_str(), udp_ip.c_str(), tcp_port, udp_port);
  // tof_sensor.PrintRegisters(0);
  // tof_sensor.PrintRegisters(1);
  // tof_sensor.SetFrameRate(40);
  double t_start= GetCurrentTime();
  ros::Rate loop_rate(40);  // 40 Hz
  for(int f(0); ros::ok(); ++f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if(!tof_sensor.GetDataAsPointCloud(cloud))  continue;

    const double max_depth(0.5);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-100.0, max_depth);
    pass.filter(*cloud);

    //TEST:Visualize depth
    tof_sensor.BufferToCVMat(&img_depth, /*img_amp=*/NULL);
    // flow_finder.Update(img_depth);
    // cv::cvtColor(img_depth,disp_img,CV_GRAY2RGB);
    // flow_finder.DrawFlow(disp_img, CV_RGB(0,255,255), /*len=*/1.0, /*thickness=*/3);
    // // for(int i(0);i<flow_finder.FlowElements().size();++i) std::cerr<<f<<" "<<flow_finder.FlowElements()[i]<<std::endl;
    // cv::imshow("depth", disp_img);
    // bkg_sbtr(img_depth,disp_img);
    cv::threshold(img_depth, disp_img, /*thresh=*/0.3, /*maxval=*/255.0, cv::THRESH_TOZERO_INV);
    cv::imshow("depth", disp_img);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud,cloud_msg);
    cloud_msg.header.frame_id= "tf_sentis_tof";
    cloud_msg.header.stamp= ros::Time::now();
    pub_cloud.publish(cloud_msg);
    analyzer_node.AnalyzePointCloud(cloud);  // Note: cloud may change

    if(f%100==0)
    {
      double duration= GetCurrentTime()-t_start;
      std::cerr<<"Duration: "<<duration<<std::endl;
      std::cerr<<"FPS: "<<double(100)/duration<<std::endl;
      t_start= GetCurrentTime();
    }

    cv::waitKey(1);  // To display cv::imshow
    ros::spinOnce();
    loop_rate.sleep();
  }
  tof_sensor.Sleep();


  // ros::spin();
  return 0;
}
//-------------------------------------------------------------------------------------------
