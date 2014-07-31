//-------------------------------------------------------------------------------------------
/*! \file    cyl_detector_node.cpp
    \brief   Cylinder detector node.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jul.28, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_trick/cyl_detector.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <pcl/ros/conversions.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
TPCLViewer PCLViewer;
bool IsCameraSetup(false);
double MarkerSize(0.044);
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if(PCLViewer.IsStopped())  return;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud_org);

  if(!IsCameraSetup)
  {
    PCLViewer.AutoCamera(cloud_org);
    IsCameraSetup= true;
  }

  ExecuteDetection(PCLViewer, cloud_org);

  PCLViewer.SpinOnce(100);
}
//-------------------------------------------------------------------------------------------

void ARMarkersCallback(const ar_track_alvar::AlvarMarkers &msg)
{
  //FIXME: compare the marker's frame-id with the point clouds' frame-id
  using namespace trick::detail;
  // std::cerr<<"msg.markers.size(): "<<msg.markers.size()<<std::endl;
  for(int i(0); i<msg.markers.size(); ++i)
  {
    int id= msg.markers[i].id;
    ARMarkers[id].Position[0]= msg.markers[i].pose.pose.position.x;
    ARMarkers[id].Position[1]= msg.markers[i].pose.pose.position.y;
    ARMarkers[id].Position[2]= msg.markers[i].pose.pose.position.z;
    ARMarkers[id].Orientation[0]= msg.markers[i].pose.pose.orientation.x;
    ARMarkers[id].Orientation[1]= msg.markers[i].pose.pose.orientation.y;
    ARMarkers[id].Orientation[2]= msg.markers[i].pose.pose.orientation.z;
    ARMarkers[id].Orientation[3]= msg.markers[i].pose.pose.orientation.w;
    ARMarkers[id].Size= MarkerSize;
    // std::cerr<<"#id: "<<id<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  using namespace trick::detail;
  ros::init(argc, argv, "pcd_write_node");
  ros::NodeHandle node("~");

  std::string pcd_in;
  node.param("pcd_in",pcd_in,std::string(""));
  node.param("marker_size",MarkerSize,0.044);

  if(pcd_in!="")
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(pcd_in, *cloud_org);
    PCLViewer.AutoCamera(cloud_org);
    ExecuteDetection(PCLViewer, cloud_org);
    while(!PCLViewer.IsStopped()) PCLViewer.SpinOnce(100);
    return 0;
  }

  std::cerr<<"Press 'v' to start"<<std::endl;

  ros::Subscriber sub_rgbd = node.subscribe("/camera/depth/points_xyzrgb", 1, &PointCloudCallback);
  ros::Subscriber sub_armarkers = node.subscribe("/ar_pose_marker", 1, &ARMarkersCallback);
  ros::spin();
  return 0;
}
//-------------------------------------------------------------------------------------------
