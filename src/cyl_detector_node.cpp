//-------------------------------------------------------------------------------------------
/*! \file    cyl_detector_node.cpp
    \brief   Cylinder detector node.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jul.28, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/cyl_detector.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
TPCLViewer PCLViewer;
bool IsCameraSetup(false);
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

void ARMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers &msg)
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
  ros::init(argc, argv, "cyl_detector");
  ros::NodeHandle node("~");

  std::string pcd_in;
  node.param("pcd_in",pcd_in,std::string(""));
  node.param("marker_size",MarkerSize,0.048);

  if(pcd_in!="")
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(pcd_in, *cloud_org);
    PCLViewer.AutoCamera(cloud_org);
    // ExecuteDetection(PCLViewer, cloud_org);

    double base_frame[7]= {0,0,0, 0,0,0,1};
    TARMarker  base_ar_marker;
    XToARMarker(base_frame, base_ar_marker);
    base_ar_marker.Size= MarkerSize;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  base_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    DetectCylinder(
        PCLViewer,
        cloud_org,
        /*focused_analysis=*/true,
        base_ar_marker,
        /*ref_ar_marker_id=*/-1,
        /*ref_ar_marker=*/base_ar_marker,
        base_point_cloud,
        /*add_to_base_point_cloud=*/false);

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
