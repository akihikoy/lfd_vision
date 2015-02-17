//-------------------------------------------------------------------------------------------
/*! \file    rt_pose_estimator_node.cpp
    \brief   Pose estimation using ray tracing
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/raytrace/pose_estimator.h"
#include "pr2_lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/LabeledPose.h"
#include "pr2_lfd_vision/LabeledPoseRevision.h"
//-------------------------------------------------------------------------------------------
#include <string>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // filters
#include <ros/ros.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

TRayTracePoseEstimator RayTracePoseEstimator;
ros::Publisher PubLabeledPoseRevision;
std::map<std::string, int> LabelToIdx;  // Map from a label to an index

cv::Mat DepthImg, NormalImg;
std_msgs::Header PointCloudHeader;
int Sequence(0);

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud= ConvertROSMsgToPointCloud(msg);

  cv::Mat rgb_img, depth_img, normal_img;
  ConvertPointCloudToRGBDImages(cloud, rgb_img, depth_img);
  ConvertPointCloudToNormalImage(cloud, normal_img, /*FS=*/9);

  // cv::medianBlur(normal_img, normal_img, 5);
  // cv::medianBlur(normal_img, normal_img, 5);
  // normal_img*= 200.0;
  // normal_img.convertTo(normal_img,CV_8UC3);
  // cv::medianBlur(normal_img, normal_img, 9);
  // cv::Mat normal_img2;
  // cv::bilateralFilter(normal_img, normal_img2, 21, 21*2, 21/2);
  // normal_img= normal_img2;
  // cv::blur(normal_img, normal_img, cv::Size(11, 21), cv::Point(-1,-1));

  // normal_img*= 255.0;
  // normal_img.convertTo(normal_img,CV_8UC3);

  // depth_img*= 255.0;
  // depth_img.convertTo(depth_img,CV_8UC1);

  depth_img.copyTo(DepthImg);
  normal_img.copyTo(NormalImg);
  PointCloudHeader= msg->header;

  int c(cv::waitKey(10));
  if(c=='\x1b'||c=='q')  ros::shutdown();
  else if(RayTracePoseEstimator.HandleKeyEvent(0,c))  {}
  else if(c==' ')  RayTracePoseEstimator.OptimizeXYZ(0, depth_img, normal_img, 7, 7);

  RayTracePoseEstimator.Render(depth_img, normal_img);

  // cv::imshow("rgb", rgb_img);
  // cv::imshow("depth", depth_img);
  cv::imshow("normal", normal_img);
}
//-------------------------------------------------------------------------------------------

void CallbackLabeledPose(const pr2_lfd_vision::LabeledPoseConstPtr &msg)
{
  // Read the message:
  int idx= LabelToIdx[msg->label];
  double pose[7];
  GPoseToX(msg->pose, pose);

  // Improve pose by ray tracing pose estimation:
  RayTracePoseEstimator.SetPose(idx, pose);
  double position_revised[3], eval_revised[2];
  RayTracePoseEstimator.OptimizeXYZ(idx, DepthImg, NormalImg, 7, 7, position_revised, eval_revised);
  std::cerr<<"Revised:";
  for(int d(0); d<3; ++d)
  {
    std::cerr<<" "<<pose[d]-position_revised[d];
    pose[d]= position_revised[d];
  }
  std::cerr<<" # "<<eval_revised[0]<<" "<<eval_revised[1]<<std::endl;

  // Publish the revised pose:
  pr2_lfd_vision::LabeledPoseRevision msg_pose_revision;
  msg_pose_revision.lpose.header.seq= Sequence;  ++Sequence;
  msg_pose_revision.lpose.header.stamp= PointCloudHeader.stamp;
  msg_pose_revision.lpose.header.frame_id= PointCloudHeader.frame_id;
  msg_pose_revision.lpose.label= msg->label;
  XToGPose(pose, msg_pose_revision.lpose.pose);
  msg_pose_revision.errors.resize(2);
  msg_pose_revision.errors[0]= eval_revised[0];
  msg_pose_revision.errors[1]= eval_revised[1];
  PubLabeledPoseRevision.publish(msg_pose_revision);
}
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  ros::init(argc, argv, "rt_pose_estimator_node");
  ros::NodeHandle node("~");

  std::string point_cloud_in;
  node.param("in_points", point_cloud_in, std::string("/camera/depth/points_xyzrgb"));
  std::string labeled_pose_in, labeled_pose_out;
  node.param("in_pose", labeled_pose_in, std::string("/labeled_pose"));
  node.param("out_pose", labeled_pose_out, std::string("/labeled_pose_revision"));

  // cv::namedWindow("rgb",1);
  // cv::namedWindow("depth",1);
  cv::namedWindow("normal",1);

  /*############Setup ray tracing############*/
  // Create object
  using namespace Imager;

  double pose[]= {0.0461,0.1142,0.6272, 0.0,0.0,0.0,1.0};
  int idx= RayTracePoseEstimator.AddObject(TRayTraceModel(),pose);
  // LabelToIdx["b99"]= idx;  // Mag cup test
  LabelToIdx["b54"]= idx;  // Coke can test
  // cylinder->Move();
  // cylinder->RotateX(90.0);
  // cylinder->RotateY(0.0);
  // cylinder->RotateZ(0.0);

  // Create camera model
  //   TODO: These values should be taken from a topic like
  //   /camera/depth/camera_info (Use height, width, and P)
  Imager::TCameraInfo cam;
  cam.Width= 640;
  cam.Height= 480;
  cam.Fx= 540.916992;
  cam.Fy= 542.752869;
  cam.Cx= 317.022348;
  cam.Cy= 230.86987;
  RayTracePoseEstimator.SetCameraInfo(cam);
  /*############Done############*/


  PubLabeledPoseRevision= node.advertise<pr2_lfd_vision::LabeledPoseRevision>(labeled_pose_out, 1);

  ros::Subscriber sub_point_cloud= node.subscribe(point_cloud_in, 1, &CallbackPointCloud);
  ros::Subscriber sub_labeled_pose= node.subscribe(labeled_pose_in, 1, &CallbackLabeledPose);
  ros::spin();

  return 0;
}
//-------------------------------------------------------------------------------------------
