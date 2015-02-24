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
#include "pr2_lfd_vision/CreateScene.h"
#include "pr2_lfd_vision/RemoveScene.h"
#include "pr2_lfd_vision/LabeledPose.h"
#include "pr2_lfd_vision/LabeledPoseOptReq.h"
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

ros::Publisher PubLabeledPoseRevision;
// Camera model:
Imager::TCameraInfo CameraInfo;
// Map from a scene name to a pose estimator
std::map<std::string, TRayTracePoseEstimator> RayTracePoseEstimators;
// Map from a label to (scene name, object index)
std::map<std::string, std::pair<std::string, int> > LabelToSceneIdx;
// Current selection:
std::map<std::string, TRayTracePoseEstimator>::iterator CurrRTPoseEstimator;
int TargetObject(0);

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

bool RemoveScene(const std::string &scene)
{
  std::map<std::string, TRayTracePoseEstimator>::iterator itr= RayTracePoseEstimators.find(scene);
  if(itr!=RayTracePoseEstimators.end())
  {
    RayTracePoseEstimators.erase(itr);
    CurrRTPoseEstimator= RayTracePoseEstimators.begin();
    return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------

bool CreateScene(pr2_lfd_vision::CreateScene::Request &req, pr2_lfd_vision::CreateScene::Response &res)
{
  /*
  req.name
  req.models[]
    label
    primitives[]
      kind
      param
      pose
    initial_pose
  */
  std::cerr<<"Making scene: "<<req.name<<std::endl;
  RemoveScene(req.name);
  RayTracePoseEstimators[req.name]= TRayTracePoseEstimator();
  for(int i_model(0),i_model_end(req.models.size()); i_model<i_model_end; ++i_model)
  {
    const pr2_lfd_vision::RayTraceModel &model(req.models[i_model]);
    std::cerr<<"  adding: "<<model.label<<std::endl;
    TRayTraceModel object;
    for(int i_prim(0),i_prim_end(model.primitives.size()); i_prim<i_prim_end; ++i_prim)
    {
      const pr2_lfd_vision::RayTracePrimitive &prim(model.primitives[i_prim]);
      TRayTraceModel::TPrimitive primitive;
      primitive.Kind= StrToRTPrimitiveKind(prim.kind);
      for(int i_param(0),i_param_end(prim.param.size()); i_param<i_param_end; ++i_param)
        primitive.Param[i_param]= prim.param[i_param];
      GPoseToX(prim.pose, primitive.Pose);
      object.Primitives.push_back(primitive);
    }  // for each i_prim
    double pose[]= {0.0,0.0,0.0, 0.0,0.0,0.0,1.0};
    GPoseToX(model.initial_pose, pose);
    if(pose[3]==0.0&&pose[4]==0.0&&pose[5]==0.0&&pose[6]==0.0)  pose[6]= 1.0;
    int idx= RayTracePoseEstimators[req.name].AddObject(object,pose);
    LabelToSceneIdx[model.label]= std::pair<std::string, int>(req.name,idx);
  }  // for each i_model

  RayTracePoseEstimators[req.name].SetCameraInfo(CameraInfo);
  CurrRTPoseEstimator= RayTracePoseEstimators.begin();
  return true;
}
//-------------------------------------------------------------------------------------------

bool RemoveScene(pr2_lfd_vision::RemoveScene::Request &req, pr2_lfd_vision::RemoveScene::Response &res)
{
  if(req.name=="")
  {
    RayTracePoseEstimators.clear();
    CurrRTPoseEstimator= RayTracePoseEstimators.begin();
    LabelToSceneIdx.clear();
    std::cerr<<"Removed: all scenes"<<std::endl;
  }
  else
  {
    RemoveScene(req.name);
    std::cerr<<"Removed: "<<req.name<<std::endl;
    // FIXME: We need to remove elements of LabelToSceneIdx whose scene name is req.name
    std::cerr<<"FIXME: need to remove elements of LabelToSceneIdx whose scene name is "<<req.name<<std::endl;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud= ConvertROSMsgToPointCloud(msg);

  cv::Mat rgb_img, depth_img, normal_img;
  ConvertPointCloudToRGBDImages(cloud, rgb_img, depth_img);
  ConvertPointCloudToNormalImage(cloud, normal_img, /*FS=*/7);

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
  else if(CurrRTPoseEstimator!=RayTracePoseEstimators.end()
      && CurrRTPoseEstimator->second.HandleKeyEvent(TargetObject,c))  {}
  else if(CurrRTPoseEstimator!=RayTracePoseEstimators.end()
      && c==' ')  CurrRTPoseEstimator->second.OptimizeXYZ(TargetObject, depth_img, normal_img, 7, 7);
  else if(c=='0' || c=='1' || c=='2' || c=='3')
  {
    switch(c)
    {
    case '0': TargetObject= 0; break;
    case '1': TargetObject= 1; break;
    case '2': TargetObject= 2; break;
    case '3': TargetObject= 3; break;
    }
    std::cerr<<"Selected: "<<TargetObject<<std::endl;
  }
  else if(c=='>')
  {
    if(RayTracePoseEstimators.size()>0)
    {
      ++CurrRTPoseEstimator;
      if(CurrRTPoseEstimator==RayTracePoseEstimators.end())
        CurrRTPoseEstimator= RayTracePoseEstimators.begin();
    }
  }
  else if(c=='<')
  {
    if(RayTracePoseEstimators.size()>0)
    {
      if(CurrRTPoseEstimator==RayTracePoseEstimators.begin())
        CurrRTPoseEstimator= RayTracePoseEstimators.end();
      --CurrRTPoseEstimator;
    }
  }

  for(std::map<std::string, TRayTracePoseEstimator>::iterator
        itr(RayTracePoseEstimators.begin()),itr_end(RayTracePoseEstimators.end());
          itr!=itr_end; ++itr)
    itr->second.Render(depth_img, normal_img);

  // cv::imshow("rgb", rgb_img);
  // cv::imshow("depth", depth_img);
  cv::imshow("normal", normal_img);
}
//-------------------------------------------------------------------------------------------

void CallbackLabeledPose(const pr2_lfd_vision::LabeledPoseConstPtr &msg)
{
  // Read the message:
  std::pair<std::string, int> scene_idx= LabelToSceneIdx[msg->label];
  double pose[7];
  GPoseToX(msg->pose, pose);

  // Improve pose by ray tracing pose estimation:
  RayTracePoseEstimators[scene_idx.first].SetPose(scene_idx.second, pose);
  double position_revised[3], eval_revised[2];
  RayTracePoseEstimators[scene_idx.first].OptimizeXYZ(scene_idx.second, DepthImg, NormalImg, 7, 7, position_revised, eval_revised);
  std::cerr<<"Revised ("<<msg->label<<" in "<<scene_idx.first<<"):";
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

void CallbackLabeledPoseOptReq(const pr2_lfd_vision::LabeledPoseOptReqConstPtr &msg)
{
  // Read the message:
  std::pair<std::string, int> scene_idx= LabelToSceneIdx[msg->lpose.label];
  double pose[7];
  GPoseToX(msg->lpose.pose, pose);

  double position_revised[3], eval_revised[2];
  if(msg->type=="lin2d")
  {
    double axis_1[3], axis_2[3];
    GPointToP(msg->axis1, axis_1);
    GPointToP(msg->axis2, axis_2);
    double range_1(msg->range[0]), range_2(msg->range[1]), n_div(msg->num_div);
    double weight_depth(msg->weight_depth), weight_normal(msg->weight_normal);
    // std::cerr<<"axis_1:"<<axis_1[0]<<", "<<axis_1[1]<<", "<<axis_1[2]<<std::endl;
    // std::cerr<<"axis_2:"<<axis_2[0]<<", "<<axis_2[1]<<", "<<axis_2[2]<<std::endl;

    // Improve pose by ray tracing pose estimation:
    RayTracePoseEstimators[scene_idx.first].SetPose(scene_idx.second, pose);
    RayTracePoseEstimators[scene_idx.first].OptimizeLin2D(
        scene_idx.second, DepthImg, NormalImg, 7, 7,
        axis_1, axis_2,
        range_1, range_2, n_div,
        weight_depth, weight_normal,
        /*opt_12=*/NULL, position_revised, eval_revised);

    std::cerr<<"Revised lin2d ("<<msg->lpose.label<<" in "<<scene_idx.first<<"):";
    for(int d(0); d<3; ++d)
    {
      std::cerr<<" "<<pose[d]-position_revised[d];
      pose[d]= position_revised[d];
    }
    std::cerr<<" # "<<eval_revised[0]<<" "<<eval_revised[1]<<std::endl;
  }

  // Publish the revised pose:
  pr2_lfd_vision::LabeledPoseRevision msg_pose_revision;
  msg_pose_revision.lpose.header.seq= Sequence;  ++Sequence;
  msg_pose_revision.lpose.header.stamp= PointCloudHeader.stamp;
  msg_pose_revision.lpose.header.frame_id= PointCloudHeader.frame_id;
  msg_pose_revision.lpose.label= msg->lpose.label;
  XToGPose(pose, msg_pose_revision.lpose.pose);
  msg_pose_revision.errors.resize(2);
  msg_pose_revision.errors[0]= eval_revised[0];
  msg_pose_revision.errors[1]= eval_revised[1];
  PubLabeledPoseRevision.publish(msg_pose_revision);
}
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  ros::init(argc, argv, "rt_pose_estimator");
  ros::NodeHandle node("~");

  std::string point_cloud_in;
  node.param("in_points", point_cloud_in, std::string("/camera/depth/points_xyzrgb"));
  std::string labeled_pose_in, labeled_pose_optreq_in, labeled_pose_out;
  node.param("in_pose", labeled_pose_in, std::string("labeled_pose"));
  node.param("in_poseoptreq", labeled_pose_optreq_in, std::string("labeled_pose_optreq"));
  node.param("out_pose", labeled_pose_out, std::string("labeled_pose_revision"));

  // cv::namedWindow("rgb",1);
  // cv::namedWindow("depth",1);
  cv::namedWindow("normal",1);

  /*############Setup ray tracing############*/
  // Create object
  using namespace Imager;

  // Create camera model
  //   TODO: These values should be taken from a topic like
  //   /camera/depth/camera_info (Use height, width, and P)
  CameraInfo.Width= 640;
  CameraInfo.Height= 480;
  CameraInfo.Fx= 540.916992;
  CameraInfo.Fy= 542.752869;
  CameraInfo.Cx= 317.022348;
  CameraInfo.Cy= 230.86987;
  CurrRTPoseEstimator= RayTracePoseEstimators.begin();

  #if 0  // For DEBUG
  RayTracePoseEstimators["test"]= TRayTracePoseEstimator();
  TRayTraceModel container;
  #if 0  // Coke can test (b54)
  TRayTraceModel::TPrimitive container_1;
  container_1.Kind= rtpkCylinder;
  container_1.Param[0]= 0.033;
  container_1.Param[1]= 0.12;
  for(int d(0);d<7;++d)  container_1.Pose[d]= 0.0;
  container_1.Pose[2]= 0.06;
  container_1.Pose[6]= 1.0;
  container.Primitives.push_back(container_1);
  #endif
  #if 1  // Mag cup test (b99)
  TRayTraceModel::TPrimitive container_1;
  container_1.Kind= rtpkTube;
  container_1.Param[0]= 0.04;
  container_1.Param[1]= 0.037;
  container_1.Param[2]= 0.10;
  container_1.Param[3]= 0.0;
  container_1.Param[4]= 0.0;
  for(int d(0);d<7;++d)  container_1.Pose[d]= 0.0;
  container_1.Pose[2]= 0.05;
  container_1.Pose[6]= 1.0;
  container.Primitives.push_back(container_1);
  TRayTraceModel::TPrimitive container_2;
  container_2.Kind= rtpkCylinder;
  container_2.Param[0]= 0.04;
  container_2.Param[1]= 0.01;
  for(int d(0);d<7;++d)  container_2.Pose[d]= 0.0;
  container_2.Pose[2]= 0.005;
  container_2.Pose[6]= 1.0;
  container.Primitives.push_back(container_2);
  #endif
  double pose[]= {0.0461,0.1142,0.6272, 0.0,0.0,0.0,1.0};
  int idx= RayTracePoseEstimators["test"].AddObject(container,pose);
#if 0  // Add second object:
container.Primitives.clear();
TRayTraceModel::TPrimitive container_3;
container_3.Kind= rtpkCylinder;
container_3.Param[0]= 0.033;
container_3.Param[1]= 0.12;
for(int d(0);d<7;++d)  container_3.Pose[d]= 0.0;
container_3.Pose[2]= 0.06;
container_3.Pose[6]= 1.0;
container.Primitives.push_back(container_3);
RayTracePoseEstimators["test"].AddObject(container,pose);
#endif
  // LabelToSceneIdx["b99"]= std::pair<std::string, int>("test",idx);  // Mag cup test
  LabelToSceneIdx["b54"]= std::pair<std::string, int>("test",idx);  // Coke can test
  // cylinder->Move();
  // cylinder->RotateX(90.0);
  // cylinder->RotateY(0.0);
  // cylinder->RotateZ(0.0);
  RayTracePoseEstimators["test"].SetCameraInfo(CameraInfo);
  CurrRTPoseEstimator= RayTracePoseEstimators.begin();
  #endif  // TEST

  /*############Done############*/


  PubLabeledPoseRevision= node.advertise<pr2_lfd_vision::LabeledPoseRevision>(labeled_pose_out, 1);
  ros::ServiceServer srv_create= node.advertiseService("create_scene", &CreateScene);
  ros::ServiceServer srv_remove= node.advertiseService("remove_scene", &RemoveScene);

  ros::Subscriber sub_point_cloud= node.subscribe(point_cloud_in, 1, &CallbackPointCloud);
  ros::Subscriber sub_labeled_pose= node.subscribe(labeled_pose_in, 1, &CallbackLabeledPose);
  ros::Subscriber sub_labeled_pose_optreq= node.subscribe(labeled_pose_optreq_in, 1, &CallbackLabeledPoseOptReq);
  ros::spin();

  return 0;
}
//-------------------------------------------------------------------------------------------
