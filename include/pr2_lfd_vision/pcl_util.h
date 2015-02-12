//-------------------------------------------------------------------------------------------
/*! \file    pcl_util.h
    \brief   PCL utility
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Oct.17, 2014
    \version 0.2
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef pcl_util_h
#define pcl_util_h
//-------------------------------------------------------------------------------------------
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <opencv2/core/core.hpp>
// #include <geometry_msgs/Pose.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline t_value Sq(const t_value &val)
{
  return val*val;
}
//-------------------------------------------------------------------------------------------

template<typename t_point>
inline bool IsValid(const t_point &pt)
{
  // pcl_isfinite is better?
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}
template<>
inline bool IsValid<float>(const float &x)
{
  return std::isfinite(x);
}
template<>
inline bool IsValid<double>(const double &x)
{
  return std::isfinite(x);
}
//-------------------------------------------------------------------------------------------

template<typename t_point>
inline bool IsInvalid(const t_point &pt)
{
  // pcl_isfinite is better?
  return !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z);
}
template<>
inline bool IsInvalid<float>(const float &x)
{
  return !std::isfinite(x);
}
template<>
inline bool IsInvalid<double>(const double &x)
{
  return !std::isfinite(x);
}
//-------------------------------------------------------------------------------------------

/* Pose to Eigen::Affine3d.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline Eigen::Affine3d XToEigMat(const t_value x[])
{
  Eigen::Affine3d res;
  res= Eigen::Translation3d(x[0],x[1],x[2])
          * Eigen::Quaterniond(x[6], x[3],x[4],x[5]);
  return res;
}
/* Eigen::Affine3d to pose.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void EigMatToX(const Eigen::Affine3d T, t_value x[7])
{
  const Eigen::Vector3d p= T.translation();
  x[0]= p[0];
  x[1]= p[1];
  x[2]= p[2];
  Eigen::Quaterniond q(T.rotation());
  x[3]= q.x();
  x[4]= q.y();
  x[5]= q.z();
  x[6]= q.w();
}
//-------------------------------------------------------------------------------------------

// Convert x to geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_value, typename t_pose>
inline void XToGPose(const t_value x[7], t_pose &pose)
{
  pose.position.x= x[0];
  pose.position.y= x[1];
  pose.position.z= x[2];
  pose.orientation.x= x[3];
  pose.orientation.y= x[4];
  pose.orientation.z= x[5];
  pose.orientation.w= x[6];
}
// Convert x to geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_pose, typename t_value>
inline t_pose XToGPose(const t_value x[7])
{
  t_pose pose;
  XToGPose<t_value,t_pose>(x, pose);
  return pose;
}
//-------------------------------------------------------------------------------------------

// Convert geometry_msgs/Pose to x; usually, t_pose==geometry_msgs::Pose
template <typename t_pose, typename t_value>
inline void GPoseToX(const t_pose &pose, t_value x[7])
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

template <typename t_1, typename t_2>
inline void GetVisualNormal(
    const t_1 &nx, const t_1 &ny, const t_1 &nz,
    t_2 &r, t_2 &g, t_2 &b)
{
  r= 0.5*(1.0-nx);
  g= 0.5*(1.0-ny);
  b= 0.5*(1.0-nz);
}
//-------------------------------------------------------------------------------------------


pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ConvertROSMsgToPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg);

/* Get RGB image and depth image from a Point Cloud.
    cf. pcl::PointXYZRGB Struct Reference
    http://docs.pointclouds.org/1.7.1/structpcl_1_1_point_x_y_z_r_g_b.html  */
void ConvertPointCloudToRGBDImages(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &rgb_img, cv::Mat &depth_img);

/* Estimate normal and store it as an image.
    FS: Window size for computing normal (should be odd).  */
void ConvertPointCloudToNormalImage(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &normal_img, int FS=9);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // pcl_util_h
//-------------------------------------------------------------------------------------------
