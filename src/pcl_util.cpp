//-------------------------------------------------------------------------------------------
/*! \file    pcl_util.cpp
    \brief   PCL utility
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Oct.17, 2014
    \version 0.2
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ConvertROSMsgToPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  //Convert the point cloud to depth image and rgb image
  // pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*msg, *cloud);

  // // Remove plains
  // bool res= RemovePlains<pcl::PointXYZRGB>(cloud,
                // /*non_planar_points_ratio=*/0.9,
                // /*ransac_dist_thresh=*/0.01,
                // /*ransac_max_iterations=*/100);

  // ref.  template<typename CloudT> void toROSMsg(const CloudT& cloud, sensor_msgs::Image& msg)
  // http://docs.pointclouds.org/1.1.0/conversions_8h_source.html#l00254
  if(cloud->width == 0 && cloud->height == 0)
    throw std::runtime_error("Needs to be a dense like cloud!!");
  else
  {
    if(cloud->points.size() != cloud->width * cloud->height)
      throw std::runtime_error("The width and height do not match the cloud size!");
  }

  return cloud;
}
//-------------------------------------------------------------------------------------------

/* Get RGB image and depth image from a Point Cloud.
    cf. pcl::PointXYZRGB Struct Reference
    http://docs.pointclouds.org/1.7.1/structpcl_1_1_point_x_y_z_r_g_b.html  */
void ConvertPointCloudToRGBDImages(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &rgb_img, cv::Mat &depth_img)
{
  rgb_img.create(cloud->height, cloud->width, CV_8UC3);
  depth_img.create(cloud->height, cloud->width, CV_32FC1);
  // xyz_img.create(cloud->height, cloud->width, CV_32FC3);
  for(size_t y(0); y<cloud->height; ++y)
  {
    for(size_t x(0); x<cloud->width; ++x)
    {
      const pcl::PointXYZRGB &pt(cloud->at(x,y));
      uint32_t rgb = *reinterpret_cast<const int*>(&pt.rgb);
      uint8_t r = (rgb >> 16) & 0x0000ff;
      uint8_t g = (rgb >> 8)  & 0x0000ff;
      uint8_t b = (rgb)       & 0x0000ff;
      rgb_img.at<cv::Vec3b>(y,x)= cv::Vec3b(b,g,r);
      depth_img.at<float>(y,x)= pt.z;
      // xyz_img.at<cv::Vec3f>(y,x)= cv::Vec3f(pt.x+1.0,pt.y+1.0,pt.z*5.0);
    }
  }
}
//-------------------------------------------------------------------------------------------

/* Estimate normal and store it as an image.
    FS: Window size for computing normal (should be odd).  */
void ConvertPointCloudToNormalImage(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    cv::Mat &normal_img, int FS)
{
  int FSh((FS-1)/2);
  normal_img.create(cloud->height, cloud->width, CV_32FC3);
  for(int y(0); y<cloud->height; ++y)
  {
    for(int x(0); x<cloud->width; ++x)
    {
      bool no_data(false);
      if(x<FSh || cloud->width-FSh<=x || y<FSh || cloud->height-FSh<=y)  no_data= true;
      else if(IsInvalid(cloud->at(x-FSh,y)) || IsInvalid(cloud->at(x+FSh,y))
            || IsInvalid(cloud->at(x,y-FSh)) || IsInvalid(cloud->at(x,y+FSh)))  no_data= true;
      if(no_data)
      {
        normal_img.at<cv::Vec3f>(y,x)= cv::Vec3f(0.0,0.0,0.0);
        continue;
      }
      #if 1  // Simple calculation (fast)
      Eigen::Vector3f ax1= cloud->at(x+FSh,y).getVector3fMap() - cloud->at(x-FSh,y).getVector3fMap();
      Eigen::Vector3f ax2= cloud->at(x,y+FSh).getVector3fMap() - cloud->at(x,y-FSh).getVector3fMap();
      #endif
      #if 0  // Complicated calculation (slow, but robust for noise)
      int num1(0), num2(0);
      Eigen::Vector3f ax1(0.0,0.0,0.0);
      Eigen::Vector3f ax2(0.0,0.0,0.0);
      int xs(x-FSh), ys(y-FSh);
      for(int fs(1); fs<=FS; fs+=2)
      {
        for(int i(0); i<FS-fs; ++i)
          for(int j(0); j<FS; ++j)
            if(IsValid(cloud->at(xs+i,ys+j)) && IsValid(cloud->at(xs+i+fs,ys+j)))
            {
              ax1+= cloud->at(xs+i+fs,ys+j).getVector3fMap() - cloud->at(xs+i,ys+j).getVector3fMap();
              ++num1;
            }
        for(int i(0); i<FS; ++i)
          for(int j(0); j<FS-fs; ++j)
            if(IsValid(cloud->at(xs+i,ys+j)) && IsValid(cloud->at(xs+i,ys+j+fs)))
            {
              ax2+= cloud->at(xs+i,ys+j+fs).getVector3fMap() - cloud->at(xs+i,ys+j).getVector3fMap();
              ++num2;
            }
      }
      ax1/= (float)num1;
      ax2/= (float)num2;
      #endif

      // Eigen::Vector3f normal= ax1.cross(ax2);
      Eigen::Vector3f normal= ax2.cross(ax1);
      normal.normalize();
      // normal_img.at<cv::Vec3f>(y,x)= cv::Vec3f(normal[0],normal[1],normal[2]);
      cv::Vec3f col;
      GetVisualNormal(normal[0],normal[1],normal[2], col(0),col(1),col(2));
      normal_img.at<cv::Vec3f>(y,x)= col;
    }
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

