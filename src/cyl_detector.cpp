//-------------------------------------------------------------------------------------------
/*! \file    cyl_detector.cpp
    \brief   Cylinder detector.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jul.28, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_trick/cyl_detector.h"
#include <list>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace trick
{
// using namespace std;
// using namespace boost;

//-------------------------------------------------------------------------------------------
namespace detail
{
std::map<int,TARMarker>  ARMarkers;

enum TDetectorMode {dmRunning=0, dmVisualizingCurrCloud, dmPause};
// static TDetectorMode  DetectorMode(dmRunning);
static TDetectorMode  DetectorMode(dmVisualizingCurrCloud);
static bool  VisInitRequired(false);
static bool  FocusingOnSingleCylinder(false);
static Eigen::Vector3d  PickedPoint(0.0,0.0,0.0);
static const int   MaxCloudSequence(10);
static std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  CloudSequence;
static std::list<TARMarker>                               BaseARMarkerSequence;
static bool                                                         PlayingItrChanged(false);
static std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator  PlayingCloudItr;
static std::list<TARMarker>::iterator                               PlayingBaseARMarkerItr;
}  // detail
//-------------------------------------------------------------------------------------------
static bool  VerbosePrint(false);
static const int   NumIndexedColors(6);
static const float IndexedColors[NumIndexedColors][3]= {{255,0,0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
//-------------------------------------------------------------------------------------------

void KeyboardEventCallback(
    const pcl::visualization::KeyboardEvent &event,
    void *viewer_void)
{
  using namespace trick::detail;

  if(event.keyDown())
  {
    if(event.getKeySym()=="space" || event.getKeySym()=="p")
    {
      DetectorMode= (DetectorMode!=dmPause ? dmPause : dmRunning);
      if(DetectorMode==dmPause)
      {
        std::cerr<<"Cylinder detection: pause (press ' ' to resume)"<<std::endl;
        PlayingItrChanged= false;
        PlayingCloudItr= CloudSequence.end();
        PlayingBaseARMarkerItr= BaseARMarkerSequence.end();
        if(CloudSequence.begin()!=CloudSequence.end())
        {
          --PlayingCloudItr;
          --PlayingBaseARMarkerItr;
        }
      }
      else
        std::cerr<<"Cylinder detection: resume"<<std::endl;
    }
    else if(event.getKeySym()=="v")
    {
      DetectorMode= (DetectorMode!=dmVisualizingCurrCloud ? dmVisualizingCurrCloud : dmRunning);
      if(DetectorMode==dmVisualizingCurrCloud)
        std::cerr<<"Cylinder detection: pause/visualizing current cloud (press 'v' to resume)"<<std::endl;
      else
        std::cerr<<"Cylinder detection: resume"<<std::endl;
    }
    else if(event.getKeySym()=="Right")
    {
      if(PlayingCloudItr!=CloudSequence.end())
      {
        ++PlayingCloudItr;
        ++PlayingBaseARMarkerItr;
        if(PlayingCloudItr==CloudSequence.end())
        {
          --PlayingCloudItr;
          --PlayingBaseARMarkerItr;
        }
        else
        {
          std::cerr<<"Next snapshot"<<std::endl;
          PlayingItrChanged= true;
        }
      }
    }
    else if(event.getKeySym()=="Left")
    {
      if(PlayingCloudItr!=CloudSequence.begin())
      {
        --PlayingCloudItr;
        --PlayingBaseARMarkerItr;
        std::cerr<<"Previous snapshot"<<std::endl;
        PlayingItrChanged= true;
      }
    }
    else if(event.getKeySym()=="i")
    {
      VisInitRequired= true;
    }
    else if(event.getKeySym()=="r")
    {
      FocusingOnSingleCylinder= false;
    }
    else if(event.getKeySym()=="d")
    {
      VerbosePrint= !VerbosePrint;
    }
    else
    {
      std::cerr<<"Key press: \""<<event.getKeySym()<<"\""<<std::endl;
    }
  }
}
//-------------------------------------------------------------------------------------------

void MouseEventCallback(
    const pcl::visualization::MouseEvent &event,
    void *viewer_void)
{
  // if(event.getButton()==pcl::visualization::MouseEvent::LeftButton &&
      // event.getType()==pcl::visualization::MouseEvent::MouseButtonRelease)
  // {
    // std::cerr<<"Left mouse button released at position ("<<event.getX()<<", "<<event.getY()<<")"<<std::endl;
  // }
}
//-------------------------------------------------------------------------------------------

// Press shift and click
void PointpickEventCallback(
    const pcl::visualization::PointPickingEvent &event,
    void *viewer_void)
{
  using namespace trick::detail;

  // if(event.getPointIndex()==-1)  return;
  // {
    // return;
  // }
  std::cerr<<"Point picked: "<<event.getPointIndex()<<std::endl;
  float x,y,z;
  event.getPoint(x,y,z);
  std::cerr<<"  ..at: "<<x<<" "<<y<<" "<<z<<std::endl;
  FocusingOnSingleCylinder= true;
  PickedPoint[0]= x;
  PickedPoint[1]= y;
  PickedPoint[2]= z;
}
//-------------------------------------------------------------------------------------------

void ExecuteDetection(
    TPCLViewer &pcl_viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_org)
{
  using namespace trick::detail;

  if(DetectorMode!=dmPause && VisInitRequired)
  {
    pcl_viewer.RemoveAll();
    pcl_viewer.AutoCamera(cloud_org);
    VisInitRequired= false;
  }

  if(DetectorMode==dmPause)
  {
    if(PlayingItrChanged)
    {
      DetectCylinder(
          pcl_viewer,
          *PlayingCloudItr,
          FocusingOnSingleCylinder,
          PickedPoint,
          *PlayingBaseARMarkerItr);
      PlayingItrChanged= false;
    }
    return;
  }
  else if(DetectorMode==dmVisualizingCurrCloud)
  {
    pcl_viewer.RemoveAll();
    pcl_viewer.AddPointCloud(cloud_org,"cloud1",1);
  }
  else if(DetectorMode==dmRunning)
  {
    // Get a closest marker from PickedPoint
    int closest_m_id(-1);
    double distance(1.0e10);
    for(std::map<int,TARMarker>::const_iterator itr(ARMarkers.begin()),last(ARMarkers.end()); itr!=last; ++itr)
    {
      double d= (Eigen::Vector3d(itr->second.Position)-PickedPoint).norm();
      if(d<distance)  {distance= d;  closest_m_id= itr->first;}
    }
    TARMarker  base_ar_marker;
    if(closest_m_id>=0)
      base_ar_marker= ARMarkers[closest_m_id];

    // Store the cloud and the marker
    CloudSequence.push_back(cloud_org);
    BaseARMarkerSequence.push_back(base_ar_marker);
    while(CloudSequence.size()>MaxCloudSequence)
    {
      CloudSequence.pop_front();
      BaseARMarkerSequence.pop_front();
    }

    // DEBUG
    {std::cerr<<"Base marker: "<<closest_m_id<<std::endl;
    for(int i(0);i<3;++i)std::cerr<<" "<<base_ar_marker.Position[i];
    for(int i(0);i<4;++i)std::cerr<<" "<<base_ar_marker.Orientation[i];  std::cerr<<std::endl;}

    DetectCylinder(
        pcl_viewer,
        cloud_org,
        FocusingOnSingleCylinder,
        PickedPoint,
        base_ar_marker);
  }

  // Draw AR markers:
  pcl::ModelCoefficients::Ptr sq_coefficients(new pcl::ModelCoefficients);
  sq_coefficients->values.resize(8);
  for(std::map<int,TARMarker>::const_iterator itr(ARMarkers.begin()),last(ARMarkers.end()); itr!=last; ++itr)
  {
    sq_coefficients->values[0]= itr->second.Position[0];
    sq_coefficients->values[1]= itr->second.Position[1];
    sq_coefficients->values[2]= itr->second.Position[2];
    sq_coefficients->values[3]= itr->second.Orientation[0];
    sq_coefficients->values[4]= itr->second.Orientation[1];
    sq_coefficients->values[5]= itr->second.Orientation[2];
    sq_coefficients->values[6]= itr->second.Orientation[3];
    sq_coefficients->values[7]= itr->second.Size;
    int i= itr->first;
    pcl_viewer.AddSquare(sq_coefficients, GetID("sq",i),
        2, IndexedColors[i%NumIndexedColors]);
  }
}
//-------------------------------------------------------------------------------------------

void DetectCylinder(
    TPCLViewer &pcl_viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_org,
    bool focused_analysis,
    const Eigen::Vector3d &focusing_point,
    const TARMarker &base_ar_marker)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_org,*cloud);
  if(VerbosePrint)  std::cerr<<"Number of original points: "<<cloud->points.size()<<std::endl;

  // Downsample the dataset using a leaf size of 0.5cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_low(new pcl::PointCloud<pcl::PointXYZ>);
  DownsampleByVoxelGrid<pcl::PointXYZ>(cloud, cloud_low, 0.005, 0.005, 0.005);
  if(VerbosePrint)  std::cerr<<"Number of downsampled points: "<<cloud_low->points.size()<<std::endl;
  if(cloud_low->points.size()==0)  return;

  // Remove plains
  bool res= RemovePlains<pcl::PointXYZ>(cloud_low,
                /*non_planar_points_ratio=*/0.3,
                /*ransac_dist_thresh=*/0.01,
                /*ransac_max_iterations=*/100);
  if(!res)  return;
  if(VerbosePrint)  std::cerr<<"Number of non-planar points: "<<cloud_low->points.size()<<std::endl;
  if(cloud_low->points.size()==0)  return;

  // Extract clusters
  std::vector<pcl::PointIndices> cluster_indices_low;
  ExtractClusters<pcl::PointXYZ>(cloud_low, cluster_indices_low,
      /*cluster_tol=*/0.03/*3cm*/,
      /*min_cluster_size=*/100,
      /*max_cluster_size=*/25000);
  if(VerbosePrint)  std::cerr<<"Clusters found: "<<cluster_indices_low.size()<<std::endl;
  if(cluster_indices_low.size()==0)  return;

  // Expand clusters to the original (before downsampling) point cloud resolution
  std::vector<pcl::PointIndices::Ptr> cluster_indices(cluster_indices_low.size());
  TNeighborSearcher<pcl::PointXYZ>  neighbor_searcher(cloud);
  int  nearest_cluster(-1);
  double  nearest_distance(1.0e10);
  for(size_t j(0); j<cluster_indices_low.size(); ++j)
  {
    cluster_indices[j]= pcl::PointIndices::Ptr(new pcl::PointIndices);
    neighbor_searcher.SearchFromPoints(
        cloud_low,
        cluster_indices_low[j],
        cluster_indices[j],
        /*radius=*/0.01);

    // Get the nearest cluster from focusing_point
    Eigen::Vector4f  center;
    pcl::compute3DCentroid<pcl::PointXYZ>(*cloud, *cluster_indices[j], center);
    // double distance= (center.head<3>().cast<double>() - focusing_point).norm();
    double distance= (center.head<3>().cast<double>() - Eigen::Vector3d(base_ar_marker.Position)).norm();
    if(distance<nearest_distance)
    {
      nearest_cluster= j;
      nearest_distance= distance;
    }
  }


  // Visualize the original cluster
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_col(new pcl::PointCloud<pcl::PointXYZRGB>);
  ColorPointCloud(cloud, cloud_col, ColorGray);
  pcl_viewer.RemoveAll();
  pcl_viewer.AddPointCloud(cloud_col,"cloud1",1);


  int i_cyl(0);
  // for each cluster j
  for(size_t j(0); j<cluster_indices.size(); ++j)
  {
    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT>::Ptr  cloud_obj(new pcl::PointCloud<PointT>);
    ExtractByIndices<PointT>(cloud_org, cloud_obj, cluster_indices[j]);

    if(VerbosePrint)  std::cerr<<"-----------------------"<<std::endl;
    if(VerbosePrint)  std::cerr<<"Cluster "<<j<<" ("<<cloud_obj->points.size()<<")"<<std::endl;
    if((focused_analysis && j!=nearest_cluster)
        || cloud_obj->points.size() < 500)
    {
      pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);
      continue;
    }

    // Apply RANSAC for each cluster to get cylinder
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cyl_coefficients_std(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr cyl_coefficients_ext(new pcl::ModelCoefficients);
    double cratio, cylinder_ratio_thresh;
    cratio= ExtractCylinder<PointT>(
        cloud_obj,
        inliers_cylinder,
        cyl_coefficients_std,
        cyl_coefficients_ext,
        /*normal_est_k=*/50,
        /*ransac_normal_dist_w=*/0.1,
        /*ransac_dist_thresh=*/0.05,
        /*ransac_radius_min=*/0.0,
        /*ransac_radius_max=*/0.5,
        /*ransac_max_iterations=*/1000,
        cylinder_ratio_thresh=0.5);

    if(VerbosePrint)  std::cerr<<"Cylinder ratio: "<<cratio<<std::endl;
    if(cratio < cylinder_ratio_thresh)
    {
      pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);
      continue;
    }

    if(VerbosePrint)  std::cerr<<"Cylinder "<<i_cyl<<std::endl;
    if(VerbosePrint)  std::cerr<<"Cylinder coefficients_std: "<<*cyl_coefficients_std<<std::endl;
    if(VerbosePrint)  std::cerr<<"Cylinder coefficients_ext: "<<*cyl_coefficients_ext<<std::endl;

    // Visualize the cylinder inliers
    double orig_col_ratio=0.9;
    for (int i(0); i<inliers_cylinder->indices.size(); ++i)
    {
      cloud_obj->points[inliers_cylinder->indices[i]].r
          = orig_col_ratio*cloud_obj->points[inliers_cylinder->indices[i]].r
              + (1.0-orig_col_ratio)*IndexedColors[i_cyl%NumIndexedColors][0];
      cloud_obj->points[inliers_cylinder->indices[i]].g
          = orig_col_ratio*cloud_obj->points[inliers_cylinder->indices[i]].g
              + (1.0-orig_col_ratio)*IndexedColors[i_cyl%NumIndexedColors][1];
      cloud_obj->points[inliers_cylinder->indices[i]].b
          = orig_col_ratio*cloud_obj->points[inliers_cylinder->indices[i]].b
              + (1.0-orig_col_ratio)*IndexedColors[i_cyl%NumIndexedColors][2];
    }

    pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);
    pcl_viewer.AddCylinder(cyl_coefficients_std,cyl_coefficients_ext,GetID("cloud_obj_cyl",j));

    // Detailed analysis on a focused cylinder
    if(focused_analysis)
    {
      /*Show the base_ar_marker*/{
        pcl::ModelCoefficients::Ptr sq_coefficients(new pcl::ModelCoefficients);
        sq_coefficients->values.resize(8);
        sq_coefficients->values[0]= base_ar_marker.Position[0];
        sq_coefficients->values[1]= base_ar_marker.Position[1];
        sq_coefficients->values[2]= base_ar_marker.Position[2];
        sq_coefficients->values[3]= base_ar_marker.Orientation[0];
        sq_coefficients->values[4]= base_ar_marker.Orientation[1];
        sq_coefficients->values[5]= base_ar_marker.Orientation[2];
        sq_coefficients->values[6]= base_ar_marker.Orientation[3];
        sq_coefficients->values[7]= base_ar_marker.Size;
        pcl_viewer.AddSquare(sq_coefficients, GetID("base_ar_marker",j),
            4, IndexedColors[j%NumIndexedColors]);
      }

      double base_frame[7];
      base_frame[0]= base_ar_marker.Position[0];
      base_frame[1]= base_ar_marker.Position[1];
      base_frame[2]= base_ar_marker.Position[2];
      base_frame[3]= base_ar_marker.Orientation[0];
      base_frame[4]= base_ar_marker.Orientation[1];
      base_frame[5]= base_ar_marker.Orientation[2];
      base_frame[6]= base_ar_marker.Orientation[3];
      TContainerProperty<pcl::PointXYZRGB>
        container_property(
          cloud_obj,
          inliers_cylinder,
          cyl_coefficients_std,
          cyl_coefficients_ext,
          base_frame,
          /*pour_edge_ratio=*/0.1,
          /*grab_height=*/0.02,  // 2cm
          /*grab_z_step_ratio=*/0.01,
          /*grab_radius_sd_thresh_ratio=*/0.2);
      container_property.Visualize(pcl_viewer, GetID("container",j));
      // container_property.PrintAsYAML(std::cout, /*g_width_margin=*/0.8);
      std::string file_name("/tmp/cyl_info.yaml");
      std::ofstream ofs(file_name.c_str());
      container_property.PrintAsYAML(ofs, /*g_width_margin=*/0.8);
      std::cerr<<"Saved cylinder info into: "<<file_name<<std::endl;

      /*DEBUG*/{
        pcl::ModelCoefficients::Ptr sq_coefficients(new pcl::ModelCoefficients);
        sq_coefficients->values.resize(8);
        sq_coefficients->values[0]= 0.0;
        sq_coefficients->values[1]= 0.0;
        sq_coefficients->values[2]= 0.0;
        sq_coefficients->values[3]= 0.0;
        sq_coefficients->values[4]= 0.0;
        sq_coefficients->values[5]= 0.0;
        sq_coefficients->values[6]= 1.0;
        sq_coefficients->values[7]= 0.044;
        pcl_viewer.AddSquare(sq_coefficients, GetID("sqdebug",j), 3, ColorWhite);
      }

    }

    ++i_cyl;
  }  // for each cluster j
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

