//-------------------------------------------------------------------------------------------
/*! \file    cyl_detector.cpp
    \brief   Cylinder detector.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jul.28, 2014
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/cyl_detector.h"
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
double MarkerSize(0.048);

enum TDetectorMode {dmRunning=0, dmVisualizingCurrCloud, dmPause};
// static TDetectorMode  DetectorMode(dmRunning);
static TDetectorMode  DetectorMode(dmVisualizingCurrCloud);
static bool  VisInitRequired(false);
static bool  AddToBasePointCloudRequired(false);
static bool  FocusingOnSingleCylinder(false);
static bool  PointCloudChanged(false);
// 3D point for selecting a base AR marker:
static Eigen::Vector3d  PickedPoint(0.0,0.0,0.0);
static int  BaseARMarkerID(-1);
static int  RefARMarkerID(-1);
// Maximum number of point clouds to be stored:
static const int   MaxCloudSequence(10);
static std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  CloudSequence;
static std::list<TARMarker>                               BaseARMarkerSequence;
static std::list<TARMarker>                               RefARMarkerSequence;
static std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator  PlayingCloudItr;
static std::list<TARMarker>::iterator                               PlayingBaseARMarkerItr;
static std::list<TARMarker>::iterator                               PlayingRefARMarkerItr;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr  BasePointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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
        PointCloudChanged= false;
        PlayingCloudItr= CloudSequence.end();
        PlayingBaseARMarkerItr= BaseARMarkerSequence.end();
        PlayingRefARMarkerItr= RefARMarkerSequence.end();
        if(CloudSequence.begin()!=CloudSequence.end())
        {
          --PlayingCloudItr;
          --PlayingBaseARMarkerItr;
          --PlayingRefARMarkerItr;
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
        ++PlayingRefARMarkerItr;
        if(PlayingCloudItr==CloudSequence.end())
        {
          --PlayingCloudItr;
          --PlayingBaseARMarkerItr;
          --PlayingRefARMarkerItr;
        }
        else
        {
          std::cerr<<"Next snapshot"<<std::endl;
          PointCloudChanged= true;
        }
      }
    }
    else if(event.getKeySym()=="Left")
    {
      if(PlayingCloudItr!=CloudSequence.begin())
      {
        --PlayingCloudItr;
        --PlayingBaseARMarkerItr;
        --PlayingRefARMarkerItr;
        std::cerr<<"Previous snapshot"<<std::endl;
        PointCloudChanged= true;
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
    else if(event.getKeySym()=="B")
    {
      std::cerr<<"Type base AR marker ID: "<<std::flush;
      std::cin>>BaseARMarkerID;
      std::cerr<<"Type reference AR marker ID: "<<std::flush;
      std::cin>>RefARMarkerID;
    }
    else if(event.getKeySym()=="a")
    {
      AddToBasePointCloudRequired= true;
      PointCloudChanged= true;
      std::cerr<<"Requested to add a point cloud to BasePointCloud"<<std::endl;
    }
    else if(event.getKeySym()=="A")
    {
      BasePointCloud= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
      AddToBasePointCloudRequired= false;
      PointCloudChanged= true;
      std::cerr<<"Cleared BasePointCloud"<<std::endl;
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

  // Get a closest AR marker from PickedPoint
  BaseARMarkerID= -1;
  double distance(1.0e10);
  for(std::map<int,TARMarker>::const_iterator itr(ARMarkers.begin()),last(ARMarkers.end()); itr!=last; ++itr)
  {
    double d= (Eigen::Vector3d(itr->second.Position)-PickedPoint).norm();
    if(d<distance)  {distance= d;  BaseARMarkerID= itr->first;}
  }
  std::cerr<<"Base AR marker: "<<BaseARMarkerID<<std::endl;
  // TODO: write code to get RefARMarkerID
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
    if(PointCloudChanged)
    {
      DetectCylinder(
          pcl_viewer,
          *PlayingCloudItr,
          FocusingOnSingleCylinder,
          *PlayingBaseARMarkerItr,
          RefARMarkerID,  // TODO: Store the ref marker IDs
          *PlayingRefARMarkerItr,
          BasePointCloud,
          AddToBasePointCloudRequired);
      PointCloudChanged= false;
      AddToBasePointCloudRequired= false;
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
    /*
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
    */
    TARMarker  base_ar_marker, ref_ar_marker;
    if(BaseARMarkerID>=0)
      base_ar_marker= ARMarkers[BaseARMarkerID];
    if(RefARMarkerID>=0)
      ref_ar_marker= ARMarkers[RefARMarkerID];
    else
      ref_ar_marker= base_ar_marker;

    // Store the cloud and the marker
    CloudSequence.push_back(cloud_org);
    BaseARMarkerSequence.push_back(base_ar_marker);
    RefARMarkerSequence.push_back(ref_ar_marker);
    while(CloudSequence.size()>MaxCloudSequence)
    {
      CloudSequence.pop_front();
      BaseARMarkerSequence.pop_front();
      RefARMarkerSequence.pop_front();
    }

    // DEBUG
    {std::cerr<<"Base marker: "<<BaseARMarkerID<<std::endl;
    for(int i(0);i<3;++i)std::cerr<<" "<<base_ar_marker.Position[i];
    for(int i(0);i<4;++i)std::cerr<<" "<<base_ar_marker.Orientation[i];  std::cerr<<std::endl;
    std::cerr<<"Ref marker: "<<RefARMarkerID<<std::endl;}

    DetectCylinder(
        pcl_viewer,
        cloud_org,
        FocusingOnSingleCylinder,
        base_ar_marker,
        RefARMarkerID,
        ref_ar_marker,
        BasePointCloud,
        AddToBasePointCloudRequired);
    AddToBasePointCloudRequired= false;
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
    const TARMarker &base_ar_marker,
    int ref_ar_marker_id,
    const TARMarker &ref_ar_marker,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_point_cloud,
    bool add_to_base_point_cloud)
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
  bool res= RemovePlanes<pcl::PointXYZ>(cloud_low,
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

    // Get the nearest cluster from base_ar_marker
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


  // for each cluster j
  for(size_t j(0); j<cluster_indices.size(); ++j)
  {
    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT>::Ptr  cloud_obj(new pcl::PointCloud<PointT>);
    ExtractByIndices<PointT>(cloud_org, cloud_obj, cluster_indices[j]);

    if(VerbosePrint)  std::cerr<<"-----------------------"<<std::endl;
    if(VerbosePrint)  std::cerr<<"Cluster "<<j<<" ("<<cloud_obj->points.size()<<")"<<std::endl;
    if(!focused_analysis
        && ( (focused_analysis && j!=nearest_cluster)
            || cloud_obj->points.size() < 500 ) )
    {
      pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);
      continue;
    }

    double base_frame[7], ref_marker_pose[7], ref_marker_pose_l[7];
    ARMarkerToX(base_ar_marker, base_frame);
    ARMarkerToX(ref_ar_marker, ref_marker_pose);
    // Transform ref_marker_pose to base_frame's local frame
    {
      Eigen::Affine3d Tbase, Tf, Tref;
      Tbase= XToEigMat(base_frame);
      Tf= Tbase.inverse();
      Tref= Tf * XToEigMat(ref_marker_pose);
      EigMatToX(Tref, ref_marker_pose_l);
    }

    // Transform cloud_obj to base_frame's local frame
    pcl::PointCloud<PointT>::Ptr  cloud_obj_l(new pcl::PointCloud<PointT>);
    {
      Eigen::Affine3d Tbase, Tf;
      Tbase= XToEigMat(base_frame);
      Tf= Tbase.inverse();
      pcl::transformPointCloud(*cloud_obj, *cloud_obj_l, Tf.matrix().cast<float>());
    }

    if(add_to_base_point_cloud && j==nearest_cluster)
    {
      (*base_point_cloud)+= (*cloud_obj_l);
      std::cerr<<"Added to BasePointCloud: "<<base_point_cloud->points.size()-cloud_obj_l->points.size()<<" --> "<<base_point_cloud->points.size()<<std::endl;
    }

    if(focused_analysis)
    {
      if(j==nearest_cluster)
      {
        if(add_to_base_point_cloud)
          cloud_obj_l= base_point_cloud;
        else
          (*cloud_obj_l)+= (*base_point_cloud);
        AnalyzeObjectVer2(pcl_viewer, cloud_obj_l, j,
            base_frame, ref_ar_marker_id, ref_marker_pose_l);
      }
    }
    else
    {
      pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);
    }
  }  // for each cluster j
}
//-------------------------------------------------------------------------------------------


// Analyzing an object's point cloud by detecting multiple cylinders
void AnalyzeObjectVer2(
    TPCLViewer &pcl_viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_obj,
    int obj_id,
    const double base_frame[7],
    int ref_ar_marker_id,
    const double ref_marker_pose[7])
{
  int j= obj_id;

  // // Apply RANSAC for each cluster to get cylinder
  // pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr cyl_coefficients_std(new pcl::ModelCoefficients);
  // pcl::ModelCoefficients::Ptr cyl_coefficients_ext(new pcl::ModelCoefficients);
  // double cratio, cylinder_ratio_thresh;
  // cratio= ExtractCylinder<pcl::PointXYZRGB>(
      // cloud_obj,
      // inliers_cylinder,
      // cyl_coefficients_std,
      // cyl_coefficients_ext,
      // /*normal_est_k=*/50,
      // /*ransac_normal_dist_w=*/0.1,
      // /*ransac_dist_thresh=*/0.05,
      // /*ransac_radius_min=*/0.0,
      // /*ransac_radius_max=*/0.5,
      // /*ransac_max_iterations=*/1000,
      // cylinder_ratio_thresh=0.5);

  // if(VerbosePrint)  std::cerr<<"Cylinder ratio: "<<cratio<<std::endl;
  // if(cratio < cylinder_ratio_thresh)
  // {
    // pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);
    // return;
  // }

  // if(VerbosePrint)  std::cerr<<"Cylinder detected"<<std::endl;
  // if(VerbosePrint)  std::cerr<<"Cylinder coefficients_std: "<<*cyl_coefficients_std<<std::endl;
  // if(VerbosePrint)  std::cerr<<"Cylinder coefficients_ext: "<<*cyl_coefficients_ext<<std::endl;

  // Color the cylinder inliers
  // double orig_col_ratio=0.9;
  // for (int i(0); i<inliers_cylinder->indices.size(); ++i)
  // {
    // cloud_obj->points[inliers_cylinder->indices[i]].r
        // = orig_col_ratio*cloud_obj->points[inliers_cylinder->indices[i]].r
            // + (1.0-orig_col_ratio)*IndexedColors[j%NumIndexedColors][0];
    // cloud_obj->points[inliers_cylinder->indices[i]].g
        // = orig_col_ratio*cloud_obj->points[inliers_cylinder->indices[i]].g
            // + (1.0-orig_col_ratio)*IndexedColors[j%NumIndexedColors][1];
    // cloud_obj->points[inliers_cylinder->indices[i]].b
        // = orig_col_ratio*cloud_obj->points[inliers_cylinder->indices[i]].b
            // + (1.0-orig_col_ratio)*IndexedColors[j%NumIndexedColors][2];
  // }

  /*Detailed analysis on a focused cylinder*/{
    /*Show the original cloud, base_ar_marker, cylinder*-/{
      pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);

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

      pcl_viewer.AddCylinder(cyl_coefficients_std, cyl_coefficients_ext,
          GetID("cloud_obj_cyl",j),
          3, IndexedColors[j%NumIndexedColors]);
    }//*/

    TContainerAnalyzer2<pcl::PointXYZRGB>  container_property;
    bool res= container_property.Analyze(
        cloud_obj,

        /*cyl_normal_est_k=*/20,  // 50
        /*cyl_ransac_normal_dist_w=*/0.1,
        /*cyl_ransac_dist_thresh=*/0.01,  // 0.05
        /*cyl_ransac_radius_min=*/0.0,
        /*cyl_ransac_radius_max=*/0.5,
        /*cyl_ransac_max_iterations=*/100,
        /*cyl_ratio_thresh=*/0.5,

        /*pour_edge_ratio=*/0.05,  // 0.1
        /*pour_hull_alpha=*/0.2,
        /*grab_height=*/0.02,  // 2cm
        /*grab_z_step_ratio=*/0.01,
        /*grab_radius_sd_thresh_ratio=*/0.2,
        ref_ar_marker_id,
        ref_marker_pose);
    if(!res)
    {
      pcl_viewer.AddPointCloud(cloud_obj,GetID("cloud_obj",j),2);
      return;
    }
    container_property.Visualize(
        pcl_viewer,
        GetID("container",j),
        base_frame /*IdentityTransform*/,    // Visualiza at the base_frame position or the origin
        IndexedColors[5]);  // IndexedColors[j%NumIndexedColors]
    // container_property.PrintAsYAML(std::cout);
    std::string file_name("/tmp/cyl_info.yaml");
    std::string file_name_pcd("/tmp/cyl_info.pcd");
    std::ofstream ofs(file_name.c_str());
    container_property.PrintAsYAML(ofs);
    std::cerr<<"Saved cylinder info into: "<<file_name<<std::endl;
    pcl::io::savePCDFile(file_name_pcd, *cloud_obj, /*binary_mode=*/true);
    std::cerr<<"Saved cylinder point cloud into: "<<file_name_pcd<<std::endl;
    if(container_property.PourPoints->points.size()>0)
      pcl::io::savePCDFile("/tmp/cyl_info_pour.pcd", *container_property.PourPoints, /*binary_mode=*/true);

  }
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

