//-------------------------------------------------------------------------------------------
/*! \file    cyl_detector.h
    \brief   Cylinder detector.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jul.28, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef cyl_detector_h
#define cyl_detector_h
//-------------------------------------------------------------------------------------------
#include "lfd_vision/pcl_util.h"
#include "lfd_vision/geom_util.h"
//-------------------------------------------------------------------------------------------
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/concave_hull.h>
// #include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/common/pca.h>
#include <map>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

struct TARMarker
{
  double Position[3];     // x,y,z
  double Orientation[4];  // x,y,z,w
  double Size;
};
namespace detail
{
extern std::map<int,TARMarker>  ARMarkers;
extern double MarkerSize;
}
static const float ColorWhite[3]= {255,255,255};
static const float ColorGray[3]= {128,128,128};
static const double IdentityTransform[7]= {0.0,0.0,0.0, 0.0,0.0,0.0,1.0};
//-------------------------------------------------------------------------------------------

inline std::string GetID(const std::string &base, int idx)
{
  std::stringstream ss;
  ss<<base<<idx;
  return ss.str();
}
//-------------------------------------------------------------------------------------------

/* Pose to TARMarker.  Size is not changed.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void XToARMarker(const t_value x[], TARMarker &ar_marker)
{
  ar_marker.Position[0]   = x[0];
  ar_marker.Position[1]   = x[1];
  ar_marker.Position[2]   = x[2];
  ar_marker.Orientation[0]= x[3];
  ar_marker.Orientation[1]= x[4];
  ar_marker.Orientation[2]= x[5];
  ar_marker.Orientation[3]= x[6];
}
/* TARMarker to pose.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void ARMarkerToX(const TARMarker &ar_marker, t_value x[7])
{
  x[0]= ar_marker.Position[0];
  x[1]= ar_marker.Position[1];
  x[2]= ar_marker.Position[2];
  x[3]= ar_marker.Orientation[0];
  x[4]= ar_marker.Orientation[1];
  x[5]= ar_marker.Orientation[2];
  x[6]= ar_marker.Orientation[3];
}
//-------------------------------------------------------------------------------------------

void KeyboardEventCallback(
    const pcl::visualization::KeyboardEvent &event,
    void *viewer_void);
void MouseEventCallback(
    const pcl::visualization::MouseEvent &event,
    void *viewer_void);
void PointpickEventCallback(
    const pcl::visualization::PointPickingEvent &event,
    void *viewer_void);
//-------------------------------------------------------------------------------------------

class TPCLViewer;

void ExecuteDetection(
    TPCLViewer &pcl_viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_org);
void DetectCylinder(
    TPCLViewer &pcl_viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_org,
    bool focused_analysis,
    const TARMarker &base_ar_marker,
    int ref_ar_marker_id,
    const TARMarker &ref_ar_marker,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_point_cloud,
    bool add_to_base_point_cloud);

// Analyzing an object's point cloud by detecting multiple cylinders
void AnalyzeObjectVer2(
    TPCLViewer &pcl_viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_obj,
    int obj_id,
    const double base_frame[7],
    int ref_ar_marker_id,
    const double ref_marker_pose[7]);
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TPCLViewer
//===========================================================================================
{
public:
  TPCLViewer()
    {
      Init();
    }

  TPCLViewer(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, const std::string &name="cloud1", int point_size=1)
    {
      Init();
      AutoCamera(cloud, name, point_size);
    }

  void Init()
    {
      viewer_= boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer_->setBackgroundColor (0, 0, 0);
      viewer_->addCoordinateSystem (0.2);

      viewer_->registerKeyboardCallback(KeyboardEventCallback, (void*)this);
      viewer_->registerMouseCallback(MouseEventCallback, (void*)this);
      viewer_->registerPointPickingCallback(PointpickEventCallback, (void*)this);
    }

  bool IsStopped()  {return viewer_->wasStopped();}
  void SpinOnce(int time=1, bool force_redraw=false)
    {
      viewer_->spinOnce(time,force_redraw);
    }

  void AutoCamera(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
      const std::string &name="cloud1",
      int point_size=1)
    {
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer_->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
      viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);

      viewer_->initCameraParameters ();
      viewer_->resetCameraViewpoint(name);
    }

  void AddPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
      const std::string &name, int point_size=1)
    {
      if(cloud->points.size()>0)
      {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer_->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
      }
    }

  /* Draw cylinder.
      coefficients_std: [0-2]: point on axis, [3-5]: axis, [6]: radius,
      coefficients_ext: [0-2]: center x,y,z, [3]: length.  */
  void AddCylinder(
      const pcl::ModelCoefficients::Ptr &coefficients_std,
      const pcl::ModelCoefficients::Ptr &coefficients_ext,
      const std::string &name, int point_size=2, const float rgb[3]=ColorWhite)
    {
      // viewer_->addCylinder (*coefficients_std, name);
      const std::vector<float> &c_std(coefficients_std->values);
      const std::vector<float> &c_ext(coefficients_ext->values);
      double radius= c_std[6];
      double len= c_ext[3];

      if(radius<0.0 || len<0.0)
      {
        std::cerr<<"#Invalid cylinder"<<std::endl;
        return;
      }

      // Generate a basic cylinder
      int num_z(20), num_th(40);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bcyl(new pcl::PointCloud<pcl::PointXYZRGB>());
      cloud_bcyl->width    = num_z*num_th;
      cloud_bcyl->height   = 1;
      cloud_bcyl->is_dense = false;
      cloud_bcyl->points.resize(cloud_bcyl->width * cloud_bcyl->height);

      size_t i(0);
      for(int iz(0); iz<num_z; ++iz)
      {
        double z= -0.5*len + static_cast<double>(iz)*len/static_cast<double>(num_z-1);
        for(int ith(0); ith<num_th; ++ith)
        {
          double th= static_cast<double>(ith)*2.0*M_PI/static_cast<double>(num_th-1);
          double x= radius*std::cos(th), y= radius*std::sin(th);
          cloud_bcyl->points[i].x= x;
          cloud_bcyl->points[i].y= y;
          cloud_bcyl->points[i].z= z;
          cloud_bcyl->points[i].r= rgb[0];
          cloud_bcyl->points[i].g= rgb[1];
          cloud_bcyl->points[i].b= rgb[2];
          ++i;
        }
      }
      // pcl::io::savePCDFileASCII ("/tmp/test_pcd.pcd", *cloud_bcyl);

      // Rotate and translate the cylinder
      Eigen::Vector3d cyl_axis(c_std[3],c_std[4],c_std[5]);
      cyl_axis.normalize();
      Eigen::Vector3d rot_axis= Eigen::Vector3d::UnitZ().cross(cyl_axis);
      rot_axis.normalize();
      double rot_angle= std::acos(Eigen::Vector3d::UnitZ().transpose()*cyl_axis);

      Eigen::Affine3d TA;
      TA= Eigen::Translation3d(c_ext[0],c_ext[1],c_ext[2]) * Eigen::AngleAxisd(rot_angle,rot_axis);
      Eigen::Matrix4f TM(TA.matrix().cast<float>());
      // std::cerr<<"Cylinder trans:\n"<<TM<<std::endl;

      // Executing the transformation
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cyl(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*cloud_bcyl, *cloud_cyl, TM);

      // AddPointCloud(cloud_bcyl, name+"_b", 1);
      AddPointCloud(cloud_cyl, name, point_size);
    }

  /* Draw square.
      coefficients: [0-2]: center x,y,z, [3-6]: orientation x,y,z,w, [7]: size.  */
  void AddSquare(
      const pcl::ModelCoefficients::Ptr &coefficients,
      const std::string &name, int point_size=2, const float rgb[3]=ColorWhite)
    {
      const std::vector<float> &c(coefficients->values);
      double size= c[7];

      // Generate a basic square
      int num_xy(40);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bsq(new pcl::PointCloud<pcl::PointXYZRGB>());
      cloud_bsq->width    = num_xy*4;
      cloud_bsq->height   = 1;
      cloud_bsq->is_dense = false;
      cloud_bsq->points.resize(cloud_bsq->width * cloud_bsq->height);
      double dirs[][2]={{1.0,1.0}, {-1.0,1.0}, {-1.0,-1.0}, {1.0,-1.0}, {1.0,1.0} };

      size_t i(0);
      for(int e(0); e<4; ++e)
      {
        double dx= 0.5*size*(dirs[e+1][0]-dirs[e][0])/static_cast<double>(num_xy);
        double dy= 0.5*size*(dirs[e+1][1]-dirs[e][1])/static_cast<double>(num_xy);
        for(int ixy(0); ixy<num_xy; ++ixy)
        {
          cloud_bsq->points[i].x= 0.5*size*dirs[e][0]+static_cast<double>(ixy)*dx;
          cloud_bsq->points[i].y= 0.5*size*dirs[e][1]+static_cast<double>(ixy)*dy;
          cloud_bsq->points[i].z= 0.0;
          cloud_bsq->points[i].r= rgb[0];
          cloud_bsq->points[i].g= rgb[1];
          cloud_bsq->points[i].b= rgb[2];
          ++i;
        }
      }

      Eigen::Affine3d TA;
      TA= Eigen::Translation3d(c[0],c[1],c[2]) * Eigen::Quaterniond(c[6],c[3],c[4],c[5]);
      Eigen::Matrix4f TM(TA.matrix().cast<float>());

      // Executing the transformation
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sq(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*cloud_bsq, *cloud_sq, TM);

      AddPointCloud(cloud_sq, name, point_size);
    }

  void AddPolygon(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
      const std::string &name, const float rgb[3]=ColorWhite)
    {
      if(cloud->points.size()>0)
        viewer_->addPolygon<pcl::PointXYZRGB>(cloud, rgb[0], rgb[1], rgb[2], name);
    }

  void RemoveAll()
    {
      viewer_->removeAllPointClouds();
      viewer_->removeAllShapes();
    }

private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer_;

};
//-------------------------------------------------------------------------------------------

// Transform a point cloud and return a new cloud (new memory is allocated)
template <typename t_point>
typename pcl::PointCloud<t_point>::Ptr  TransformCloud(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in,
    const Eigen::Affine3d &T)
{
  typename pcl::PointCloud<t_point>::Ptr  cloud_out(new pcl::PointCloud<t_point>());
  pcl::transformPointCloud(*cloud_in, *cloud_out, T.matrix().cast<float>());
  return cloud_out;
}
//-------------------------------------------------------------------------------------------

// Apply VoxelGrid filter of leaf size (lx,ly,lz)
template <typename t_point>
void DownsampleByVoxelGrid(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in,
    typename pcl::PointCloud<t_point>::Ptr &cloud_out,
    float lx, float ly, float lz)
{
  // Create the filtering object: downsample the dataset
  pcl::VoxelGrid<t_point> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (lx,ly,lz);
  vg.filter (*cloud_out);
}
//-------------------------------------------------------------------------------------------

// Remove planar objects from a point cloud
template <typename t_point>
bool RemovePlanes(
    typename pcl::PointCloud<t_point>::Ptr &cloud_io,
    const double &non_planar_points_ratio,
    const double &ransac_dist_thresh,
    int ransac_max_iterations)
{
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<t_point>  seg;
  pcl::PointIndices::Ptr  inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr  coefficients(new pcl::ModelCoefficients);
  // pcl::PointCloud<t_point>::Ptr  cloud_plane(new pcl::PointCloud<t_point> ());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations);
  seg.setDistanceThreshold(ransac_dist_thresh);

  int nr_points= cloud_io->points.size();
  while(cloud_io->points.size() > non_planar_points_ratio*nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_io);
    seg.segment (*inliers, *coefficients); //*
    if(inliers->indices.size() == 0)
    {
      std::cerr<<"Error: Could not estimate a planar model for the given dataset."<<std::endl;
      return false;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<t_point> extract;
    extract.setInputCloud(cloud_io);
    extract.setIndices(inliers);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_io);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

// Extract clusters from a point cloud
template <typename t_point>
void ExtractClusters(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in,
    std::vector<pcl::PointIndices> &cluster_indices,
    const double &cluster_tol,
    int min_cluster_size, int max_cluster_size)
{
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<t_point>::Ptr tree (new pcl::search::KdTree<t_point>);
  tree->setInputCloud (cloud_in);

  pcl::EuclideanClusterExtraction<t_point> ec;
  ec.setClusterTolerance(cluster_tol);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);
}
//-------------------------------------------------------------------------------------------

static void ColorPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out,
    const float rgb[3])
{
  if(cloud_in->points.size()>0)
  {
    pcl::copyPointCloud(*cloud_in, *cloud_out);
    for (int i(0); i<cloud_out->points.size(); ++i)
    {
      cloud_out->points[i].r = rgb[0];
      cloud_out->points[i].g = rgb[1];
      cloud_out->points[i].b = rgb[2];
    }
  }
}
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in,
    const float rgb[3])
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
  ColorPointCloud(cloud_in, cloud_out, rgb);
  return cloud_out;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
template <typename t_point>
class TNeighborSearcher
//===========================================================================================
{
public:
  TNeighborSearcher(const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in)
    {
      kdtree_.setInputCloud(cloud_in);
      tmp_indices_.resize(cloud_in->points.size());
    }

  void SearchFromPoints(
      const typename pcl::PointCloud<t_point>::ConstPtr &cloud_ref,
      const pcl::PointIndices &ref_indices,
      pcl::PointIndices::Ptr &indices_out,
      const double &radius)
    {
      std::vector<int> &ext_indices(indices_out->indices);
      ext_indices.clear();
      for(std::vector<int>::const_iterator pit(ref_indices.indices.begin()),plast(ref_indices.indices.end()); pit!=plast; ++pit)
      {
        t_point search_point;
        search_point.x = cloud_ref->points[*pit].x;
        search_point.y = cloud_ref->points[*pit].y;
        search_point.z = cloud_ref->points[*pit].z;
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;

        kdtree_.radiusSearch(search_point,radius,k_indices,k_sqr_distances);
        std::sort(k_indices.begin(),k_indices.end());
        std::vector<int>::iterator last_itr= std::set_union(ext_indices.begin(), ext_indices.end(), k_indices.begin(), k_indices.end(), tmp_indices_.begin());
        ext_indices.resize(last_itr-tmp_indices_.begin());
        std::copy(tmp_indices_.begin(),last_itr, ext_indices.begin());
      }
    }

private:
  pcl::KdTreeFLANN<t_point> kdtree_;
  std::vector<int> tmp_indices_;
};
//-------------------------------------------------------------------------------------------

template <typename t_point>
void ExtractByIndices(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in,
    typename pcl::PointCloud<t_point>::Ptr &cloud_out,
    const pcl::PointIndices::ConstPtr &indices)
{
  pcl::ExtractIndices<t_point> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*cloud_out);
}
template <typename t_point>
typename pcl::PointCloud<t_point>::Ptr ExtractByIndices(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in,
    const pcl::PointIndices::ConstPtr &indices)
{
  typename pcl::PointCloud<t_point>::Ptr  cloud_out(new pcl::PointCloud<t_point>);
  ExtractByIndices<t_point>(cloud_in, cloud_out, indices);
  return cloud_out;
}
//-------------------------------------------------------------------------------------------

template <typename t_point_in, typename t_point_out>
void ApplyMLSFilter(
    const typename pcl::PointCloud<t_point_in>::ConstPtr &cloud_in,
    typename pcl::PointCloud<t_point_out>::Ptr &cloud_out,
    const double &search_radius, bool compute_normals)
{
  typename pcl::search::KdTree<t_point_in>::Ptr  tree(new pcl::search::KdTree<t_point_in>);
  pcl::MovingLeastSquares<t_point_in, t_point_out> mls;

  mls.setComputeNormals(compute_normals);

  // Set parameters
  mls.setInputCloud(cloud_in);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(search_radius);

  // Reconstruct
  mls.process(*cloud_out);
}
//-------------------------------------------------------------------------------------------


/* Analyze the cylinder information to get the center position and the length.
    coefficients_ext: [0-2]: center x,y,z, [3]: length */
template <typename t_point>
void GetCylinderProp(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_obj,
    const pcl::PointIndices::ConstPtr &inliers_cyl,
    const pcl::ModelCoefficients::Ptr &coefficients_std,
    pcl::ModelCoefficients::Ptr &coefficients_ext)
{
  const std::vector<float> &c_std(coefficients_std->values);
  std::vector<float> &c_ext(coefficients_ext->values);
  Eigen::Vector3d cyl_axis(c_std[3],c_std[4],c_std[5]);
  cyl_axis.normalize();
  Eigen::Vector3d cyl_point(c_std[0],c_std[1],c_std[2]);

  // Project each point on the cylinder axis, compute their minimum and maximum
  double tmin(1.0e10), tmax(-1.0e10);
  for (int i(0); i<inliers_cyl->indices.size(); ++i)
  {
    // Project the point on the cylinder axis
    Eigen::Vector3d p(cloud_obj->points[inliers_cyl->indices[i]].x,cloud_obj->points[inliers_cyl->indices[i]].y,cloud_obj->points[inliers_cyl->indices[i]].z);
    double t= (p-cyl_point).transpose()*cyl_axis;
    if(t<tmin)  tmin= t;
    if(t>tmax)  tmax= t;
  }

  // std::cerr<<"tmin: "<<tmin<<std::endl;
  // std::cerr<<"tmax: "<<tmax<<std::endl;
  Eigen::Vector3d cyl_center= cyl_point + 0.5*(tmax+tmin)*cyl_axis;
  c_ext.resize(4);
  c_ext[0]= cyl_center[0];
  c_ext[1]= cyl_center[1];
  c_ext[2]= cyl_center[2];
  c_ext[3]= tmax-tmin;
}
//-------------------------------------------------------------------------------------------

/* Extract a cylinder from a point cloud.
    coefficients_std: [0-2]: point on axis, [3-5]: axis, [6]: radius,
    coefficients_ext: [0-2]: center x,y,z, [3]: length.
  \return The ratio of the number of inliers per the number of cloud_in.
  \note If the ratio of the number of inliers per the number of cloud_in is smaller than cylinder_ratio_thresh, coefficients_ext is not computed. */
template <typename t_point>
double ExtractCylinder(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in,
    pcl::PointIndices::Ptr &inliers,
    pcl::ModelCoefficients::Ptr &coefficients_std,
    pcl::ModelCoefficients::Ptr &coefficients_ext,
    int normal_est_k,
    const double &ransac_normal_dist_w,
    const double &ransac_dist_thresh,
    const double &ransac_radius_min,
    const double &ransac_radius_max,
    int ransac_max_iterations,
    const double &cylinder_ratio_thresh)
{
  // Estimate point normals
  pcl::PointCloud<pcl::Normal>::Ptr  cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<t_point, pcl::Normal>  ne;
  typename pcl::search::KdTree<t_point>::Ptr  tree(new pcl::search::KdTree<t_point>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_in);
  ne.setKSearch(normal_est_k);
  ne.compute(*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<t_point, pcl::Normal>  seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(ransac_normal_dist_w);
  seg.setMaxIterations(ransac_max_iterations);
  seg.setDistanceThreshold(ransac_dist_thresh);
  seg.setRadiusLimits(ransac_radius_min, ransac_radius_max);
  seg.setInputCloud(cloud_in);
  seg.setInputNormals(cloud_normals);
  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers, *coefficients_std);

  double cratio= (double)inliers->indices.size() / (double)cloud_in->points.size();
  if(cratio<cylinder_ratio_thresh)  return cratio;

  GetCylinderProp<t_point>(cloud_in,inliers,coefficients_std,coefficients_ext);
  return cratio;
}
//-------------------------------------------------------------------------------------------

/* Get a cylinder's inliers from coefficients.
    coefficients_std: [0-2]: point on axis, [3-5]: axis, [6]: radius,
    coefficients_ext: [0-2]: center x,y,z, [3]: length. */
template <typename t_point>
void GetCylinderInliers(
    const typename pcl::PointCloud<t_point>::ConstPtr &cloud_in,
    pcl::PointIndices::Ptr &inliers_out,
    const pcl::ModelCoefficients::Ptr &coefficients_std,
    const pcl::ModelCoefficients::Ptr &coefficients_ext,
    const double &radius_extension_ratio)
{
  const std::vector<float> &c_std(coefficients_std->values);
  const std::vector<float> &c_ext(coefficients_ext->values);
  Eigen::Vector3d cyl_axis(c_std[3],c_std[4],c_std[5]);
  cyl_axis.normalize();
  Eigen::Vector3d cyl_center(c_ext[0],c_ext[1],c_ext[2]);
  const double length(c_ext[3]);
  const double radius(c_std[6]*radius_extension_ratio);

  // Add a point to inliers if it is inside the cylinder
  inliers_out->indices.clear();
  // double tmin(1.0e10), tmax(-1.0e10);
  for(size_t i(0); i<cloud_in->points.size(); ++i)
  {
    // Project the point on the cylinder axis
    Eigen::Vector3d p(cloud_in->points[i].x,cloud_in->points[i].y,cloud_in->points[i].z);
    double t= (p-cyl_center).transpose()*cyl_axis;
    // If t is not in [-0.5*length,0.5*length], this point is out of the cylinder
    if(t<-0.5*length || t>0.5*length)  continue;
    double dist= (p-(t*cyl_axis+cyl_center)).norm();
    // If dist is not in [0,radius], this point is out of the cylinder
    if(dist>radius)  continue;
    inliers_out->indices.push_back(i);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
enum TPrimitiveKind {pkCylinder=0};
//-------------------------------------------------------------------------------------------

//===========================================================================================
// Grab primitive that models a local grab shape
struct TGrabPrimitive
//===========================================================================================
{
  TPrimitiveKind Kind;
  Eigen::Vector3d  P1, P2;  // Bottom and top points (pkCylinder)
  double           Width;  // Width (pkCylinder)
};
//-------------------------------------------------------------------------------------------

static void ToCylinderCoefficients(
    const TGrabPrimitive &prm,
    pcl::ModelCoefficients::Ptr &coefficients_std,
    pcl::ModelCoefficients::Ptr &coefficients_ext,
    const Eigen::Affine3d &Tbase=Eigen::Affine3d::Identity())
{
  Eigen::Vector3d p1= (Tbase * Eigen::Translation3d(prm.P1)).translation();
  Eigen::Vector3d p2= (Tbase * Eigen::Translation3d(prm.P2)).translation();
  Eigen::Vector3d center= 0.5*(p1+p2);
  Eigen::Vector3d axis= p2-p1;
  double length= axis.norm();
  axis.normalize();
  coefficients_std->values[0]= center[0];
  coefficients_std->values[1]= center[1];
  coefficients_std->values[2]= center[2];
  coefficients_std->values[3]= axis[0];
  coefficients_std->values[4]= axis[1];
  coefficients_std->values[5]= axis[2];
  coefficients_std->values[6]= prm.Width/2.0;
  coefficients_ext->values[0]= center[0];
  coefficients_ext->values[1]= center[1];
  coefficients_ext->values[2]= center[2];
  coefficients_ext->values[3]= length;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// Point cloud primitive
struct TPointCloudPrimitive
//===========================================================================================
{
  TPrimitiveKind Kind;
  pcl::PointCloud<pcl::PointXYZ>::Ptr  Points;
  Eigen::Vector3d  Center;  // for pkCylinder
  Eigen::Vector3d  Axis;  // for pkCylinder
  double  Radius;  // for pkCylinder
  double  Length;  // for pkCylinder
};
//-------------------------------------------------------------------------------------------

static void ToCylinderCoefficients(
    const TPointCloudPrimitive &prm,
    pcl::ModelCoefficients::Ptr &coefficients_std,
    pcl::ModelCoefficients::Ptr &coefficients_ext,
    const Eigen::Affine3d &Tbase=Eigen::Affine3d::Identity())
{
  Eigen::Vector3d center= (Tbase * Eigen::Translation3d(prm.Center)).translation();
  Eigen::Vector3d axis= Tbase.rotation() * prm.Axis;
  coefficients_std->values.resize(7);
  coefficients_std->values[0]= center[0];
  coefficients_std->values[1]= center[1];
  coefficients_std->values[2]= center[2];
  coefficients_std->values[3]= axis[0];
  coefficients_std->values[4]= axis[1];
  coefficients_std->values[5]= axis[2];
  coefficients_std->values[6]= prm.Radius;
  coefficients_ext->values.resize(4);
  coefficients_ext->values[0]= center[0];
  coefficients_ext->values[1]= center[1];
  coefficients_ext->values[2]= center[2];
  coefficients_ext->values[3]= prm.Length;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
/* Detailed container analyzer.  Analyzing an object's point cloud with multiple cylinders.
    coefficients_std: [0-2]: point on axis, [3-5]: axis, [6]: radius,
    coefficients_ext: [0-2]: center x,y,z, [3]: length,
    base_frame: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w,
    pour_edge_ratio: points whose z value is within Length*this from z-max are considered as PourPoints. */
template <typename t_point>
struct TContainerAnalyzer2
//===========================================================================================
{
  typedef boost::shared_ptr<TContainerAnalyzer2<t_point> >  Ptr;
  typedef boost::shared_ptr<const TContainerAnalyzer2<t_point> >  ConstPtr;
  typename pcl::PointCloud<t_point>::Ptr        Cloud;
  typename pcl::PointCloud<pcl::PointXYZ>::Ptr  PourPoints;
  std::vector<TGrabPrimitive>  GrabPrimitives;

  // pcl::PointIndices::Ptr  CylInliers;
  // Eigen::Vector3d  CylCenter;
  // Eigen::Vector3d  CylAxis;
  // double  CylRadius;
  // double  CylLength;

  std::vector<TPointCloudPrimitive>  Cylinders;

  double  Length;
  int     RefMarkerID;  // Reference AR marker on the container
  double  RefMarkerPose[7];  // Its pose

  TContainerAnalyzer2(void)  {}

  bool Analyze(
      typename pcl::PointCloud<t_point>::Ptr &cloud_in,

      int           cyl_normal_est_k,
      const double &cyl_ransac_normal_dist_w,
      const double &cyl_ransac_dist_thresh,
      const double &cyl_ransac_radius_min,
      const double &cyl_ransac_radius_max,
      int           cyl_ransac_max_iterations,
      const double &cyl_ratio_thresh,

      const double &pour_edge_ratio,
      const double &pour_hull_alpha,
      const double &grab_height,
      const double &grab_z_step_ratio,
      const double &grab_radius_sd_thresh_ratio,
      const double ref_marker_id=-1,
      const double *ref_marker_pose=NULL)
    {
      #if 0
      Eigen::Affine3d Tbase, Tf;
      Tbase= XToEigMat(base_frame);
      Tf= Tbase.inverse();
      //*DEBUG*/std::cerr<<"Tbase= "<<Tbase.matrix()<<std::endl;
      //*DEBUG*/std::cerr<<"Tf= "<<Tf.matrix()<<std::endl;
      //*DEBUG*/std::cerr<<"Tbase*Tf= "<<(Tbase*Tf).matrix()<<std::endl;

      // Apply the transformation
      Cloud= typename pcl::PointCloud<t_point>::Ptr(new pcl::PointCloud<t_point>());
      pcl::transformPointCloud(*cloud_in, *Cloud, Tf.matrix().cast<float>());
      #endif
      Cloud= typename pcl::PointCloud<t_point>::Ptr(new pcl::PointCloud<t_point>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gray(new pcl::PointCloud<pcl::PointXYZ>());

      pcl::copyPointCloud(*cloud_in, *Cloud);
      // pcl::copyPointCloud(*Cloud, *cloud_gray);

      //FIXME:create an argument to check if applying MLS
      //FIXME:magic numbers 0.002, 25000
      if(cloud_in->points.size()>25000)
        DownsampleByVoxelGrid<t_point>(Cloud, Cloud, 0.002, 0.002, 0.002);
      //FIXME:magic numbers 0.02
      ApplyMLSFilter<t_point,pcl::PointXYZ>(
          Cloud,
          cloud_gray,
          /*search_radius=*/0.02, /*compute_normals=*/false);

      /*Getting point cloud primitives (Cylinders)*/{
        Cylinders.clear();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*cloud_gray, *cloud_rest);

        while(true)
        {
          // Apply RANSAC to get a cylinder
          pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
          pcl::ModelCoefficients::Ptr cyl_coefficients_std(new pcl::ModelCoefficients);
          pcl::ModelCoefficients::Ptr cyl_coefficients_ext(new pcl::ModelCoefficients);
          double cratio;
          cratio= ExtractCylinder<pcl::PointXYZ>(
              cloud_rest,
              inliers_cylinder,
              cyl_coefficients_std,
              cyl_coefficients_ext,
              cyl_normal_est_k,
              cyl_ransac_normal_dist_w,
              cyl_ransac_dist_thresh,
              cyl_ransac_radius_min,
              cyl_ransac_radius_max,
              cyl_ransac_max_iterations,
              /*cyl_ratio_thresh=*/0.0);

          // Extend the cylinder inliers:
          // FIXME the magic number 1.2
          GetCylinderInliers<pcl::PointXYZ>(cloud_rest, inliers_cylinder,
              cyl_coefficients_std, cyl_coefficients_ext,
              /*radius_extension_ratio=*/1.2);
          cratio= (double)inliers_cylinder->indices.size() / (double)cloud_gray->points.size();

          // No cylinder case:
          // if(cratio < cyl_ratio_thresh)
            // return false;

          // FIXME the magic number 0.1
          if(cratio < 0.1)
            break;

          /*Getting basic cylinder info*/{
            TPointCloudPrimitive  pc_primitive;

            const std::vector<float> &c_std(cyl_coefficients_std->values);
            const std::vector<float> &c_ext(cyl_coefficients_ext->values);

            pc_primitive.Kind= pkCylinder;
            pc_primitive.Points= ExtractByIndices<pcl::PointXYZ>(cloud_rest, inliers_cylinder);
            pc_primitive.Center= Eigen::Vector3d(c_ext[0], c_ext[1], c_ext[2]);
            pc_primitive.Axis= Eigen::Vector3d(c_std[3], c_std[4], c_std[5]);
            if(pc_primitive.Axis[2]<0.0)  pc_primitive.Axis= Eigen::Vector3d(-c_std[3], -c_std[4], -c_std[5]);
            pc_primitive.Axis.normalize();
            pc_primitive.Radius= c_std[6];
            pc_primitive.Length= c_ext[3];

            Cylinders.push_back(pc_primitive);
          }  // Getting basic cylinder info

          /*DEBUG*/std::cerr<<"Cyl inliers, cloud size, rest cloud size: "<<inliers_cylinder->indices.size()<<", "<<cloud_rest->points.size()<<", "<<cloud_rest->points.size()-inliers_cylinder->indices.size()<<std::endl;
          /*DEBUG*/std::cerr<<"  len,rad,center,axis: "<<Cylinders.back().Length<<", "<<Cylinders.back().Radius<<", "<<Cylinders.back().Center.transpose()<<", "<<Cylinders.back().Axis.transpose()<<std::endl;
          // FIXME the magic number 0.1 and 0.2
          if(cloud_rest->points.size()-inliers_cylinder->indices.size() < 0.2*(double)cloud_gray->points.size())
            break;

          // Remove the cylinder inliers
          pcl::ExtractIndices<pcl::PointXYZ> extract_rest;
          extract_rest.setInputCloud(cloud_rest);
          extract_rest.setIndices(inliers_cylinder);
          extract_rest.setNegative(true);
          extract_rest.filter(*cloud_rest);
        }
      }  // Getting point cloud primitives (Cylinders)

      if(ref_marker_id>=0 && ref_marker_pose)
      {
        #if 0
        RefMarkerID= ref_marker_id;
        Eigen::Affine3d  Tref;
        Tref= Tf * XToEigMat(ref_marker_pose);
        EigMatToX(Tref, RefMarkerPose);
        #endif
        RefMarkerID= ref_marker_id;
        for(int d(0); d<7; ++d)  RefMarkerPose[d]= ref_marker_pose[d];
      }

      /*Getting Length and PourPoints*/{
        pcl::PointXYZ pt_min, pt_max;
        pcl::getMinMax3D(*cloud_gray, pt_min, pt_max);
        Length= pt_max.z - pt_min.z;

        // Getting PourPoints
        PourPoints= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ>  pass_z;
        pass_z.setInputCloud(cloud_gray);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(pt_max.z-pour_edge_ratio*Length, pt_max.z);
        pass_z.filter(*edge_points);

        if(edge_points->points.size()>5)  // Otherwise, impossible to compute a concave hull
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr  proj(new pcl::PointCloud<pcl::PointXYZ>()),
              cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());
          pcl::PCA<pcl::PointXYZ> pca;
          pca.setInputCloud(edge_points);
          pca.project(*edge_points, *proj);
          for(size_t i(0); i<proj->points.size(); ++i)  proj->points[i].z= 0.0;
          pca.reconstruct(*proj, *cloud_projected);

          // const float pour_ps2_color[3]= {0,255,255};
          // Get PourPoints as a Concave Hull
          pcl::ConcaveHull<pcl::PointXYZ> chull;
          chull.setInputCloud(cloud_projected);
          chull.setAlpha(pour_hull_alpha);
          chull.reconstruct(*PourPoints);
        }
      }

      /*Getting GrabPrimitives*/{
        GrabPrimitives.clear();

        // Analyze for each point cloud primitives (Cylinders)
        for(int ci(0); ci<Cylinders.size(); ++ci)
        {
          const TPointCloudPrimitive  &pc_prm(Cylinders[ci]);

          bool flag(false);

          // Rotate the cloud along with pc_prm.Axis
          Eigen::Vector3d rot_axis= pc_prm.Axis.cross(Eigen::Vector3d::UnitZ());
          /*DEBUG*/std::cerr<<"rot_axis: "<<rot_axis.transpose()<<std::endl;
          rot_axis.normalize();
          double rot_angle= std::acos(pc_prm.Axis.transpose()*Eigen::Vector3d::UnitZ());
          /*DEBUG*/std::cerr<<"rot_angle: "<<rot_angle<<std::endl;
          Eigen::Affine3d Tcyl;
          Tcyl= Eigen::AngleAxisd(rot_angle,rot_axis) * Eigen::Translation3d(-pc_prm.Center);
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cyl(new pcl::PointCloud<pcl::PointXYZ>());
          pcl::transformPointCloud(*pc_prm.Points, *cloud_cyl, Tcyl.matrix().cast<float>());

          // Get z-min/z-max property
          pcl::PointXYZ pt_min, pt_max;
          pcl::getMinMax3D(*cloud_cyl, pt_min, pt_max);
          pcl::PassThrough<pcl::PointXYZ>  pass_z;
          pass_z.setInputCloud(cloud_cyl);
          pass_z.setFilterFieldName("z");
          double g_len(pt_max.z-pt_min.z);
          /*DEBUG*/std::cerr<<"pt_min: "<<pt_min<<std::endl;
          /*DEBUG*/std::cerr<<"pt_max: "<<pt_max<<std::endl;

          // Extract grabbable points along with pc_prm.Axis
          pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_grabed(new pcl::PointCloud<pcl::PointXYZ>());
          for(double z(pt_min.z+0.5*grab_height); z<pt_max.z-0.5*grab_height; z+= grab_z_step_ratio*(g_len-grab_height))
          {
            pass_z.setFilterLimits(z-0.5*grab_height, z+0.5*grab_height);
            pass_z.filter(*cloud_grabed);
            double r_err_sd(0.0);
            for(int i(0); i<cloud_grabed->points.size(); ++i)
            {
              double r= std::sqrt(Sq(cloud_grabed->points[i].x)+Sq(cloud_grabed->points[i].y));
              r_err_sd+= Sq(r-pc_prm.Radius);
            }
            r_err_sd= std::sqrt(r_err_sd/static_cast<double>(cloud_grabed->points.size()));
            // If true, z is grabbable
            /*DEBUG*/std::cerr<<ci<<" "<<z<<":  "<<r_err_sd<<" / "<<grab_radius_sd_thresh_ratio*pc_prm.Radius<<" ("<<cloud_grabed->points.size()<<std::endl;
            if(r_err_sd < grab_radius_sd_thresh_ratio*pc_prm.Radius)
            {
              Eigen::Vector3d p= (Tcyl.inverse() * Eigen::Translation3d(0,0,z)).translation();
              if(flag)
                GrabPrimitives.back().P2= p;
              else
              {
                GrabPrimitives.push_back(TGrabPrimitive());
                GrabPrimitives.back().Kind= pkCylinder;
                GrabPrimitives.back().P1= p;
                GrabPrimitives.back().P2= p;
                GrabPrimitives.back().Width= 2.0*pc_prm.Radius;
                flag= true;
              }
            }
            else  // z is not grabbable
            {
              if(flag && GrabPrimitives.back().P1==GrabPrimitives.back().P2)
                GrabPrimitives.pop_back();
              flag= false;
            }
          }
          if(flag && GrabPrimitives.back().P1==GrabPrimitives.back().P2)
            GrabPrimitives.pop_back();

        }  // Analyze for each point cloud primitives (Cylinders)
      }  // Getting GrabPrimitives

      return true;
    }

  void Visualize(
      TPCLViewer &viewer,
      const std::string &name,
      const double base_frame[7]=IdentityTransform,
      const float rgb[3]=ColorWhite)
    {
      Eigen::Affine3d Tbase;
      Tbase= XToEigMat(base_frame);
      boost::function<typename pcl::PointCloud<t_point>::Ptr (const typename pcl::PointCloud<t_point>::ConstPtr)>
          tfpc= boost::bind(&TransformCloud<t_point>, _1, Tbase);

      const float pour_ps_color[3]= {255,0,0};
      const float grab_ps_color[3]= {0,255,0};
      viewer.AddPointCloud(tfpc(Cloud), name+"_cloud", 2);
      viewer.AddPointCloud(tfpc(ColorPointCloud(PourPoints,pour_ps_color)), name+"_pour", 3);
      viewer.AddPolygon(tfpc(ColorPointCloud(PourPoints,pour_ps_color)), name+"_pourl", pour_ps_color);

      pcl::ModelCoefficients::Ptr  coefficients_std(new pcl::ModelCoefficients);
      pcl::ModelCoefficients::Ptr  coefficients_ext(new pcl::ModelCoefficients);

      for(int ci(0); ci<Cylinders.size(); ++ci)
      {
        const TPointCloudPrimitive  &pc_prm(Cylinders[ci]);
        if(pc_prm.Kind==pkCylinder)
        {
          ToCylinderCoefficients(pc_prm, coefficients_std, coefficients_ext, Tbase);
          viewer.AddCylinder(coefficients_std, coefficients_ext, GetID(name+"_cyl",ci), 2, ColorWhite);
        }
      }

      for(size_t i(0); i<GrabPrimitives.size(); ++i)
      {
        const TGrabPrimitive &prm(GrabPrimitives[i]);
        if(prm.Kind==pkCylinder)
        {
          ToCylinderCoefficients(prm, coefficients_std, coefficients_ext, Tbase);
          viewer.AddCylinder(coefficients_std, coefficients_ext, GetID(name+"_gcyl",i), 4, grab_ps_color);
        }
      }

      pcl::ModelCoefficients::Ptr sq_coefficients(new pcl::ModelCoefficients);
      sq_coefficients->values.resize(8);
      for(int i(0); i<7; ++i)
        sq_coefficients->values[i]= base_frame[i];
      sq_coefficients->values[7]= detail::MarkerSize;
      viewer.AddSquare(sq_coefficients, name+"_base", 5, rgb);

      double ref_marker_pose[7];
      EigMatToX(Tbase*XToEigMat(RefMarkerPose), ref_marker_pose);
      for(int i(0); i<7; ++i)
        sq_coefficients->values[i]= ref_marker_pose[i];
      sq_coefficients->values[7]= detail::MarkerSize;
      viewer.AddSquare(sq_coefficients, name+"_ref", 4, rgb);
    }

  void PrintAsYAML(std::ostream &os)
    {
      os<<"help: Container model created by TContainerAnalyzer2."<<std::endl;
      os<<"#Reference AR marker ID and pose:"<<std::endl;
      os<<"ref_marker_id: "<<RefMarkerID<<std::endl;
      os<<"ref_marker_pose: ["
          <<RefMarkerPose[0]<<", "<<RefMarkerPose[1]<<", "<<RefMarkerPose[2]<<",  "
          <<RefMarkerPose[3]<<", "<<RefMarkerPose[4]<<", "<<RefMarkerPose[5]<<", "<<RefMarkerPose[6]
          <<"]"<<std::endl;

      os<<"#Set of grab primitives:"<<std::endl;
      os<<"grab_primitives:"<<std::endl;
      for(size_t i(0); i<GrabPrimitives.size(); ++i)
      {
        const TGrabPrimitive &prm(GrabPrimitives[i]);
        if(prm.Kind==pkCylinder)
        {
          os<<"- kind: "<<"pkCylinder"<<std::endl;
          os<<"  p1: "<<"["<<prm.P1[0]<<", "<<prm.P1[1]<<", "<<prm.P1[2]<<"]"<<std::endl;
          os<<"  p2: "<<"["<<prm.P2[0]<<", "<<prm.P2[1]<<", "<<prm.P2[2]<<"]"<<std::endl;
          os<<"  width: "<<prm.Width<<std::endl;
        }
      }

      os<<"#Pouring edge point candidates:"<<std::endl;
      os<<"l_p_pour_e_set:"<<std::endl;
      for(size_t i(0); i<PourPoints->points.size(); ++i)
      {
        os<<"- [" <<PourPoints->points[i].x<<", "
                  <<PourPoints->points[i].y<<", "
                  <<PourPoints->points[i].z<<"]"<<std::endl;
      }
    }

};
//-------------------------------------------------------------------------------------------





//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // cyl_detector_h
//-------------------------------------------------------------------------------------------
