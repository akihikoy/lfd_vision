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
namespace detail {extern std::map<int,TARMarker>  ARMarkers;}
static const float ColorWhite[3]= {255,255,255};
static const float ColorGray[3]= {128,128,128};
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline t_value Sq(const t_value &val)
{
  return val*val;
}
//-------------------------------------------------------------------------------------------

inline std::string GetID(const std::string &base, int idx)
{
  std::stringstream ss;
  ss<<base<<idx;
  return ss.str();
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
    const Eigen::Vector3d &focusing_point,
    const TARMarker &base_ar_marker);
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
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer_->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
      viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
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

  void RemoveAll()
    {
      viewer_->removeAllPointClouds();
    }

private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer_;

};
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
bool RemovePlains(
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

  int nr_points = cloud_io->points.size ();
  while (cloud_io->points.size() > non_planar_points_ratio*nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_io);
    seg.segment (*inliers, *coefficients); //*
    if (inliers->indices.size () == 0)
    {
      std::cerr<<"Error: Could not estimate a planar model for the given dataset."<<std::endl;
      return false;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<t_point> extract;
    extract.setInputCloud (cloud_io);
    extract.setIndices (inliers);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_io);
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
  pcl::copyPointCloud(*cloud_in, *cloud_out);
  for (int i(0); i<cloud_out->points.size(); ++i)
  {
    cloud_out->points[i].r = rgb[0];
    cloud_out->points[i].g = rgb[1];
    cloud_out->points[i].b = rgb[2];
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
  seg.segment (*inliers, *coefficients_std);

  double cratio= (double)inliers->indices.size() / (double)cloud_in->points.size();
  if(cratio<cylinder_ratio_thresh)  return cratio;

  GetCylinderProp<t_point>(cloud_in,inliers,coefficients_std,coefficients_ext);
  return cratio;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
/* Detailed container analyzer.
    coefficients_std: [0-2]: point on axis, [3-5]: axis, [6]: radius,
    coefficients_ext: [0-2]: center x,y,z, [3]: length,
    base_frame: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w,
    pour_edge_ratio: points whose z value is within Length*this from z-max are considered as PourPoints. */
template <typename t_point>
struct TContainerProperty
//===========================================================================================
{
  typename pcl::PointCloud<t_point>::Ptr  Cloud;
  typename pcl::PointCloud<pcl::PointXYZ>::Ptr  PourPoints;
  typename pcl::PointCloud<pcl::PointXYZ>::Ptr  GrabPoints;
  pcl::PointIndices::Ptr  CylInliers;
  Eigen::Vector3d  CylCenter;
  Eigen::Vector3d  CylAxis;
  double  CylRadius;
  double  CylLength;
  double  Length;

  TContainerProperty(
      typename pcl::PointCloud<t_point>::Ptr &cloud_in,
      pcl::PointIndices::Ptr &inliers,
      const pcl::ModelCoefficients::Ptr &coefficients_std,
      const pcl::ModelCoefficients::Ptr &coefficients_ext,
      const double base_frame[7],
      const double &pour_edge_ratio,
      const double &grab_height,
      const double &grab_z_step_ratio,
      const double &grab_radius_sd_thresh_ratio)
    {
      const std::vector<float> &c_std(coefficients_std->values);
      const std::vector<float> &c_ext(coefficients_ext->values);

      Eigen::Affine3d Tbase, Tf;
      Tbase= Eigen::Translation3d(base_frame[0],base_frame[1],base_frame[2])
              * Eigen::Quaterniond(base_frame[6],base_frame[3],base_frame[4],base_frame[5]);
      Tf= Tbase.inverse();
      //*DEBUG*/std::cerr<<"Tbase= "<<Tbase.matrix()<<std::endl;
      //*DEBUG*/std::cerr<<"Tf= "<<Tf.matrix()<<std::endl;
      //*DEBUG*/std::cerr<<"Tbase*Tf= "<<(Tbase*Tf).matrix()<<std::endl;

      // Apply the transformation
      Cloud= typename pcl::PointCloud<t_point>::Ptr(new pcl::PointCloud<t_point>());
      pcl::transformPointCloud(*cloud_in, *Cloud, Tf.matrix().cast<float>());

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gray(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*Cloud, *cloud_gray);

      CylInliers= inliers;
      CylCenter= (Tf * Eigen::Translation3d(c_ext[0], c_ext[1], c_ext[2])).translation();
      CylAxis= Tf.rotation() * Eigen::Vector3d(c_std[3], c_std[4], c_std[5]);
      if(CylAxis[2]<0.0)  CylAxis= Tf.rotation() * Eigen::Vector3d(-c_std[3], -c_std[4], -c_std[5]);
      CylAxis.normalize();
      CylRadius= c_std[6];
      CylLength= c_ext[3];
      //*DEBUG*/std::cerr<<"cyl_axis= "<<cyl_axis.transpose()<<std::endl;
      //*DEBUG*/std::cerr<<"CylAxis= "<<CylAxis.transpose()<<std::endl;
      //*DEBUG*/std::cerr<<"cyl_axis2= "<<(Tbase.rotation()*CylAxis).transpose()<<std::endl;


      /*Getting Length and PourPoints...*/{
        pcl::PointXYZ pt_min, pt_max;
        pcl::getMinMax3D(*cloud_gray, pt_min, pt_max);
        Length= pt_max.z - pt_min.z;

        PourPoints= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ>  pass_z;
        pass_z.setInputCloud(cloud_gray);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(pt_max.z-pour_edge_ratio*Length, pt_max.z);
        pass_z.filter(*PourPoints);
      }

      /*Getting GrabPoints...*/{
        // Rotate the cloud along with CylAxis
        Eigen::Vector3d rot_axis= CylAxis.cross(Eigen::Vector3d::UnitZ());
        rot_axis.normalize();
        double rot_angle= std::acos(CylAxis.transpose()*Eigen::Vector3d::UnitZ());
        Eigen::Affine3d Tcyl;
        Tcyl= Eigen::AngleAxisd(rot_angle,rot_axis);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cyl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_gray, *cloud_cyl, Tcyl.matrix().cast<float>());

        // Get z-min/z-max property
        pcl::PointXYZ pt_min, pt_max;
        pcl::getMinMax3D(*cloud_cyl, pt_min, pt_max);
        pcl::PassThrough<pcl::PointXYZ>  pass_z;
        pass_z.setInputCloud(cloud_cyl);
        pass_z.setFilterFieldName("z");
        double g_len(pt_max.z-pt_min.z);

        // Extract grabbable points along with CylAxis
        pcl::PointCloud<pcl::PointXYZ>::Ptr  grab_points_cyl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_grabed(new pcl::PointCloud<pcl::PointXYZ>());
        for(double z(pt_min.z+0.5*grab_height); z<pt_max.z-0.5*grab_height; z+= grab_z_step_ratio*(g_len-grab_height))
        {
          pass_z.setFilterLimits(z-0.5*grab_height, z+0.5*grab_height);
          pass_z.filter(*cloud_grabed);
          double r_err_sd(0.0);
          for(int i(0); i<cloud_grabed->points.size(); ++i)
          {
            double r= std::sqrt(Sq(cloud_grabed->points[i].x)+Sq(cloud_grabed->points[i].y));
            r_err_sd+= Sq(r-CylRadius);
          }
          r_err_sd= std::sqrt(r_err_sd/static_cast<double>(cloud_grabed->points.size()));
          if(r_err_sd < grab_radius_sd_thresh_ratio*CylRadius)
            grab_points_cyl->points.push_back(pcl::PointXYZ(0.0, 0.0, z));
        }
        grab_points_cyl->width    = grab_points_cyl->points.size();
        grab_points_cyl->height   = 1;
        grab_points_cyl->is_dense = false;

        // Inv transform the grabbable points onto the base frame
        GrabPoints= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*grab_points_cyl, *GrabPoints, Tcyl.inverse().matrix().cast<float>());
      }  // Getting GrabPoints
    }

  void Visualize(TPCLViewer &viewer, const std::string &name)
    {
      const float pour_ps_color[3]= {255,0,0};
      const float grab_ps_color[3]= {0,255,0};
      viewer.AddPointCloud(Cloud, name+"_cloud", 1);
      viewer.AddPointCloud(ColorPointCloud(PourPoints,pour_ps_color), name+"_pour", 3);
      viewer.AddPointCloud(ColorPointCloud(GrabPoints,grab_ps_color), name+"_grab", 5);

      pcl::ModelCoefficients::Ptr  coefficients_std(new pcl::ModelCoefficients);
      pcl::ModelCoefficients::Ptr  coefficients_ext(new pcl::ModelCoefficients);
      coefficients_std->values.resize(7);
      coefficients_std->values[0]= CylCenter[0];
      coefficients_std->values[1]= CylCenter[1];
      coefficients_std->values[2]= CylCenter[2];
      coefficients_std->values[3]= CylAxis[0];
      coefficients_std->values[4]= CylAxis[1];
      coefficients_std->values[5]= CylAxis[2];
      coefficients_std->values[6]= CylRadius;
      coefficients_ext->values.resize(4);
      coefficients_ext->values[0]= CylCenter[0];
      coefficients_ext->values[1]= CylCenter[1];
      coefficients_ext->values[2]= CylCenter[2];
      coefficients_ext->values[3]= CylLength;
      viewer.AddCylinder(coefficients_std, coefficients_ext, name+"_cyl", 3);
    }

  void PrintAsPyFormat(std::ostream &os, const double &g_width_margin)
    {
      // typename pcl::PointCloud<t_point>::Ptr  Cloud;
      // typename pcl::PointCloud<pcl::PointXYZ>::Ptr  PourPoints;
      // typename pcl::PointCloud<pcl::PointXYZ>::Ptr  GrabPoints;
      // Eigen::Vector3d  CylCenter;
      // Eigen::Vector3d  CylAxis;
      // double  CylRadius;
      // double  CylLength;
      // double  Length;

      Eigen::Vector3d rot_axis= Eigen::Vector3d::UnitZ().cross(CylAxis);
      rot_axis.normalize();
      double rot_angle= std::acos(Eigen::Vector3d::UnitZ().transpose()*CylAxis);
      Eigen::Quaterniond q_cyl(Eigen::AngleAxisd(rot_angle,rot_axis));

      // #Gripper width to grab
      // t.attributes['b1']['g_width']= 0.08
      // #Grab pose:
      // t.attributes['b1']['l_x_grab']= [0.0, 0.0, 0.015, -0.0386798980774, 0.0474739514813, 0.0058252014884, 0.998106285144]
      // #Pouring edge point:
      // t.attributes['b1']['l_x_pour_e']= [0.0, -0.04, 0.11, 0.0,0.0,0.0,1.0]

      os<<"#Gripper width to grab:"<<std::endl;
      os<<"attribute['g_width']= "<<2.0*CylRadius*g_width_margin<<std::endl;

      std::string delim;
      delim= "\n  ";
      os<<"#Pouring edge point candidates:"<<std::endl;
      os<<"attribute['l_x_pour_e_set']= [";
      for(size_t i(0); i<PourPoints->points.size(); ++i)
      {
        os<<delim<<"["<<PourPoints->points[i].x<<", "
                      <<PourPoints->points[i].y<<", "
                      <<PourPoints->points[i].z<<",  "
                      <<" 0.0,0.0,0.0,1.0]";
        delim= ",\n  ";
      }
      os<<std::endl;

      delim= "\n  ";
      os<<"#Grab pose candidates:"<<std::endl;
      os<<"attribute['l_x_grab_set']= [";
      for(size_t i(0); i<GrabPoints->points.size(); ++i)
      {
        os<<delim<<"["<<GrabPoints->points[i].x<<", "
                      <<GrabPoints->points[i].y<<", "
                      <<GrabPoints->points[i].z<<",  "
                      <<q_cyl.x()<<", "<<q_cyl.y()<<", "<<q_cyl.z()<<", "<<q_cyl.w()<<"]";
        delim= ",\n  ";
      }
      os<<std::endl;
    }

};
//-------------------------------------------------------------------------------------------





//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // cyl_detector_h
//-------------------------------------------------------------------------------------------
