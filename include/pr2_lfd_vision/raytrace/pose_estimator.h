//-------------------------------------------------------------------------------------------
/*! \file    pose_estimator.h
    \brief   Pose estimation using ray tracing
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef pose_estimator_h
#define pose_estimator_h
//-------------------------------------------------------------------------------------------
#include "depthscene.h"
#include "pr2_lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include <cmath>
#include <opencv2/core/core.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

inline double Rad2Deg(const double &q)  {return q*180.0/M_PI;}
inline double Deg2Rad(const double &q)  {return q*M_PI/180.0;}
//-------------------------------------------------------------------------------------------

// Compute YZX-Euler angles from quaternion.
// ref. http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
inline void QuaternionToYZXEuler(
    const double &qx, const double &qy, const double &qz, const double &qw,
    double &ey, double &ez, double &ex)
{
  double sqw = qw*qw;
  double sqx = qx*qx;
  double sqy = qy*qy;
  double sqz = qz*qz;
  double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
  double test = qx*qy + qz*qw;
  if (test > 0.499*unit) { // singularity at north pole
          ey = 2.0 * std::atan2(qx,qw);
          ez = M_PI/2.0;
          ex = 0.0;
          return;
  }
  if (test < -0.499*unit) { // singularity at south pole
          ey = -2.0 * std::atan2(qx,qw);
          ez = -M_PI/2.0;
          ex = 0.0;
          return;
  }
  ey = std::atan2(2.0*qy*qw-2.0*qx*qz , sqx - sqy - sqz + sqw);
  ez = std::asin(2.0*test/unit);
  ex = std::atan2(2.0*qx*qw-2.0*qy*qz , -sqx + sqy - sqz + sqw);
}
//-------------------------------------------------------------------------------------------


enum TRayTracePrimitiveKind{
  rtpkSphere    =0,  // Param={radius}
  rtpkSpheroid    ,  // Param={radius_x,radius_y,radius_z}
  rtpkCuboid      ,  // Param={len_x,len_y,len_z}
  rtpkCylinder    ,  // Param={radius,height}
  rtpkTube        ,  // Param={radius_out,radius_in,height}
  rtpkTorus       ,
  rtpkPolyhedra   };

struct TRayTraceModel
{
  struct TPrimitive
  {
    TRayTracePrimitiveKind Kind;
    double Param[3];
    double Pose[7];  // x,y,z, qx,qy,qz,qw
  };
  std::vector<TPrimitive> Primitives;
  double BoundingRadius;  // If positive, BoundingRadius is automatically computed

  TRayTraceModel() : Primitives(), BoundingRadius(-1.0)  {}
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TRayTracePoseEstimator
//===========================================================================================
{
public:
  // Rotation of an object in YZX-Euler angles in DEGREE.
  struct TDegRotation {double Y,Z,X;  TDegRotation():Y(0.0),Z(0.0),X(0.0){}};
  struct TRadRotation {double Y,Z,X;  TRadRotation():Y(0.0),Z(0.0),X(0.0){}};
  static inline void Deg2Rad3(const TDegRotation &deg, TRadRotation &rad)
    {rad.X=Deg2Rad(deg.X); rad.Y=Deg2Rad(deg.Y); rad.Z=Deg2Rad(deg.Z);}
  static inline void Rad2Deg3(const TRadRotation &rad, TDegRotation &deg)
    {deg.X=Rad2Deg(rad.X); deg.Y=Rad2Deg(rad.Y); deg.Z=Rad2Deg(rad.Z);}

  static inline cv::Vec3f GetVisualNormal(const Imager::Vector &n)
    {
      cv::Vec3f col;
      trick::GetVisualNormal(n.x,n.y,n.z, col(0),col(1),col(2));
      return col;
    }

public:
  // Add an object to the scene and return its index
  int AddObject(const TRayTraceModel &model, const double pose[7]);

  // Set x,y,z of the object
  void SetXYZ(int index, const double xyz[3]);
  // Set rotation ex,ey,ez(in radian) of the object
  void SetRotation(int index, const double &ex, const double &ey, const double &ez);
  // Set quaternion qx,qy,qz,qw of the object
  void SetQ(int index, const double q[4]);
  // Set pose=x,y,z,quaternion of the object
  void SetPose(int index, const double pose[7]);

  void UpdateROI();

  void Render(
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp=2, int step_yp=2);

  void GetDistance(
      cv::Mat &depth_img, cv::Mat &normal_img,
      double &sqdiff_depth, double &sqdiff_normal,
      int step_xp=2, int step_yp=2);

  void OptimizeXY(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      double xy_opt[2], double eval_opt[1]);
  void OptimizeZ(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      double z_opt[1], double eval_opt[1]);
  void OptimizeXYZ(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp=2, int step_yp=2,
      double position_opt[3]=NULL, double eval_opt[2]=NULL);

  // Handle a key event where c is the output of cv::waitKey
  bool HandleKeyEvent(int index, int c);

  void SetCameraInfo(const Imager::TCameraInfo &cam)  {camera_= cam;}
  const Imager::TCameraInfo& CameraInfo() const {return camera_;}

  void SetROI(const Imager::TROI &roi)  {roi_= roi;}
  const Imager::TROI& ROI() const {return roi_;}

private:
  Imager::DepthScene scene_;
  std::vector<TDegRotation>  rotations_;
  Imager::TCameraInfo camera_;
  Imager::TROI roi_;

};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // pose_estimator_h
//-------------------------------------------------------------------------------------------
