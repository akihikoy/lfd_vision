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
#include "../geom_util.h"
//-------------------------------------------------------------------------------------------
#include <map>
#include <cmath>
#include <iostream>
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
  rtpkInvalid   =-1,
  rtpkSphere    =0,  // Param={radius}
  rtpkSpheroid    ,  // Param={radius_x,radius_y,radius_z}
  rtpkCuboid      ,  // Param={half_len_x,half_len_y,half_len_z}
  rtpkCylinder    ,  // Param={radius,height}
  rtpkHalfCylinder,  // Param={radius,height}; Only y>=0 part.
  rtpkTube        ,  // Param={radius_out,radius_in,height,dx,dy}, dx,dy: displacement of hole position
  rtpkHalfTube    ,  // Param={radius_out,radius_in,height,dx,dy}, dx,dy: displacement of hole position; Only y>=0 part.
  rtpkTorus       ,
  rtpkPolyhedra   };

struct TRayTraceModel
{
  struct TPrimitive
  {
    TRayTracePrimitiveKind Kind;
    double Param[5];
    double Pose[7];  // x,y,z, qx,qy,qz,qw
  };
  std::vector<TPrimitive> Primitives;

  TRayTraceModel() : Primitives()  {}
};
//-------------------------------------------------------------------------------------------

inline TRayTracePrimitiveKind StrToRTPrimitiveKind(const std::string &kind)
{
  if(kind=="rtpkSphere"       )   return rtpkSphere        ;
  if(kind=="rtpkSpheroid"     )   return rtpkSpheroid      ;
  if(kind=="rtpkCuboid"       )   return rtpkCuboid        ;
  if(kind=="rtpkCylinder"     )   return rtpkCylinder      ;
  if(kind=="rtpkHalfCylinder" )   return rtpkHalfCylinder  ;
  if(kind=="rtpkTube"         )   return rtpkTube          ;
  if(kind=="rtpkHalfTube"     )   return rtpkHalfTube      ;
  if(kind=="rtpkTorus"        )   return rtpkTorus         ;
  if(kind=="rtpkPolyhedra"    )   return rtpkPolyhedra     ;
  return rtpkInvalid;
}
inline std::string RTPrimitiveKindToStr(const TRayTracePrimitiveKind &kind)
{
  switch(kind)
  {
  case rtpkSphere        :  return "rtpkSphere"       ;
  case rtpkSpheroid      :  return "rtpkSpheroid"     ;
  case rtpkCuboid        :  return "rtpkCuboid"       ;
  case rtpkCylinder      :  return "rtpkCylinder"     ;
  case rtpkHalfCylinder  :  return "rtpkHalfCylinder" ;
  case rtpkTube          :  return "rtpkTube"         ;
  case rtpkHalfTube      :  return "rtpkHalfTube"     ;
  case rtpkTorus         :  return "rtpkTorus"        ;
  case rtpkPolyhedra     :  return "rtpkPolyhedra"    ;
  }
  return "";
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TRayTracePoseEstimator
//===========================================================================================
{
public:
  struct TPose {double X[7]; /*[0-2]: position x,y,z, [3-6]: orientation x,y,z,w.*/};

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
  inline void SetXYZ(int index, const double &x, const double &y, const double &z);
  // Set x,y,z of the object
  inline void SetXYZ(int index, const double xyz[3]);
  // Set quaternion qx,qy,qz,qw of the object
  inline void SetQ(int index, const double q[4]);
  // Set pose=x,y,z,quaternion of the object
  inline void SetPose(int index, const double pose[7]);

  Imager::TROI2D<int> GetImageROI() const;

  void Render(
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp=2, int step_yp=2);

  /*Get a distance between the ray traced model image and actual images.
    index : if -1, all rendered intersections are considered,
        if >=0, only intersections whose solid is mapped to index are considered.
    depth_img, normal_img : depth and normal images.
    sqdiff_depth, sqdiff_normal : square errors of valid depth and normal points.
    n_invalid_depth : number of invalid depth pixels in depth_img on the model image.
    n_invalid_normal : number of invalid normal pixels in normal_img on the model image.
    n_invalid_range : number of invalid pixels that is out of range of the camera.
    step_xp, step_yp : step size to compute the model image.  Greater is faster but bigger error.
  */
  void GetDistance(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      double &sqdiff_depth, double &sqdiff_normal,
      int &n_invalid_depth, int &n_invalid_normal, int &n_invalid_range,
      int step_xp=2, int step_yp=2);

  void OptimizeXY(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      const double &range_x, const double &range_y, const double &n_div,
      const double &w_depth, const double &w_normal,
      double xy_opt[2], double eval_opt[2]);
  void OptimizeZ(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      const double &range_z, const double &n_div,
      const double &w_depth, const double &w_normal,
      double z_opt[1], double eval_opt[2]);
  void OptimizeXYZ(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp=2, int step_yp=2,
      double position_opt[3]=NULL, double eval_opt[2]=NULL);

  void OptimizeLin2D(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      const double axis_1[3], const double axis_2[3],
      const double &range_1, const double &range_2, const double &n_div,
      const double &w_depth, const double &w_normal,
      double opt_12[2], double position_opt[3], double eval_opt[2]);

  // Handle a key event where c is the output of cv::waitKey
  bool HandleKeyEvent(int index, int c);

  void SetCameraInfo(const Imager::TCameraInfo &cam)  {camera_= cam;}
  const Imager::TCameraInfo& CameraInfo() const {return camera_;}

private:
  Imager::DepthScene scene_;
  std::vector<TPose>  poses_;  // [index]= pose; current poses
  std::vector<TDegRotation>  rotations_;  // [index]= rotation; having current Euler rotation since objects in scene_ does not have it.
  std::vector<Imager::TROI3D>  local_roi_;  // [index]= local ROI
  std::map<const Imager::SolidObject*,int>  solid_to_index_;  // [*solid]= index; to know the object index of given solid pointer (of an intersection)
  Imager::TCameraInfo camera_;

  // Set rotation ex,ey,ez(in radian) of the object
  // WARNING: this function does not update poses_; use: SetPose, SetQ, SetXYZ
  inline void set_rotation(int index, const double &ex, const double &ey, const double &ez);

};
//-------------------------------------------------------------------------------------------


// Set x,y,z of the object
inline void TRayTracePoseEstimator::SetXYZ(int index, const double &x, const double &y, const double &z)
{
  Imager::SolidObject *obj= scene_.RefSolidObject(index);
  obj->Move(x,y,z);
  poses_[index].X[0]= x;
  poses_[index].X[1]= y;
  poses_[index].X[2]= z;
}
//-------------------------------------------------------------------------------------------

// Set x,y,z of the object
inline void TRayTracePoseEstimator::SetXYZ(int index, const double xyz[3])
{
  SetXYZ(index, xyz[0], xyz[1], xyz[2]);
}
//-------------------------------------------------------------------------------------------

// Set rotation ex,ey,ez(in radian) of the object
// WARNING: this function does not update poses_; use: SetPose, SetQ, SetXYZ
inline void TRayTracePoseEstimator::set_rotation(int index, const double &ex, const double &ey, const double &ez)
{
  Imager::SolidObject *obj= scene_.RefSolidObject(index);
  TDegRotation &rot(rotations_[index]);
  // Reset rotation (inverse YZX):
  obj->RotateY(-rot.Y);
  obj->RotateZ(-rot.Z);
  obj->RotateX(-rot.X);
  // Rotate YZX:
  rot.X= Rad2Deg(ex);
  rot.Z= Rad2Deg(ez);
  rot.Y= Rad2Deg(ey);
  obj->RotateX(rot.X);
  obj->RotateZ(rot.Z);
  obj->RotateY(rot.Y);
}
//-------------------------------------------------------------------------------------------

// Set quaternion qx,qy,qz,qw of the object
inline void TRayTracePoseEstimator::SetQ(int index, const double q[4])
{
  double ey(0.0),ez(0.0),ex(0.0);
  QuaternionToYZXEuler(q[0],q[1],q[2],q[3], ey,ez,ex);
  set_rotation(index, ex,ey,ez);
  for(int d(0);d<4;++d)  poses_[index].X[3+d]= q[d];
}
//-------------------------------------------------------------------------------------------

// Set pose=x,y,z,quaternion of the object
inline void TRayTracePoseEstimator::SetPose(int index, const double pose[7])
{
  SetXYZ(index, pose);
  SetQ(index, pose+3);
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // pose_estimator_h
//-------------------------------------------------------------------------------------------
