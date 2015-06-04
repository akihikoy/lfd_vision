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
  rtpkRectTube    ,  // Param={half_len_x_out,half_len_y_out,half_len_z, half_len_x_in,half_len_y_in,dx,dy}, dx,dy: displacement of hole position
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
  if(kind=="rtpkRectTube"     )   return rtpkRectTube      ;
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
  case rtpkRectTube      :  return "rtpkRectTube"     ;
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
  // struct TPose {double X[7]; /*[0-2]: position x,y,z, [3-6]: orientation x,y,z,w.*/};

  // Rotation of an object in YZX-Euler angles in DEGREE.
  struct TDegRotation {double Y,Z,X;  TDegRotation():Y(0.0),Z(0.0),X(0.0){}};
  struct TRadRotation {double Y,Z,X;  TRadRotation():Y(0.0),Z(0.0),X(0.0){}};
  static inline void Deg2Rad3(const TDegRotation &deg, TRadRotation &rad)
    {rad.X=Deg2Rad(deg.X); rad.Y=Deg2Rad(deg.Y); rad.Z=Deg2Rad(deg.Z);}
  static inline void Rad2Deg3(const TRadRotation &rad, TDegRotation &deg)
    {deg.X=Rad2Deg(rad.X); deg.Y=Rad2Deg(rad.Y); deg.Z=Rad2Deg(rad.Z);}

  // Description for evaluation
  struct TEvalDescription
    {
      double SqDiffDepth, SqDiffNormal;  // square errors of valid depth and normal points.
      int NumNoDepth;    // number of unobserved depth pixels on depth_img in NumInsideCam of the model image.
      int NumNoNormal;   // number of unobserved normal pixels on normal_img in NumInsideCam of the model image.
      int NumInsideCam;      // number of pixels that are inside the range of camera.
      int NumOutOfCam;        // number of pixels that are outside the range of the camera.
      int NumMatchedDepth;    // number of matched depth pixels in depth_img in NumInsideCam of the model image (threshold is used).
      int NumMatchedNormal;   // number of matched normal pixels in normal_img in NumInsideCam of the model image (threshold is used).
      // NOTE: number of unmatched * pixels = NumInsideCam - (NumMatched* + NumNo*)

      // Cache of evaluations
      mutable double WError;
      mutable double RMatchedDepth, RUnmatchedDepth, RNoDepth;
      mutable double RMatchedNormal, RUnmatchedNormal, RNoNormal;
      mutable int Quality;

      TEvalDescription()
        : SqDiffDepth(0.0),
          SqDiffNormal(0.0),
          NumNoDepth(0),
          NumNoNormal(0),
          NumInsideCam(0),
          NumOutOfCam(0),
          NumMatchedDepth(0),
          NumMatchedNormal(0),
          WError(0.0),
          RMatchedDepth(0.0), RUnmatchedDepth(0.0), RNoDepth(0.0),
          RMatchedNormal(0.0), RUnmatchedNormal(0.0), RNoNormal(0.0),
          Quality(0)
        {}
    };

  static inline cv::Vec3f GetVisualNormal(const Imager::Vector &n)
    {
      cv::Vec3f col;
      trick::GetVisualNormal(n.x,n.y,n.z, col(0),col(1),col(2));
      return col;
    }

public:
  TRayTracePoseEstimator()
    : sq_diff_depth_thresh_(0.0),
      sq_diff_normal_thresh_(0.0),
      nodata_sq_diff_depth_(0.0),
      nodata_sq_diff_normal_(0.0),
      th_good_depth_ratio_(0.3),
      th_bad_depth_ratio_(0.7),
      th_good_normal_ratio_(0.1),
      th_bad_normal_ratio_(0.8)    {}

  // Add an object to the scene and return its index
  int AddObject(const TRayTraceModel &model, const double pose[7]);

  // Get pose=x,y,z,quaternion of the object
  const TPose& Pose(int index) const {return poses_[index];}

  // Set x,y,z of the object
  inline void SetXYZ(int index, const double &x, const double &y, const double &z);
  // Set x,y,z of the object
  inline void SetXYZ(int index, const double xyz[3]);
  // Set quaternion qx,qy,qz,qw of the object
  inline void SetQ(int index, const double q[4]);
  // Set pose=x,y,z,quaternion of the object
  inline void SetPose(int index, const double pose[7]);

  bool GetImageROI(Imager::TROI2D<int> &img_roi) const;

  void Render(
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp=2, int step_yp=2);

  /*Get a description for evaluation which is measurements between
    the ray traced model image and actual images.
    index : if -1, all rendered intersections are considered,
        if >=0, only intersections whose solid is mapped to index are considered.
    depth_img, normal_img : depth and normal images.
    desc : obtained description.
    step_xp, step_yp : step size to compute the model image.  Greater is faster but bigger error.
    f_depth_normalize: normalization factor of depth.
    Return true if desc is valid.
  */
  bool GetEvalDescription(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      TEvalDescription &desc, int step_xp=2, int step_yp=2,
      const double &f_depth_normalize=1.0);

  // void OptimizeXY(
      // int index,
      // cv::Mat &depth_img, cv::Mat &normal_img,
      // int step_xp, int step_yp,
      // const double &range_x, const double &range_y, const double &n_div,
      // const double &w_depth, const double &w_normal,
      // double xy_opt[2], TEvalDescription *eval_desc_opt);
  // void OptimizeZ(
      // int index,
      // cv::Mat &depth_img, cv::Mat &normal_img,
      // int step_xp, int step_yp,
      // const double &range_z, const double &n_div,
      // const double &w_depth, const double &w_normal,
      // double z_opt[1], TEvalDescription *eval_desc_opt);
  void OptimizeXYZ(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp=2, int step_yp=2,
      double position_opt[3]=NULL, TEvalDescription *eval_desc_opt=NULL);

  void OptimizeLin1D(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      const double axis_1[3],
      const double &range_1, const double &n_div,
      const double &w_depth, const double &w_normal,
      double opt_1[1], double position_opt[3], TEvalDescription *eval_desc_opt=NULL);
  void OptimizeLin2D(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      const double axis_1[3], const double axis_2[3],
      const double &range_1, const double &range_2, const double &n_div,
      const double &w_depth, const double &w_normal,
      double opt_12[2], double position_opt[3], TEvalDescription *eval_desc_opt=NULL);

  void OptimizeRot1D(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      const double axis_1[3],
      const double &range_1, const double &n_div,
      const double &w_depth, const double &w_normal,
      double opt_1[1], double rotation_opt[4], TEvalDescription *eval_desc_opt=NULL);
  void OptimizeRot2D(
      int index,
      cv::Mat &depth_img, cv::Mat &normal_img,
      int step_xp, int step_yp,
      const double axis_1[3], const double axis_2[3],
      const double &range_1, const double &range_2, const double &n_div,
      const double &w_depth, const double &w_normal,
      double opt_12[2], double rotation_opt[4], TEvalDescription *eval_desc_opt=NULL);

  /* Compare two evaluation descriptions,
      return
        -1: desc1 < desc2 (desc1 is worse),
        0 : desc1 == desc2 (equal),
        +1: desc1 > desc2 (desc1 is better).
      w_depth: weight of depth-img difference.
      w_normal: weight of normal-img difference. */
  inline int CompareEvalDescriptions(
      const TEvalDescription &desc1, const TEvalDescription &desc2,
      const double &w_depth, const double &w_normal)
    {
      // return CompareEvalDescriptions1(desc1, desc2, w_depth, w_normal);
      return CompareEvalDescriptions2(desc1, desc2, w_depth, w_normal);
    }

  /* Check an evaluation description satisfies a threshold (ver 2),
      return true if satisfied.
      w_depth: weight of depth-img difference.
      w_normal: weight of normal-img difference. */
  inline bool CheckEvalDescriptions(
      const TEvalDescription &desc, const TEvalDescription &threshold,
      const double &w_depth, const double &w_normal)
    {
      return CheckEvalDescriptions2(desc, threshold, w_depth, w_normal);
    }

  inline int EvaluateDescription1(
      const TEvalDescription &desc, const double &w_depth, const double &w_normal);
  inline int CompareEvalDescriptions1(
      const TEvalDescription &desc1, const TEvalDescription &desc2,
      const double &w_depth, const double &w_normal);

  inline int EvaluateDescription2(
      const TEvalDescription &desc, const double &w_depth, const double &w_normal);
  inline int CompareEvalDescriptions2(
      const TEvalDescription &desc1, const TEvalDescription &desc2,
      const double &w_depth, const double &w_normal);
  inline bool CheckEvalDescriptions2(
      const TEvalDescription &desc, const TEvalDescription &threshold,
      const double &w_depth, const double &w_normal);


  // Handle a key event where c is the output of cv::waitKey
  bool HandleKeyEvent(int index, int c);

  void SetCameraInfo(const Imager::TCameraInfo &cam)  {camera_= cam;}
  const Imager::TCameraInfo& CameraInfo() const {return camera_;}

  const double& SqDiffDepthThresh() const {return sq_diff_depth_thresh_;}
  const double& SqDiffNormalThresh() const {return sq_diff_normal_thresh_;}
  const double& NodataSqDiffDepth() const {return nodata_sq_diff_depth_;}
  const double& NodataSqDiffNormal() const {return nodata_sq_diff_normal_;}
  const double& ThGoodDepthRatio () const {return th_good_depth_ratio_ ;}
  const double& ThBadDepthRatio  () const {return th_bad_depth_ratio_  ;}
  const double& ThGoodNormalRatio() const {return th_good_normal_ratio_;}
  const double& ThBadNormalRatio () const {return th_bad_normal_ratio_ ;}
  void SetSqDiffDepthThresh(const double &v)  {sq_diff_depth_thresh_= v;}
  void SetSqDiffNormalThresh(const double &v)  {sq_diff_normal_thresh_= v;}
  void SetNodataSqDiffDepth(const double &v)  {nodata_sq_diff_depth_= v;}
  void SetNodataSqDiffNormal(const double &v)  {nodata_sq_diff_normal_= v;}
  void SetThGoodDepthRatio (const double &v)  {th_good_depth_ratio_ = v;}
  void SetThBadDepthRatio  (const double &v)  {th_bad_depth_ratio_  = v;}
  void SetThGoodNormalRatio(const double &v)  {th_good_normal_ratio_= v;}
  void SetThBadNormalRatio (const double &v)  {th_bad_normal_ratio_ = v;}

private:
  Imager::DepthScene scene_;
  std::vector<TPose>  poses_;  // [index]= pose; current poses
  std::vector<TDegRotation>  rotations_;  // [index]= rotation; having current Euler rotation since objects in scene_ does not have it.
  std::vector<Imager::TROI3D>  local_roi_;  // [index]= local ROI
  std::map<const Imager::SolidObject*,int>  solid_to_index_;  // [*solid]= index; to know the object index of given solid pointer (of an intersection)
  Imager::TCameraInfo camera_;

  double sq_diff_depth_thresh_;  // If depth error is less than this, consider matched
  double sq_diff_normal_thresh_;  // If normal error is less than this, consider matched

  double nodata_sq_diff_depth_;  // Depth error given to a no-data point
  double nodata_sq_diff_normal_;  // Normal error given to a no-data point

  // Thresholds of ratio for CompareEvalDescriptions2
  double th_good_depth_ratio_;
  double th_bad_depth_ratio_;
  double th_good_normal_ratio_;
  double th_bad_normal_ratio_;

  // Set rotation ex,ey,ez(in radian) of the object
  // WARNING: this function does not update poses_; use: SetPose, SetQ, SetXYZ
  inline void set_rotation(int index, const double &ex, const double &ey, const double &ez);

};  // TRayTracePoseEstimator
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

inline int TRayTracePoseEstimator::EvaluateDescription1(
    const TEvalDescription &desc, const double &w_depth, const double &w_normal)
{
  desc.WError= desc.SqDiffDepth*w_depth + desc.SqDiffNormal*w_normal;
}
//-------------------------------------------------------------------------------------------

/* Compare two evaluation descriptions (ver 1),
    return
      -1: desc1 < desc2 (desc1 is worse),
      0 : desc1 == desc2 (equal),
      +1: desc1 > desc2 (desc1 is better).
    w_depth: weight of depth-img difference.
    w_normal: weight of normal-img difference. */
inline int TRayTracePoseEstimator::CompareEvalDescriptions1(
    const TEvalDescription &desc1, const TEvalDescription &desc2,
    const double &w_depth, const double &w_normal)
{
  const int D1_IS_BETTER(+1), D2_IS_BETTER(-1), EQUAL(0);
  if(desc1.NumOutOfCam>0 && desc2.NumOutOfCam>0)  return EQUAL;
  if(desc1.NumOutOfCam>0)  return D2_IS_BETTER;  // We do not use if some points are out of the range
  if(desc2.NumOutOfCam>0)  return D1_IS_BETTER;  // We do not use if some points are out of the range
  EvaluateDescription1(desc1, w_depth, w_normal);
  EvaluateDescription1(desc2, w_depth, w_normal);
  if(desc1.WError==desc2.WError)  return EQUAL;
  return (desc1.WError < desc2.WError) ? D1_IS_BETTER : D2_IS_BETTER;
}
//-------------------------------------------------------------------------------------------


inline int TRayTracePoseEstimator::EvaluateDescription2(
    const TEvalDescription &desc, const double &w_depth, const double &w_normal)
{
  desc.Quality= 0;

  double n_total= desc.NumInsideCam + desc.NumOutOfCam;
  desc.RMatchedDepth= (double)desc.NumMatchedDepth / n_total;
  desc.RUnmatchedDepth= (double)(desc.NumInsideCam-desc.NumMatchedDepth-desc.NumNoDepth) / n_total;
  desc.RNoDepth= (double)desc.NumNoDepth / n_total;
  desc.RMatchedNormal= (double)desc.NumMatchedNormal / n_total;
  desc.RUnmatchedNormal= (double)(desc.NumInsideCam-desc.NumMatchedNormal-desc.NumNoNormal) / n_total;
  desc.RNoNormal= (double)desc.NumNoNormal / n_total;

  if(desc.RMatchedDepth > th_good_depth_ratio_)  desc.Quality+= 2;
  else if(desc.RUnmatchedDepth > th_bad_depth_ratio_)  desc.Quality+= 0;
  else  desc.Quality+= 1;
  if(desc.RMatchedNormal > th_good_normal_ratio_)  desc.Quality+= 2;
  else if(desc.RUnmatchedNormal > th_bad_normal_ratio_)  desc.Quality+= 0;
  else  desc.Quality+= 1;

  desc.WError= desc.SqDiffDepth*w_depth + desc.SqDiffNormal*w_normal;
}
//-------------------------------------------------------------------------------------------

/* Compare two evaluation descriptions (ver 2),
    return
      -1: desc1 < desc2 (desc1 is worse),
      0 : desc1 == desc2 (equal),
      +1: desc1 > desc2 (desc1 is better).
    w_depth: weight of depth-img difference.
    w_normal: weight of normal-img difference. */
inline int TRayTracePoseEstimator::CompareEvalDescriptions2(
    const TEvalDescription &desc1, const TEvalDescription &desc2,
    const double &w_depth, const double &w_normal)
{
  const int D1_IS_BETTER(+1), D2_IS_BETTER(-1), EQUAL(0);

  EvaluateDescription2(desc1, w_depth, w_normal);
  EvaluateDescription2(desc2, w_depth, w_normal);

  if(desc1.Quality > desc2.Quality)  return D1_IS_BETTER;
  else if(desc1.Quality < desc2.Quality)  return D2_IS_BETTER;

  // else: desc1.Quality == desc2.Quality
  if(desc1.WError==desc2.WError)  return EQUAL;
  return (desc1.WError < desc2.WError) ? D1_IS_BETTER : D2_IS_BETTER;

  // if(desc1.NumOutOfCam>0 && desc2.NumOutOfCam>0)  return EQUAL;
  // if(desc1.NumOutOfCam>0)  return D2_IS_BETTER;  // We do not use if some points are out of the range
  // if(desc2.NumOutOfCam>0)  return D1_IS_BETTER;  // We do not use if some points are out of the range
}
//-------------------------------------------------------------------------------------------

/* Check an evaluation description satisfies a threshold (ver 2),
    return true if satisfied.
    w_depth: weight of depth-img difference.
    w_normal: weight of normal-img difference. */
inline bool TRayTracePoseEstimator::CheckEvalDescriptions2(
    const TEvalDescription &desc, const TEvalDescription &threshold,
    const double &w_depth, const double &w_normal)
{
  EvaluateDescription2(desc, w_depth, w_normal);
  // first, check by Quality
  if(desc.Quality < threshold.Quality)  return false;
  // then, compare SqDiffDepth and SqDiffNormal
  return desc.SqDiffDepth <= threshold.SqDiffDepth
          && desc.SqDiffNormal <= threshold.SqDiffNormal;
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // pose_estimator_h
//-------------------------------------------------------------------------------------------
