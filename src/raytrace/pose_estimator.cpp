//-------------------------------------------------------------------------------------------
/*! \file    pose_estimator.cpp
    \brief   Pose estimation using ray tracing
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/raytrace/pose_estimator.h"
#include "pr2_lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include <iostream>
#include <list>
//-------------------------------------------------------------------------------------------
namespace trick
{

//===========================================================================================
// Utility functions
//===========================================================================================

// template<typename T>
// inline T Sq(const T &x)  {return x*x;}

Imager::SolidObject* CreateRayTracePrimitive(const TRayTraceModel::TPrimitive &prim)
{
  using namespace Imager;
  switch(prim.Kind)
  {
  case rtpkSphere   :
    return new Sphere(Vector(prim.Pose[0], prim.Pose[1], prim.Pose[2]), prim.Param[0]);
  case rtpkSpheroid :
    // FIXME: Implement from here to the end of this function
    // Spheroid* spheroid = new Spheroid(4.0, 2.0, 1.0);
    break;
  case rtpkCuboid   :
    break;
  case rtpkCylinder :
    break;
  case rtpkTube     :
    break;
  case rtpkTorus    :
    break;
  case rtpkPolyhedra:
    break;
  default :
    return NULL;
  }
  // NOTE: translation and rotation are common, other than sphere.
  // spheroid->Move(0.0, 0.0, -50.0);
  // spheroid->RotateX(-12.0);
  // spheroid->RotateY(-60.0);
  // TODO: We need to add a method to Imager::SolidObject_Reorientable that resets the rotation.
  return NULL;
}
//-------------------------------------------------------------------------------------------

Imager::SolidObject* CreateRayTraceModel(const TRayTraceModel &model)
{
  using namespace Imager;
  // FIXME: implement from here
  // if(model.Primitives.size()==0)  return NULL;
  // const TRayTraceModel::TPrimitive &prim(model.Primitives[0]);
  // CreateRayTracePrimitive...

  // FIXME: For TEST:

  // Coke can test:
  Cylinder* cylinder_in= new Cylinder(0.033, 0.12);
  SolidObject_Wrapper *cylinder= new SolidObject_Wrapper(Vector(0.0, 0.0, -0.06), cylinder_in);

  // Mag cup test:
  // Cylinder* cylinder_out = new Cylinder(0.04, 0.10);
  // Cylinder* cylinder_in = new Cylinder(0.037, 0.10);
  // cylinder_in->Move(0.0, 0.0, 0.01);
  // SetIntersection *cylinder= new SetDifference(Vector(0.0, 0.0, -0.05), cylinder_out, cylinder_in);

  return cylinder;
}
//-------------------------------------------------------------------------------------------

// Radius of the bounding sphere (automatically computed).
double GetBoundingRadius(const TRayTraceModel &model)
{
  if(model.BoundingRadius>0.0)  return model.BoundingRadius;
  // FIXME: implement this function

  // FIXME: For TEST:
  return std::sqrt(0.1*0.1+0.04*0.04)*1.1/*margin*/;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TRayTracePoseEstimator
//===========================================================================================

// Add an object to the scene and return its index
int TRayTracePoseEstimator::AddObject(const TRayTraceModel &model, const double pose[7])
{
  Imager::SolidObject *obj= CreateRayTraceModel(model);
  scene_.AddSolidObject(obj);
  int index= scene_.NumSolidObjects()-1;
  rotations_.push_back(TDegRotation());
  double rad= GetBoundingRadius(model);

  SetPose(index, pose);

  // Create region of interest
  // FIXME:TODO:We need to consider every object
  roi_.Cx= obj->Center().x;
  roi_.Cy= obj->Center().y;
  roi_.Cz= obj->Center().z;
  roi_.Radius= rad;

  return index;
}
//-------------------------------------------------------------------------------------------

// Set x,y,z of the object
void TRayTracePoseEstimator::SetXYZ(int index, const double xyz[3])
{
  Imager::SolidObject *obj= scene_.RefSolidObject(index);
  obj->Move(xyz[0],xyz[1],xyz[2]);
}
//-------------------------------------------------------------------------------------------

// Set rotation ex,ey,ez(in radian) of the object
void TRayTracePoseEstimator::SetRotation(int index, const double &ex, const double &ey, const double &ez)
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
void TRayTracePoseEstimator::SetQ(int index, const double q[4])
{
  double ey(0.0),ez(0.0),ex(0.0);
  QuaternionToYZXEuler(q[0],q[1],q[2],q[3], ey,ez,ex);
  SetRotation(index, ex,ey,ez);
}
//-------------------------------------------------------------------------------------------

// Set pose=x,y,z,quaternion of the object
void TRayTracePoseEstimator::SetPose(int index, const double pose[7])
{
  SetXYZ(index, pose);
  SetQ(index, pose+3);
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::UpdateROI()
{
  // FIXME:TODO: consider every index
  Imager::SolidObject *obj= scene_.RefSolidObject(0);
  roi_.Cx= obj->Center().x;
  roi_.Cy= obj->Center().y;
  roi_.Cz= obj->Center().z;
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::Render(
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp)
{
  UpdateROI();
  std::list<Imager::Intersection> intersections;
  scene_.Render4(camera_, roi_, intersections, step_xp, step_yp);

  for(std::list<Imager::Intersection>::const_iterator itr(intersections.begin()),last(intersections.end());
      itr!=last; ++itr)
  {
    int px(0), py(0);
    camera_.Project(itr->point.x, itr->point.y, itr->point.z, px, py);
    if(camera_.IsInvalid(px,py))  continue;
    cv::Vec3f normal(GetVisualNormal(itr->surfaceNormal));

    depth_img.at<float>(py,px)= itr->point.z;
    normal_img.at<cv::Vec3f>(py,px)= normal;
  }
}
//-------------------------------------------------------------------------------------------

/*Get a distance between the ray traced model image and actual images.
  depth_img, normal_img : depth and normal images.
  sqdiff_depth, sqdiff_normal : square errors of valid depth and normal points.
  n_invalid_depth : number of invalid depth pixels in depth_img on the model image.
  n_invalid_normal : number of invalid normal pixels in normal_img on the model image.
  n_invalid_range : number of invalid pixels that is out of range of the camera.
  step_xp, step_yp : step size to compute the model image.  Greater is faster but bigger error.
*/
void TRayTracePoseEstimator::GetDistance(
    cv::Mat &depth_img, cv::Mat &normal_img,
    double &sqdiff_depth, double &sqdiff_normal,
    int &n_invalid_depth, int &n_invalid_normal, int &n_invalid_range,
    int step_xp, int step_yp)
{
  UpdateROI();
  std::list<Imager::Intersection> intersections;
  scene_.Render4(camera_, roi_, intersections, step_xp, step_yp);

  cv::Vec3f invalid_normal(0.0,0.0,0.0);
  sqdiff_depth= 0.0;
  sqdiff_normal= 0.0;
  int n_valid_depth(0), n_valid_normal(0);
  n_invalid_depth= 0;
  n_invalid_normal= 0;
  n_invalid_range= 0;
  for(std::list<Imager::Intersection>::const_iterator itr(intersections.begin()),last(intersections.end());
      itr!=last; ++itr)
  {
    int px(0), py(0);
    camera_.Project(itr->point.x, itr->point.y, itr->point.z, px, py);
    if(camera_.IsInvalid(px,py))  {++n_invalid_range;  continue;}
    cv::Vec3f normal(GetVisualNormal(itr->surfaceNormal));

    // Update difference:
    if(IsValid(depth_img.at<float>(py,px)))
    {
      sqdiff_depth+= Sq(depth_img.at<float>(py,px) - itr->point.z);
      ++n_valid_depth;
    }
    else
    {
      // sqdiff_depth+= 0.1;  // FIXME: how to treat invalid sensor data?
      ++n_invalid_depth;
    }
    if(normal_img.at<cv::Vec3f>(py,px) != invalid_normal)
    {
      sqdiff_normal+= Sq(cv::norm(normal_img.at<cv::Vec3f>(py,px) - normal));
      ++n_valid_normal;
    }
    else
    {
      // sqdiff_normal+= 0.5;  // FIXME: how to treat invalid sensor data?
      ++n_invalid_normal;
    }
  }
  if(n_valid_depth>0)   sqdiff_depth/= static_cast<double>(n_valid_depth);
  else                  sqdiff_depth= 1.0;
  if(n_valid_normal>0)  sqdiff_normal/= static_cast<double>(n_valid_normal);
  else                  sqdiff_normal= 1.0;
  // if(!no_render)
    // std::cerr<<sqdiff_depth<<"\t"<<sqdiff_normal<<std::endl;
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeXY(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    double xy_opt[2], double eval_opt[1])
{
  Imager::SolidObject *obj= scene_.RefSolidObject(index);
  double x0(obj->Center().x), y0(obj->Center().y), z0(obj->Center().z);
  double rx(0.10), ry(0.10);
  double x_best(x0), y_best(y0), eval_best(100.0);
  for(double x(x0-rx); x<x0+rx; x+= 2.0*rx/20.0)
  {
    for(double y(y0-ry); y<y0+ry; y+= 2.0*ry/80.0)
    {
      obj->Move(x,y,z0);
      double sqdiff_depth(0.0), sqdiff_normal(0.0), eval(0.0);
      int n_invalid_depth(0), n_invalid_normal(0), n_invalid_range(0);
      GetDistance(depth_img, normal_img,
          sqdiff_depth, sqdiff_normal,
          n_invalid_depth, n_invalid_normal, n_invalid_range,
          step_xp, step_yp);
      if(n_invalid_range>0)  continue;  // We do not use if some points are out of the range
      eval= sqdiff_depth*1.0 + sqdiff_normal;
      if(eval<eval_best)
      {
        x_best= x;
        y_best= y;
        eval_best= eval;
        // std::cerr<<eval_best<<"\t"<<x_best<<"\t"<<y_best<<std::endl;
      }
    }
  }
  obj->Move(x_best,y_best,z0);
  if(xy_opt)
  {
    xy_opt[0]= x_best;
    xy_opt[1]= y_best;
  }
  if(eval_opt)  eval_opt[0]= eval_best;
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeZ(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    double z_opt[1], double eval_opt[1])
{
  Imager::SolidObject *obj= scene_.RefSolidObject(0);
  double x0(obj->Center().x), y0(obj->Center().y), z0(obj->Center().z);
  double rz(0.10);
  double z_best(z0), eval_best(100.0);
  for(double z(z0-rz); z<z0+rz; z+= 2.0*rz/80.0)
  {
    obj->Move(x0,y0,z);
    double sqdiff_depth(0.0), sqdiff_normal(0.0), eval(0.0);
    int n_invalid_depth(0), n_invalid_normal(0), n_invalid_range(0);
    GetDistance(depth_img, normal_img,
        sqdiff_depth, sqdiff_normal,
        n_invalid_depth, n_invalid_normal, n_invalid_range,
        step_xp, step_yp);
    if(n_invalid_range>0)  continue;  // We do not use if some points are out of the range
    eval= sqdiff_depth*10.0 + sqdiff_normal*0.1;
    if(eval<eval_best)
    {
      z_best= z;
      eval_best= eval;
      // std::cerr<<eval_best<<"\t"<<z_best<<std::endl;
    }
  }
  obj->Move(x0,y0,z_best);
  if(z_opt)  z_opt[0]= z_best;
  if(eval_opt)  eval_opt[0]= eval_best;
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeXYZ(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    double position_opt[3], double eval_opt[2])
{
  double *xy_opt(NULL), *z_opt(NULL), *xy_eval_opt(NULL), *z_eval_opt(NULL);
  if(position_opt)
  {
    xy_opt= position_opt+0;
    z_opt= position_opt+2;
  }
  if(eval_opt)
  {
    xy_eval_opt= eval_opt+0;
    z_eval_opt= eval_opt+1;
  }
  OptimizeXY(index, depth_img, normal_img, step_xp, step_yp, xy_opt, xy_eval_opt);
  OptimizeZ(index, depth_img, normal_img, step_xp, step_yp, z_opt, z_eval_opt);
}
//-------------------------------------------------------------------------------------------

// Handle a key event where c is the output of cv::waitKey
bool TRayTracePoseEstimator::HandleKeyEvent(int index, int c)
{
  if((65360<=c && c<=65364) || c==65367
    || (130896<=c && c<=130900) || c==130903)
  {
    Imager::SolidObject *obj= scene_.RefSolidObject(index);
    TRadRotation rot;  Deg2Rad3(rotations_[index],rot);
    double astep(0.1), cstep(0.01);
    switch(c)
    {
    case 65361/*LEFT*/:   SetRotation(index, rot.X, rot.Y, rot.Z-astep);   break;
    case 65363/*RIGHT*/:  SetRotation(index, rot.X, rot.Y, rot.Z+astep);   break;
    case 65362/*UP*/:     SetRotation(index, rot.X-astep, rot.Y, rot.Z);   break;
    case 65364/*DOWN*/:   SetRotation(index, rot.X+astep, rot.Y, rot.Z);   break;
    case 65360/*HOME*/:   SetRotation(index, rot.X, rot.Y-astep, rot.Z);   break;
    case 65367/*END*/:    SetRotation(index, rot.X, rot.Y+astep, rot.Z);   break;
    case 130897/*SHIFT+LEFT*/:   obj->Translate(-cstep,0.0,0.0);   break;
    case 130899/*SHIFT+RIGHT*/:  obj->Translate(+cstep,0.0,0.0);   break;
    case 130898/*SHIFT+UP*/:     obj->Translate(0.0,-cstep,0.0);   break;
    case 130900/*SHIFT+DOWN*/:   obj->Translate(0.0,+cstep,0.0);   break;
    case 130896/*SHIFT+HOME*/:   obj->Translate(0.0,0.0,-cstep);   break;
    case 130903/*SHIFT+END*/:    obj->Translate(0.0,0.0,+cstep);   break;
    }
    return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

