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

// Set pose of object; pose={x,y,z, qx,qy,qz,qw}
Imager::SolidObject* SolidObject_SetPose(Imager::SolidObject *obj, const double pose[7])
{
  const double *q(pose+3);
  double ey(0.0),ez(0.0),ex(0.0);
  QuaternionToYZXEuler(q[0],q[1],q[2],q[3], ey,ez,ex);
  obj->RotateX(Rad2Deg(ex));
  obj->RotateZ(Rad2Deg(ez));
  obj->RotateY(Rad2Deg(ey));
  obj->Move(pose[0],pose[1],pose[2]);
  return obj;
}
//-------------------------------------------------------------------------------------------

/* Create a primitive ray tracing model of given model.
    elements : internal solid objects are stored including the returned one. */
Imager::SolidObject* CreateRayTracePrimitive(
    const TRayTraceModel::TPrimitive &prim,
    std::list<const Imager::SolidObject*>  &elements)
{
  using namespace Imager;
  SolidObject *obj(NULL), *sub_obj1(NULL), *sub_obj2(NULL), *sub_obj3(NULL), *sub_obj4(NULL);
  switch(prim.Kind)
  {
  case rtpkSphere   :
    obj= new Sphere(Vector(0.0, 0.0, 0.0), prim.Param[0]);
    break;
  case rtpkSpheroid :
    obj= new Spheroid(prim.Param[0], prim.Param[1], prim.Param[2]);
    break;
  case rtpkCuboid   :
    obj= new Cuboid(prim.Param[0], prim.Param[1], prim.Param[2]);
    break;
  case rtpkCylinder :
    obj= new Cylinder(prim.Param[0], prim.Param[1]);
    break;
  case rtpkHalfCylinder :
    sub_obj1= new Cylinder(prim.Param[0], prim.Param[1]);
    sub_obj2= new Cuboid(prim.Param[0]*1.05, prim.Param[0]*0.52, prim.Param[1]*1.05);
    sub_obj2->Move(0.0, -prim.Param[0]*0.52, 0.0);
    obj= new SetDifference(Vector(0.0, 0.0, 0.0), sub_obj1, sub_obj2);
    break;
  case rtpkTube     :
    /*cylinder_out:*/sub_obj1= new Cylinder(prim.Param[0], prim.Param[2]);
    /*cylinder_in: */sub_obj2= new Cylinder(prim.Param[1], prim.Param[2]*1.05);  // 1.05 means height expansion needed for better modeling
    sub_obj2->Move(prim.Param[3],prim.Param[4],0.0);
    obj= new SetDifference(Vector(0.0, 0.0, 0.0),
        /*cylinder_out=*/sub_obj1,
        /*cylinder_in= */sub_obj2);
    break;
  case rtpkHalfTube     :
    /*cylinder_out:*/sub_obj1= new Cylinder(prim.Param[0], prim.Param[2]);
    /*cylinder_in: */sub_obj2= new Cylinder(prim.Param[1], prim.Param[2]*1.05);  // 1.05 means height expansion needed for better modeling
    sub_obj2->Move(prim.Param[3],prim.Param[4],0.0);
    /*tube:*/sub_obj3= new SetDifference(Vector(0.0, 0.0, 0.0),
        /*cylinder_out=*/sub_obj1,
        /*cylinder_in= */sub_obj2);
    sub_obj4= new Cuboid(prim.Param[0]*1.05, prim.Param[0]*0.52, prim.Param[2]*1.05);
    sub_obj4->Move(0.0, -prim.Param[0]*0.52, 0.0);
    obj= new SetDifference(Vector(0.0, 0.0, 0.0), sub_obj3, sub_obj4);
    break;
  case rtpkRectTube   :  // Param={half_len_x_out,half_len_y_out,half_len_z, half_len_x_in,half_len_y_in,dx,dy}, dx,dy: displacement of hole position
    /*cuboid_out:*/sub_obj1= new Cuboid(prim.Param[0], prim.Param[1], prim.Param[2]);
    /*cuboid_in: */sub_obj2= new Cuboid(prim.Param[3], prim.Param[4], prim.Param[2]*1.05);  // 1.05 means z-expansion needed for better modeling
    sub_obj2->Move(prim.Param[5],prim.Param[6],0.0);
    obj= new SetDifference(Vector(0.0, 0.0, 0.0),
        /*cuboid_out=*/sub_obj1,
        /*cuboid_in= */sub_obj2);
    break;
  case rtpkTorus    :
    // TODO: Implement from here to the end of this function
    break;
  case rtpkPolyhedra:
    break;
  default :
    std::cerr<<"Unknown primitive kind: "<<(int)prim.Kind<<std::endl;
    return NULL;
  }
  if(obj==NULL)
  {
    std::cerr<<"Not implemented primitive kind: "<<(int)prim.Kind<<std::endl;
    return NULL;
  }
  else
  {
    elements.push_back(obj);
    if(sub_obj1)  elements.push_back(sub_obj1);
    if(sub_obj2)  elements.push_back(sub_obj2);
    if(sub_obj3)  elements.push_back(sub_obj3);
    if(sub_obj4)  elements.push_back(sub_obj4);
    return obj;
  }
}
//-------------------------------------------------------------------------------------------

/* Create a ray tracing model of given model.
    elements : internal solid objects are stored including the returned one. */
Imager::SolidObject* CreateRayTraceModel(
    const TRayTraceModel &model,
    std::list<const Imager::SolidObject*>  &elements)
{
  using namespace Imager;
  #if 1
  if(model.Primitives.size()==0)
  {
    std::cerr<<"No primitives are given."<<std::endl;
    return NULL;
  }

  if(model.Primitives.size()==1)
  {
    SolidObject *object_1= CreateRayTracePrimitive(model.Primitives[0], elements);
    SolidObject_SetPose(object_1, model.Primitives[0].Pose);
    SolidObject *res= new SolidObject_Wrapper(Vector(0.0, 0.0, 0.0), object_1);
    elements.push_back(res);
    return res;
  }

  // model.Primitives.size() >= 2
  SolidObject *unified= CreateRayTracePrimitive(model.Primitives[0], elements);
  SolidObject_SetPose(unified, model.Primitives[0].Pose);
  for(int i(1),i_end(model.Primitives.size()); i<i_end; ++i)
  {
    SolidObject *obj_next= CreateRayTracePrimitive(model.Primitives[i], elements);
    SolidObject_SetPose(obj_next, model.Primitives[i].Pose);
    unified= new SetUnion(Vector(0.0, 0.0, 0.0), unified, obj_next);
    elements.push_back(unified);
  }
  return unified;
  #endif

  // For TEST:
  #if 0
  // Coke can test:
  Cylinder* cylinder_in= new Cylinder(0.033, 0.12);
  SolidObject_Wrapper *cylinder= new SolidObject_Wrapper(Vector(0.0, 0.0, -0.06), cylinder_in);

  // Mag cup test:
  // Cylinder* cylinder_out = new Cylinder(0.04, 0.10);
  // Cylinder* cylinder_in = new Cylinder(0.037, 0.10);
  // cylinder_in->Move(0.0, 0.0, 0.01);
  // SetIntersection *cylinder= new SetDifference(Vector(0.0, 0.0, -0.05), cylinder_out, cylinder_in);

  return cylinder;
  #endif
}
//-------------------------------------------------------------------------------------------


// Region of interest in 3D (primitive).
Imager::TROI3D GetROI3DPrimitive(const TRayTraceModel::TPrimitive &prim)
{
  using namespace Imager;
  TROI3D roi;
  switch(prim.Kind)
  {
  case rtpkSphere   :
    roi.SetMin(-prim.Param[0],-prim.Param[0],-prim.Param[0]);
    roi.SetMax(+prim.Param[0],+prim.Param[0],+prim.Param[0]);
    break;
  case rtpkSpheroid :
    roi.SetMin(-prim.Param[0],-prim.Param[1],-prim.Param[2]);
    roi.SetMax(+prim.Param[0],+prim.Param[1],+prim.Param[2]);
    break;
  case rtpkCuboid   :
    roi.SetMin(-prim.Param[0],-prim.Param[1],-prim.Param[2]);
    roi.SetMax(+prim.Param[0],+prim.Param[1],+prim.Param[2]);
    break;
  case rtpkCylinder :
    roi.SetMin(-prim.Param[0],-prim.Param[0],-prim.Param[1]);
    roi.SetMax(+prim.Param[0],+prim.Param[0],+prim.Param[1]);
    break;
  case rtpkHalfCylinder :
    roi.SetMin(-prim.Param[0],           0.0,-prim.Param[1]);
    roi.SetMax(+prim.Param[0],+prim.Param[0],+prim.Param[1]);
    break;
  case rtpkTube     :
    roi.SetMin(-prim.Param[0],-prim.Param[0],-prim.Param[2]);
    roi.SetMax(+prim.Param[0],+prim.Param[0],+prim.Param[2]);
    break;
  case rtpkHalfTube     :
    roi.SetMin(-prim.Param[0],           0.0,-prim.Param[2]);
    roi.SetMax(+prim.Param[0],+prim.Param[0],+prim.Param[2]);
    break;
  case rtpkRectTube  :
    roi.SetMin(-prim.Param[0],-prim.Param[1],-prim.Param[2]);
    roi.SetMax(+prim.Param[0],+prim.Param[1],+prim.Param[2]);
    break;
  case rtpkTorus    :
    // TODO: Implement from here to the end of this function
    std::cerr<<"Not implemented primitive kind: "<<(int)prim.Kind<<std::endl;
    break;
  case rtpkPolyhedra:
    std::cerr<<"Not implemented primitive kind: "<<(int)prim.Kind<<std::endl;
    break;
  default :
    std::cerr<<"Unknown primitive kind: "<<(int)prim.Kind<<std::endl;
    return TROI3D();
  }
  return roi;
}
//-------------------------------------------------------------------------------------------

// Region of interest in 3D (automatically computed).
Imager::TROI3D GetROI3D(const TRayTraceModel &model)
{
  if(model.Primitives.size()==0)  return Imager::TROI3D();

  Imager::TROI3D roi;
  bool init(true);
  for(int i(0),i_end(model.Primitives.size()); i<i_end; ++i)
  {
    Imager::TROI3D roi2= GetROI3DPrimitive(model.Primitives[i]);
    roi2= roi2.Transform(model.Primitives[i].Pose);
    if(init)  {roi= roi2;  init= false;}
    else      {roi.Add(roi2);}
  }
  return roi;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TRayTracePoseEstimator
//===========================================================================================

// Add an object to the scene and return its index
int TRayTracePoseEstimator::AddObject(const TRayTraceModel &model, const double pose[7])
{
  std::list<const Imager::SolidObject*> elements;
  Imager::SolidObject *obj= CreateRayTraceModel(model, elements);
  if(obj==NULL)  return -1;
  scene_.AddSolidObject(obj);
  int index= scene_.NumSolidObjects()-1;
  for(std::list<const Imager::SolidObject*>::const_iterator itr(elements.begin()),itr_end(elements.end()); itr!=itr_end; ++itr)
    solid_to_index_[*itr]= index;
  poses_.push_back(TPose());
  local_roi_.push_back(GetROI3D(model));
  rotations_.push_back(TDegRotation());

  SetPose(index, pose);

  return index;
}
//-------------------------------------------------------------------------------------------

Imager::TROI2D<int> TRayTracePoseEstimator::GetImageROI() const
{
  Imager::TROI2D<int> img_roi;
  bool init(true);
  for(int i(0),i_end(poses_.size()); i<i_end; ++i)
  {
    Imager::TROI2D<int> img_roi2;
    local_roi_[i].ToImageROI(camera_, poses_[i].X, img_roi2);
    if(init)  {img_roi= img_roi2;  init= false;}
    else      {img_roi.Add(img_roi2);}
  }
  return img_roi;
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::Render(
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp)
{
  std::list<Imager::Intersection> intersections;
  scene_.Render4(camera_, GetImageROI(), intersections, step_xp, step_yp);

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
  index : if -1, all rendered intersections are considered,
      if >=0, only intersections whose solid is mapped to index are considered.
  depth_img, normal_img : depth and normal images.
  sqdiff_depth, sqdiff_normal : square errors of valid depth and normal points.
  n_invalid_depth : number of invalid depth pixels in depth_img on the model image.
  n_invalid_normal : number of invalid normal pixels in normal_img on the model image.
  n_invalid_range : number of invalid pixels that is out of range of the camera.
  step_xp, step_yp : step size to compute the model image.  Greater is faster but bigger error.
*/
void TRayTracePoseEstimator::GetDistance(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    double &sqdiff_depth, double &sqdiff_normal,
    int &n_invalid_depth, int &n_invalid_normal, int &n_invalid_range,
    int step_xp, int step_yp)
{
  std::list<Imager::Intersection> intersections;
  scene_.Render4(camera_, GetImageROI(), intersections, step_xp, step_yp);

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
    //*DEBUG*/std::cerr<<index<<"\t"<<solid_to_index_[itr->solid]<<std::endl;
    if(index>=0 && solid_to_index_[itr->solid]!=index)  continue;
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
  //*DEBUG*/std::cerr<<sqdiff_depth<<"\t"<<sqdiff_normal<<"\t"<<n_valid_depth<<", "<<n_valid_normal<<std::endl;
  //*DEBUG*/std::cerr<<sqdiff_depth<<"\t"<<sqdiff_normal<<"\t"<<n_invalid_depth<<", "<<n_invalid_normal<<", "<<n_invalid_range<<std::endl;
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeXY(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double &range_x, const double &range_y, const double &n_div,
    const double &w_depth, const double &w_normal,
    double xy_opt[2], double eval_opt[2])
{
  double x0(poses_[index].X[0]), y0(poses_[index].X[1]), z0(poses_[index].X[2]);
  double rx(0.5*range_x), ry(0.5*range_y);
  double x_best(x0), y_best(y0), eval_best(100.0);
  double sqdiff_depth_best(100.0), sqdiff_normal_best(100.0);
  for(double x(x0-rx); x<x0+rx; x+= 2.0*rx/n_div)
  {
    for(double y(y0-ry); y<y0+ry; y+= 2.0*ry/n_div)
    {
      SetXYZ(index, x,y,z0);
      double sqdiff_depth(0.0), sqdiff_normal(0.0), eval(0.0);
      int n_invalid_depth(0), n_invalid_normal(0), n_invalid_range(0);
      GetDistance(index, depth_img, normal_img,
          sqdiff_depth, sqdiff_normal,
          n_invalid_depth, n_invalid_normal, n_invalid_range,
          step_xp, step_yp);
      if(n_invalid_range>0)  continue;  // We do not use if some points are out of the range
      eval= sqdiff_depth*w_depth + sqdiff_normal*w_normal;
      if(eval<eval_best)
      {
        x_best= x;
        y_best= y;
        eval_best= eval;
        sqdiff_depth_best= sqdiff_depth;
        sqdiff_normal_best= sqdiff_normal;
        // std::cerr<<eval_best<<"\t"<<x_best<<"\t"<<y_best<<std::endl;
      }
    }
  }
  SetXYZ(index, x_best,y_best,z0);
  if(xy_opt)
  {
    xy_opt[0]= x_best;
    xy_opt[1]= y_best;
  }
  if(eval_opt)
  {
    eval_opt[0]= sqdiff_depth_best;
    eval_opt[1]= sqdiff_normal_best;
  }
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeZ(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double &range_z, const double &n_div,
    const double &w_depth, const double &w_normal,
    double z_opt[1], double eval_opt[2])
{
  double x0(poses_[index].X[0]), y0(poses_[index].X[1]), z0(poses_[index].X[2]);
  double rz(0.5*range_z);
  double z_best(z0), eval_best(100.0);
  double sqdiff_depth_best(100.0), sqdiff_normal_best(100.0);
  for(double z(z0-rz); z<z0+rz; z+= 2.0*rz/n_div)
  {
    SetXYZ(index, x0,y0,z);
    double sqdiff_depth(0.0), sqdiff_normal(0.0), eval(0.0);
    int n_invalid_depth(0), n_invalid_normal(0), n_invalid_range(0);
    GetDistance(index, depth_img, normal_img,
        sqdiff_depth, sqdiff_normal,
        n_invalid_depth, n_invalid_normal, n_invalid_range,
        step_xp, step_yp);
    if(n_invalid_range>0)  continue;  // We do not use if some points are out of the range
    eval= sqdiff_depth*w_depth + sqdiff_normal*w_normal;
    if(eval<eval_best)
    {
      z_best= z;
      eval_best= eval;
      sqdiff_depth_best= sqdiff_depth;
      sqdiff_normal_best= sqdiff_normal;
      // std::cerr<<eval_best<<"\t"<<z_best<<std::endl;
    }
  }
  SetXYZ(index, x0,y0,z_best);
  if(z_opt)  z_opt[0]= z_best;
  if(eval_opt)
  {
    eval_opt[0]= sqdiff_depth_best;
    eval_opt[1]= sqdiff_normal_best;
  }
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeXYZ(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    double position_opt[3], double eval_opt[2])
{
  double *xy_opt(NULL), *z_opt(NULL);
  if(position_opt)
  {
    xy_opt= position_opt+0;
    z_opt= position_opt+2;
  }
  OptimizeXY(index, depth_img, normal_img, step_xp, step_yp,
      /*range_x=*/0.20, /*range_y=*/0.20, /*n_div=*/40.0,
      /*w_depth=*/5.0, /*w_normal=*/1.0,
      xy_opt, eval_opt);
  OptimizeZ(index, depth_img, normal_img, step_xp, step_yp,
      /*range_z=*/0.20, /*n_div=*/40.0,
      /*w_depth=*/5.0, /*w_normal=*/0.5,
      z_opt, eval_opt);
  OptimizeXY(index, depth_img, normal_img, step_xp, step_yp,
      /*range_x=*/0.05, /*range_y=*/0.05, /*n_div=*/20.0,
      /*w_depth=*/5.0, /*w_normal=*/1.0,
      xy_opt, eval_opt);
  OptimizeZ(index, depth_img, normal_img, step_xp, step_yp,
      /*range_z=*/0.05, /*n_div=*/20.0,
      /*w_depth=*/5.0, /*w_normal=*/0.5,
      z_opt, eval_opt);
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeLin2D(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double axis_1[3], const double axis_2[3],
    const double &range_1, const double &range_2, const double &n_div,
    const double &w_depth, const double &w_normal,
    double opt_12[2], double position_opt[3], double eval_opt[2])
{
  Eigen::Vector3d xyz0(poses_[index].X), a1(axis_1), a2(axis_2), xyz(0.0,0.0,0.0);
  double r1(0.5*range_1), r2(0.5*range_2);
  double t1_best(0.0), t2_best(0.0), eval_best(100.0);
  double sqdiff_depth_best(100.0), sqdiff_normal_best(100.0);
  for(double t1(-r1); t1<+r1; t1+= 2.0*r1/n_div)
  {
    for(double t2(-r2); t2<+r2; t2+= 2.0*r2/n_div)
    {
      xyz= xyz0 + t1*a1 + t2*a2;
      SetXYZ(index, xyz[0],xyz[1],xyz[2]);
      double sqdiff_depth(0.0), sqdiff_normal(0.0), eval(0.0);
      int n_invalid_depth(0), n_invalid_normal(0), n_invalid_range(0);
      GetDistance(index, depth_img, normal_img,
          sqdiff_depth, sqdiff_normal,
          n_invalid_depth, n_invalid_normal, n_invalid_range,
          step_xp, step_yp);
      if(n_invalid_range>0)  continue;  // We do not use if some points are out of the range
      eval= sqdiff_depth*w_depth + sqdiff_normal*w_normal;
      if(eval<eval_best)
      {
        t1_best= t1;
        t2_best= t2;
        eval_best= eval;
        sqdiff_depth_best= sqdiff_depth;
        sqdiff_normal_best= sqdiff_normal;
        // std::cerr<<eval_best<<"\t"<<t1_best<<"\t"<<t2_best<<std::endl;
      }
    }
  }
  xyz= xyz0 + t1_best*a1 + t2_best*a2;
  SetXYZ(index, xyz[0],xyz[1],xyz[2]);
  if(opt_12)
  {
    opt_12[0]= t1_best;
    opt_12[1]= t2_best;
  }
  if(position_opt)
  {
    position_opt[0]= xyz[0];
    position_opt[1]= xyz[1];
    position_opt[2]= xyz[2];
  }
  if(eval_opt)
  {
    eval_opt[0]= sqdiff_depth_best;
    eval_opt[1]= sqdiff_normal_best;
  }
}
//-------------------------------------------------------------------------------------------

// Handle a key event where c is the output of cv::waitKey
bool TRayTracePoseEstimator::HandleKeyEvent(int index, int c)
{
  if((65360<=c && c<=65364) || c==65367
    || (130896<=c && c<=130900) || c==130903)
  {
    double astep(0.1), cstep(0.01);
    const double axis[3][3]= { {1.0,0.0,0.0}, {0.0,1.0,0.0}, {0.0,0.0,1.0} };
    const double *x(poses_[index].X);
    double q[4], xyz[3]={x[0],x[1],x[2]};
    switch(c)
    {
    case 65361/*LEFT*/:   RotateAngleAxis(-astep, axis[2], x+3, q); SetQ(index,q);   break;
    case 65363/*RIGHT*/:  RotateAngleAxis(+astep, axis[2], x+3, q); SetQ(index,q);   break;
    case 65362/*UP*/:     RotateAngleAxis(-astep, axis[0], x+3, q); SetQ(index,q);   break;
    case 65364/*DOWN*/:   RotateAngleAxis(+astep, axis[0], x+3, q); SetQ(index,q);   break;
    case 65360/*HOME*/:   RotateAngleAxis(-astep, axis[1], x+3, q); SetQ(index,q);   break;
    case 65367/*END*/:    RotateAngleAxis(+astep, axis[1], x+3, q); SetQ(index,q);   break;
    case 130897/*SHIFT+LEFT*/:   xyz[0]-=cstep; SetXYZ(index,xyz);   break;
    case 130899/*SHIFT+RIGHT*/:  xyz[0]+=cstep; SetXYZ(index,xyz);   break;
    case 130898/*SHIFT+UP*/:     xyz[1]-=cstep; SetXYZ(index,xyz);   break;
    case 130900/*SHIFT+DOWN*/:   xyz[1]+=cstep; SetXYZ(index,xyz);   break;
    case 130896/*SHIFT+HOME*/:   xyz[2]-=cstep; SetXYZ(index,xyz);   break;
    case 130903/*SHIFT+END*/:    xyz[2]+=cstep; SetXYZ(index,xyz);   break;
    }
    return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

