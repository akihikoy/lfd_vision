//-------------------------------------------------------------------------------------------
/*! \file    pose_estimator.cpp
    \brief   Pose estimation using ray tracing
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.10, 2015
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/raytrace/pose_estimator.h"
#include "lfd_vision/pcl_util.h"
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

bool TRayTracePoseEstimator::GetImageROI(Imager::TROI2D<int> &img_roi) const
{
  bool init(true);
  for(int i(0),i_end(poses_.size()); i<i_end; ++i)
  {
    Imager::TROI2D<int> img_roi2;
    bool is_valid= local_roi_[i].ToImageROI(camera_, poses_[i].X, img_roi2);
    if(!is_valid)  continue;
    if(init)  {img_roi= img_roi2;  init= false;}
    else      {img_roi.Add(img_roi2);}
  }
  if(init)  return false;
  if(img_roi.Min[0]<-camera_.Width/2)  img_roi.Min[0]= -camera_.Width/2;
  if(img_roi.Min[1]<-camera_.Height/2)  img_roi.Min[1]= -camera_.Height/2;
  if(img_roi.Max[0]>camera_.Width+camera_.Width/2)  img_roi.Max[0]= camera_.Width+camera_.Width/2;
  if(img_roi.Max[1]>camera_.Height+camera_.Height/2)  img_roi.Max[1]= camera_.Height+camera_.Height/2;
//*DEBUG*/std::cerr<<"img_roi: "<<img_roi.Min[0]<<", "<<img_roi.Min[1]<<",  "<<img_roi.Max[0]<<", "<<img_roi.Max[1]<<"; "<<(!init)<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::Render(
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp)
{
  std::list<Imager::Intersection> intersections;
  Imager::TROI2D<int> img_roi;
  if(!GetImageROI(img_roi))  return;
  scene_.Render4(camera_, img_roi, intersections, step_xp, step_yp);

  for(std::list<Imager::Intersection>::const_iterator itr(intersections.begin()),last(intersections.end());
      itr!=last; ++itr)
  {
    if(itr->point.z<=camera_.ZcMin)  continue;
    int px(0), py(0);
    camera_.Project(itr->point.x, itr->point.y, itr->point.z, px, py);
    if(camera_.IsInvalid(px,py))  continue;
    cv::Vec3f normal(GetVisualNormal(itr->surfaceNormal));

    depth_img.at<float>(py,px)= itr->point.z;
//*DEBUG*/std::cerr<<normal_img.data<<" : "<<normal_img.rows<<", "<<normal_img.cols<<" : "<<py<<", "<<px<<std::endl;
    normal_img.at<cv::Vec3f>(py,px)= normal;
  }
}
//-------------------------------------------------------------------------------------------

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
bool TRayTracePoseEstimator::GetEvalDescription(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    TEvalDescription &desc, int step_xp, int step_yp,
    const double &f_depth_normalize)
{
  std::list<Imager::Intersection> intersections;
  Imager::TROI2D<int> img_roi;
  if(!GetImageROI(img_roi))  return false;
  scene_.Render4(camera_, img_roi, intersections, step_xp, step_yp);

  float invalid_depth(0.0f);
  cv::Vec3f invalid_normal(0.0,0.0,0.0);
  double f_depth_n(f_depth_normalize);
  if(f_depth_n<1.0e-6)  f_depth_n= 1.0;
  desc= TEvalDescription();
  double sq_diff_depth(0.0), sq_diff_normal(0.0);
  for(std::list<Imager::Intersection>::const_iterator itr(intersections.begin()),last(intersections.end());
      itr!=last; ++itr)
  {
    //*DEBUG*/std::cerr<<index<<"\t"<<solid_to_index_[itr->solid]<<std::endl;
    if(index>=0 && solid_to_index_[itr->solid]!=index)  continue;
    if(itr->point.z<=camera_.ZcMin)  continue;
    int px(0), py(0);
    camera_.Project(itr->point.x, itr->point.y, itr->point.z, px, py);
    if(camera_.IsInvalid(px,py))  {++desc.NumOutOfCam;  continue;}
    ++desc.NumInsideCam;
    cv::Vec3f normal(GetVisualNormal(itr->surfaceNormal));

    // Update difference:
    if(depth_img.at<float>(py,px) != invalid_depth)
    {
      sq_diff_depth= Sq((depth_img.at<float>(py,px) - itr->point.z)/f_depth_n);
      desc.SqDiffDepth+= sq_diff_depth;
      if(sq_diff_depth<=sq_diff_depth_thresh_)
        ++desc.NumMatchedDepth;
    }
    else
    {
      desc.SqDiffDepth+= nodata_sq_diff_depth_;  // no sensor data
      ++desc.NumNoDepth;
    }
    if(normal_img.at<cv::Vec3f>(py,px) != invalid_normal)
    {
      sq_diff_normal= Sq(cv::norm(normal_img.at<cv::Vec3f>(py,px) - normal));
      /*DEBUG*-/if(IsInvalid(sq_diff_normal))
      {
        std::cerr<<"sq_diff_normal: "<<sq_diff_normal<<std::endl;
        std::cerr<<"  normal_img.at<cv::Vec3f>(py,px): "<<normal_img.at<cv::Vec3f>(py,px)<<std::endl;
        std::cerr<<"  normal: "<<normal<<std::endl;
      }//*/
      desc.SqDiffNormal+= sq_diff_normal;
      if(sq_diff_normal<=sq_diff_normal_thresh_)
        ++desc.NumMatchedNormal;
    }
    else
    {
      desc.SqDiffNormal+= nodata_sq_diff_normal_;  // no sensor data
      ++desc.NumNoNormal;
    }
  }
  int n_valid_depth= desc.NumInsideCam - desc.NumNoDepth;
  int n_valid_normal= desc.NumInsideCam - desc.NumNoNormal;
  if(n_valid_depth>0)   desc.SqDiffDepth/= static_cast<double>(n_valid_depth);
  else                  desc.SqDiffDepth= 1.0;
  if(n_valid_normal>0)  desc.SqDiffNormal/= static_cast<double>(n_valid_normal);
  else                  desc.SqDiffNormal= 1.0;
  //*DEBUG*/std::cerr<<desc.SqDiffDepth<<"\t"<<desc.SqDiffNormal<<"\t"<<n_valid_depth<<", "<<n_valid_normal<<std::endl;
  //*DEBUG*/std::cerr<<desc.SqDiffDepth<<"\t"<<desc.SqDiffNormal<<"\t"<<n_invalid_depth<<", "<<n_invalid_normal<<", "<<n_invalid_range<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

#if 0
DEPRECATED: Use OptimizeLin2D
void TRayTracePoseEstimator::OptimizeXY(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double &range_x, const double &range_y, const double &n_div,
    const double &w_depth, const double &w_normal,
    double xy_opt[2], TEvalDescription *eval_desc_opt)
{
  double x0(poses_[index].X[0]), y0(poses_[index].X[1]), z0(poses_[index].X[2]);
  double rx(0.5*range_x), ry(0.5*range_y);
  // Evaluate current pose:
  double x_best(x0), y_best(y0);
  TEvalDescription eval_desc_best;
  GetEvalDescription(index, depth_img, normal_img,
      eval_desc_best, step_xp, step_yp, z0);
  // Brute force optimization:
  for(double x(x0-rx); x<x0+rx; x+= 2.0*rx/n_div)
  {
    for(double y(y0-ry); y<y0+ry; y+= 2.0*ry/n_div)
    {
      SetXYZ(index, x,y,z0);
      TEvalDescription eval_desc;
      if(!GetEvalDescription(index, depth_img, normal_img,
            eval_desc, step_xp, step_yp, z0))
        continue;
      if(CompareEvalDescriptions(eval_desc,eval_desc_best,w_depth,w_normal)>0)
      {
        x_best= x;
        y_best= y;
        eval_desc_best= eval_desc;
      }
    }
  }
  SetXYZ(index, x_best,y_best,z0);
  if(xy_opt)
  {
    xy_opt[0]= x_best;
    xy_opt[1]= y_best;
  }
  if(eval_desc_opt)
  {
    *eval_desc_opt= eval_desc_best;
  }
}
//-------------------------------------------------------------------------------------------

DEPRECATED: Use OptimizeLin1D
void TRayTracePoseEstimator::OptimizeZ(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double &range_z, const double &n_div,
    const double &w_depth, const double &w_normal,
    double z_opt[1], TEvalDescription *eval_desc_opt)
{
  double x0(poses_[index].X[0]), y0(poses_[index].X[1]), z0(poses_[index].X[2]);
  double rz(0.5*range_z);
  // Evaluate current pose:
  double z_best(z0);
  TEvalDescription eval_desc_best;
  GetEvalDescription(index, depth_img, normal_img,
      eval_desc_best, step_xp, step_yp, z0);
  // Brute force optimization:
  for(double z(z0-rz); z<z0+rz; z+= 2.0*rz/n_div)
  {
    SetXYZ(index, x0,y0,z);
    TEvalDescription eval_desc;
    if(!GetEvalDescription(index, depth_img, normal_img,
          eval_desc, step_xp, step_yp, z0))
      continue;
    if(CompareEvalDescriptions(eval_desc,eval_desc_best,w_depth,w_normal)>0)
    {
      z_best= z;
      eval_desc_best= eval_desc;
    }
  }
  SetXYZ(index, x0,y0,z_best);
  if(z_opt)  z_opt[0]= z_best;
  if(eval_desc_opt)
  {
    *eval_desc_opt= eval_desc_best;
  }
}
//-------------------------------------------------------------------------------------------
#endif

void TRayTracePoseEstimator::OptimizeXYZ(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    double position_opt[3], TEvalDescription *eval_desc_opt)
{
  double axis_x[3]={1.0,0.0,0.0}, axis_y[3]={0.0,1.0,0.0}, axis_z[3]={0.0,0.0,1.0};

  OptimizeLin2D(index, depth_img, normal_img, step_xp, step_yp,
      axis_x, axis_y,
      /*range_1=*/0.20, /*range_2=*/0.20, /*n_div=*/40.0,
      /*w_depth=*/5.0, /*w_normal=*/1.0,
      NULL, position_opt, eval_desc_opt);

  OptimizeLin1D(index, depth_img, normal_img, step_xp, step_yp,
      axis_z,
      /*range_1=*/0.20, /*n_div=*/40.0,
      /*w_depth=*/5.0, /*w_normal=*/0.5,
      NULL, position_opt, eval_desc_opt);

  OptimizeLin2D(index, depth_img, normal_img, step_xp, step_yp,
      axis_x, axis_y,
      /*range_1=*/0.05, /*range_2=*/0.05, /*n_div=*/20.0,
      /*w_depth=*/5.0, /*w_normal=*/1.0,
      NULL, position_opt, eval_desc_opt);

  OptimizeLin1D(index, depth_img, normal_img, step_xp, step_yp,
      axis_z,
      /*range_1=*/0.05, /*n_div=*/20.0,
      /*w_depth=*/5.0, /*w_normal=*/0.5,
      NULL, position_opt, eval_desc_opt);
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeLin1D(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double axis_1[3],
    const double &range_1, const double &n_div,
    const double &w_depth, const double &w_normal,
    double opt_1[1], double position_opt[3], TEvalDescription *eval_desc_opt)
{
  Eigen::Vector3d xyz0(poses_[index].X), a1(axis_1), xyz(0.0,0.0,0.0);
  double r1(0.5*range_1);
  double t1_best(0.0);
  // Evaluate current pose:
  TEvalDescription eval_desc_best;
  GetEvalDescription(index, depth_img, normal_img,
      eval_desc_best, step_xp, step_yp, xyz0[2]);
  // Brute force optimization:
  for(double t1(-r1); t1<+r1; t1+= 2.0*r1/n_div)
  {
    xyz= xyz0 + t1*a1;
    SetXYZ(index, xyz[0],xyz[1],xyz[2]);
    TEvalDescription eval_desc;
    if(!GetEvalDescription(index, depth_img, normal_img,
          eval_desc, step_xp, step_yp, xyz0[2]))
      continue;
    if(CompareEvalDescriptions(eval_desc,eval_desc_best,w_depth,w_normal)>0)
    {
      t1_best= t1;
      eval_desc_best= eval_desc;
    }
  }
  xyz= xyz0 + t1_best*a1;
  SetXYZ(index, xyz[0],xyz[1],xyz[2]);
  if(opt_1)
  {
    opt_1[0]= t1_best;
  }
  if(position_opt)
  {
    position_opt[0]= xyz[0];
    position_opt[1]= xyz[1];
    position_opt[2]= xyz[2];
  }
  if(eval_desc_opt)
  {
    *eval_desc_opt= eval_desc_best;
  }
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeLin2D(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double axis_1[3], const double axis_2[3],
    const double &range_1, const double &range_2, const double &n_div,
    const double &w_depth, const double &w_normal,
    double opt_12[2], double position_opt[3], TEvalDescription *eval_desc_opt)
{
  Eigen::Vector3d xyz0(poses_[index].X), a1(axis_1), a2(axis_2), xyz(0.0,0.0,0.0);
  double r1(0.5*range_1), r2(0.5*range_2);
  double t1_best(0.0), t2_best(0.0);
  // Evaluate current pose:
  TEvalDescription eval_desc_best;
  GetEvalDescription(index, depth_img, normal_img,
      eval_desc_best, step_xp, step_yp, xyz0[2]);
  // Brute force optimization:
  for(double t1(-r1); t1<+r1; t1+= 2.0*r1/n_div)
  {
    for(double t2(-r2); t2<+r2; t2+= 2.0*r2/n_div)
    {
      xyz= xyz0 + t1*a1 + t2*a2;
      SetXYZ(index, xyz[0],xyz[1],xyz[2]);
      TEvalDescription eval_desc;
      if(!GetEvalDescription(index, depth_img, normal_img,
            eval_desc, step_xp, step_yp, xyz0[2]))
        continue;
      if(CompareEvalDescriptions(eval_desc,eval_desc_best,w_depth,w_normal)>0)
      {
        t1_best= t1;
        t2_best= t2;
        eval_desc_best= eval_desc;
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
  if(eval_desc_opt)
  {
    *eval_desc_opt= eval_desc_best;
  }
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeRot1D(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double axis_1[3],
    const double &range_1, const double &n_div,
    const double &w_depth, const double &w_normal,
    double opt_1[1], double rotation_opt[4], TEvalDescription *eval_desc_opt)
{
  double z0(poses_[index].X[2]);
  Eigen::Quaterniond q0(QToEigMat(poses_[index].X+3));
  Eigen::Vector3d a1(axis_1);
  double qx[4]= {0.0,0.0,0.0,0.0};
  double r1(0.5*range_1);
  double t1_best(0.0);
  // Evaluate current pose:
  TEvalDescription eval_desc_best;
  GetEvalDescription(index, depth_img, normal_img,
      eval_desc_best, step_xp, step_yp, z0);
  // Brute force optimization:
  for(double t1(-r1); t1<+r1; t1+= 2.0*r1/n_div)
  {
    Eigen::Quaterniond q= Eigen::AngleAxisd(t1, a1) * q0;
    EigMatToQ(q, qx);
    SetQ(index, qx);
    TEvalDescription eval_desc;
    if(!GetEvalDescription(index, depth_img, normal_img,
          eval_desc, step_xp, step_yp, z0))
      continue;
    if(CompareEvalDescriptions(eval_desc,eval_desc_best,w_depth,w_normal)>0)
    {
      t1_best= t1;
      eval_desc_best= eval_desc;
    }
  }
  Eigen::Quaterniond q= Eigen::AngleAxisd(t1_best, a1) * q0;
  EigMatToQ(q, qx);
  SetQ(index, qx);
  if(opt_1)
  {
    opt_1[0]= t1_best;
  }
  if(rotation_opt)
  {
    rotation_opt[0]= qx[0];
    rotation_opt[1]= qx[1];
    rotation_opt[2]= qx[2];
    rotation_opt[3]= qx[3];
  }
  if(eval_desc_opt)
  {
    *eval_desc_opt= eval_desc_best;
  }
}
//-------------------------------------------------------------------------------------------

void TRayTracePoseEstimator::OptimizeRot2D(
    int index,
    cv::Mat &depth_img, cv::Mat &normal_img,
    int step_xp, int step_yp,
    const double axis_1[3], const double axis_2[3],
    const double &range_1, const double &range_2, const double &n_div,
    const double &w_depth, const double &w_normal,
    double opt_12[2], double rotation_opt[4], TEvalDescription *eval_desc_opt)
{
  double z0(poses_[index].X[2]);
  Eigen::Quaterniond q0(QToEigMat(poses_[index].X+3));
  Eigen::Vector3d a1(axis_1), a2(axis_2);
  double qx[4]= {0.0,0.0,0.0,0.0};
  double r1(0.5*range_1), r2(0.5*range_2);
  double t1_best(0.0), t2_best(0.0);
  // Evaluate current pose:
  TEvalDescription eval_desc_best;
  GetEvalDescription(index, depth_img, normal_img,
      eval_desc_best, step_xp, step_yp, z0);
  // Brute force optimization:
  for(double t1(-r1); t1<+r1; t1+= 2.0*r1/n_div)
  {
    for(double t2(-r2); t2<+r2; t2+= 2.0*r2/n_div)
    {
      Eigen::Quaterniond q= Eigen::AngleAxisd(t2, a2) * (Eigen::AngleAxisd(t1, a1) * q0);
      EigMatToQ(q, qx);
      SetQ(index, qx);
      TEvalDescription eval_desc;
      if(!GetEvalDescription(index, depth_img, normal_img,
            eval_desc, step_xp, step_yp, z0))
        continue;
      if(CompareEvalDescriptions(eval_desc,eval_desc_best,w_depth,w_normal)>0)
      {
        t1_best= t1;
        t2_best= t2;
        eval_desc_best= eval_desc;
      }
    }
  }
  Eigen::Quaterniond q= Eigen::AngleAxisd(t2_best, a2) * (Eigen::AngleAxisd(t1_best, a1) * q0);
  EigMatToQ(q, qx);
  SetQ(index, qx);
  if(opt_12)
  {
    opt_12[0]= t1_best;
    opt_12[1]= t2_best;
  }
  if(rotation_opt)
  {
    rotation_opt[0]= qx[0];
    rotation_opt[1]= qx[1];
    rotation_opt[2]= qx[2];
    rotation_opt[3]= qx[3];
  }
  if(eval_desc_opt)
  {
    *eval_desc_opt= eval_desc_best;
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

