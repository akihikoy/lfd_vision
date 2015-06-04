//-------------------------------------------------------------------------------------------
/*! \file    ode_pour_sim.cpp
    \brief   Grasping and pouring simulator using ODE.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.18, 2015
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/ode_grpour_sim.h"
#include "pr2_lfd_vision/geom_util.h"
//-------------------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <list>
#include <algorithm>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

//-------------------------------------------------------------------------------------------
namespace ode_pour
{

// int MaxContacts(10);  // maximum number of contact points per body
int MaxContacts(1);  // maximum number of contact points per body
double VizJointLen(0.1);
double VizJointRad(0.02);
int    BallNum(100);
int    BallType(0);  // 0: Sphere, 1: Box
double BallRad(0.025);
double BallBoxRatio(1.7);  // If BallType is box, its size = BallRad*this value
double BoxThickness(0.005);
// double GripThickness(0.01);
double RcvPos[3]= {0.6, 0.0, 0.0};
double RcvSize[3]= {0.3, 0.4, 0.2};
double ContactBounce(0.1);
double ContactBounceVel(0.01);
double ContactSoftCFM(0.1);
double GripperHeight(0.40);
double TargetAngle(0.0);
double TargetPourX(0.0),TargetPourY(0.0),TargetPourZ(0.0);
// double TargetGripperPos(0.0);
// double TargetGripperX(0.0),TargetGripperY(0.0),TargetGripperZ(0.0),TargetGripperTh(0.0);
double TimeStep(0.04);
double Gravity(-0.5);
bool Running(true);
void (*SensingCallback)(const TSensors1 &sensors)= NULL;
void (*StepCallback)(const double &time, const double &time_step)= NULL;
//-------------------------------------------------------------------------------------------

double PPourOffset[3]= {0.0,0.0,0.0};
TRotatedBoundingBox<double,/*using_minmax=*/true> BoundBoxSrc, BoundBoxRcv;

static TEnvironment *Env(NULL);
static void NearCallback(void*,dGeomID,dGeomID);
//-------------------------------------------------------------------------------------------


dReal IndexedColors[][4]= {
    /*0*/ {1.0, 0.0, 0.0, 0.8},
          {0.0, 1.0, 0.0, 0.8},
          {0.0, 0.0, 1.0, 0.8},
    /*3*/ {1.0, 1.0, 0.0, 0.8},
          {1.0, 0.0, 1.0, 0.8},
          {0.0, 1.0, 1.0, 0.8},
    /*6*/ {1.0, 0.0, 0.0, 0.3},
          {0.0, 1.0, 0.0, 0.3},
          {0.0, 0.0, 1.0, 0.3},
    /*9*/ {1.0, 1.0, 0.0, 0.3},
          {1.0, 0.0, 1.0, 0.3},
          {0.0, 1.0, 1.0, 0.3},
    /*12*/{1.0, 1.0, 1.0, 0.8}};
inline void SetColor(int i)
{
  dReal *col= IndexedColors[i];
  dsSetColorAlpha(col[0],col[1],col[2],col[3]);
}
//-------------------------------------------------------------------------------------------

// Insert value to container sorted in descending order (>>>)
template <typename t_container>
inline void InsertDescending(t_container &container, const typename t_container::value_type &value)
{
  container.insert(std::find_if(container.begin(),container.end(),
      std::bind2nd(std::less<typename t_container::value_type>(),value)),value);
}
//-------------------------------------------------------------------------------------------

/* ODE::dBody pose to pose.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void ODEBodyToX(const dBody &body, t_value x[7])
{
  const dReal *p= body.getPosition();  // x,y,z
  const dReal *q= body.getQuaternion();  // qw,qx,qy,qz
  x[0]= p[0]; x[1]= p[1]; x[2]= p[2];
  x[3]= q[1]; x[4]= q[2]; x[5]= q[3]; x[6]= q[0];
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TDynRobot
//===========================================================================================

void TDynRobot::Draw()
{
  dsSetTexture (DS_WOOD);

  dReal rad, len;
  dReal sides[4];
  for (std::vector<TNCBox>::const_iterator itr(link_b_.begin()),last(link_b_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getLengths(sides);
    dsDrawBox (itr->getPosition(), itr->getRotation(), sides);
  }
  for (std::vector<TNCSphere>::const_iterator itr(link_sp_.begin()),last(link_sp_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    dsDrawSphere (itr->getPosition(), itr->getRotation(), itr->getRadius());
  }
  for (std::vector<TNCCapsule>::const_iterator itr(link_ca_.begin()),last(link_ca_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getParams(&rad, &len);
    dsDrawCapsule (itr->getPosition(), itr->getRotation(), len,rad);
  }
  for (std::vector<TNCCylinder>::const_iterator itr(link_cy_.begin()),last(link_cy_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getParams(&rad, &len);
    dsDrawCylinder (itr->getPosition(), itr->getRotation(), len,rad);
  }
  for (std::vector<TTriMeshGeom>::const_iterator itr(link_tm_.begin()),last(link_tm_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    // const dReal *pos = itr->getPosition();
    // const dReal *rot = itr->getRotation();
    const dVector3 pos={0,0,0,0};
    const dMatrix3 rot={1,0,0,0 ,0,1,0,0, 0,0,1,0};

    for (int i(dGeomTriMeshGetTriangleCount(*itr)); i>0; --i)
    {
      std::cerr<<"i:"<<i<<std::endl;
      dVector3 v[3];
      dGeomTriMeshGetTriangle(*itr, i-1, &v[0], &v[1], &v[2]);
      std::cerr<<v[0][0]<<" "<<v[0][1]<<" "<<v[0][2]<<std::endl;
      std::cerr<<v[1][0]<<" "<<v[1][1]<<" "<<v[1][2]<<std::endl;
      std::cerr<<v[2][0]<<" "<<v[2][1]<<" "<<v[2][2]<<std::endl;
      dsDrawTriangle(pos, rot, v[0], v[1], v[2], 1);
    }
  }

  dVector3 pos,axis;
  dMatrix3 rot;
  for (std::vector<TNCHingeJoint>::const_iterator itr(joint_h_.begin()),last(joint_h_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getAnchor(pos);
    itr->getAxis(axis);
    dRFromAxisAndAngle (rot,-axis[1],axis[0],0.0, 0.5*M_PI-std::asin(axis[2]));
    dsDrawCylinder (pos, rot, VizJointLen,VizJointRad);
  }

  // for (std::vector<TNCSliderJoint>::const_iterator itr(joint_s_.begin()),last(joint_s_.end()); itr!=last; ++itr)
  // {
    // SetColor(itr->ColorCode);
    // // itr->getAnchor(pos);
    // const dReal *p1(dBodyGetPosition(itr->getBody(0))), *p2(dBodyGetPosition(itr->getBody(1)));
    // for(int d(0); d<3; ++d)  pos[d]= 0.5*(p1[d]+p2[d]);
    // itr->getAxis(axis);
    // dRFromAxisAndAngle (rot,-axis[1],axis[0],0.0, 0.5*M_PI-std::asin(axis[2]));
    // dsDrawCylinder (pos, rot, VizJointLen, VizJointRad);
  // }
}
//-------------------------------------------------------------------------------------------

#if 0
//===========================================================================================
// class TGripper1 : public TDynRobot
//===========================================================================================

/*override*/void TGripper1::Create(dWorldID world, dSpaceID space)
{
  double B[3];
  B[0]= 0.0;  B[1]= -0.36;  B[2]= 0.15;

  body_.clear();
  link_b_.clear();
  link_ca_.clear();
  link_cy_.clear();
  link_sp_.clear();
  link_tm_.clear();

  body_.resize(6);
  link_sp_.resize(3);
  link_b_.resize(3);

  double dummy_sp_rad= 0.005;
  double initw= 0.32;
  double sizes0[3]= {0.05,0.1,0.02};
  double sizey= 0.3, sizez= 0.08;
  double thickness= GripThickness;
  double sp_poss[3][3]={
      { 0.0, 0.0, 0.0},
      { 0.0, 2.0*dummy_sp_rad, 0.0},
      { 0.0, 4.0*dummy_sp_rad, 0.0}
      };
  double box_sizes[3][3]={
      {sizes0[0], sizes0[1], sizes0[2]},
      {thickness, sizey, sizez},
      {thickness, sizey, sizez}
      };
  double box_poss[3][3]={
      { 0.0, 5.0*dummy_sp_rad+0.5*sizes0[1], 0.0},
      { 0.5*initw, 5.0*dummy_sp_rad+sizes0[1]+0.5*sizey, 0.0},
      {-0.5*initw, 5.0*dummy_sp_rad+sizes0[1]+0.5*sizey, 0.0}
      };
  double box_rots[3][4]={  // angle, axis-x, axis-y, axis-z
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0}
      };

  int ib(0);
  for(int i(0); i<3; ++i,++ib)
  {
    link_sp_[i].create(space, dummy_sp_rad);
    body_[ib].create(world);
    body_[ib].setPosition(B[0]+sp_poss[i][0], B[1]+sp_poss[i][1], B[2]+sp_poss[i][2]);
    dMass m;
    m.setSphere(1.0, dummy_sp_rad);
    body_[ib].setMass(&m);
    link_sp_[i].setBody(body_[ib]);
    link_sp_[i].ColorCode= 5;
  }

  // body_[0].create(world);
  // body_[0].setPosition(B[0], B[1], B[2]+0.5*sizez);
  // dMass m;
  // m.setBox(1.0, sizexy, sizexy, sizez);
  // body_[0].setMass(&m);
  for(int i(0); i<3; ++i,++ib)
  {
    link_b_[i].create(space, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[ib].create(world);
    body_[ib].setPosition(B[0]+box_poss[i][0], B[1]+box_poss[i][1], B[2]+box_poss[i][2]);
    // link_b_[i].setPosition(B[0]+box_poss[i][0], B[1]+box_poss[i][1], B[2]+box_poss[i][2]);
    if(box_rots[i][0]!=0.0)
    {
      dMatrix3 R;
      dRFromAxisAndAngle(R,box_rots[i][1],box_rots[i][2],box_rots[i][3],box_rots[i][0]);
      body_[ib].setRotation (R);
      // link_b_[i].setRotation (R);
    }
    dMass m;
    m.setBox(1.0, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[ib].setMass(&m);
    link_b_[i].setBody(body_[ib]);
    // link_b_[i].setBody(body_[0]);
    link_b_[i].ColorCode= 5;
  }

  // joint_h_.resize(1);
  // int bid= 3;  // Attaching to this body
  // // int bid= 0;  // Attaching to this body
  // dReal fmax(1000.0);  // NOTE: set zero to control by joint torque
  // joint_h_[0].create(world);
  // joint_h_[0].attach(body_[bid],0);
  // joint_h_[0].setAnchor(B[0]+box_poss[bid][0]+0.5*sizexy, B[1]+box_poss[bid][1], B[2]+box_poss[bid][2]+0.5*sizez);
  // joint_h_[0].setAxis(0.0,1.0,0.0);
  // joint_h_[0].setParam(dParamFMax,fmax);

  dReal fmaxs(100.0);  // NOTE: set zero to control by joint torque
  joint_s_.resize(5);
  joint_s_[0].create(world);
  joint_s_[0].attach(0,body_[0]);
  joint_s_[0].setAxis(1.0,0.0,0.0);
  joint_s_[0].setParam(dParamFMax,fmaxs);

  joint_s_[1].create(world);
  joint_s_[1].attach(body_[0],body_[1]);
  joint_s_[1].setAxis(0.0,1.0,0.0);
  joint_s_[1].setParam(dParamFMax,fmaxs);

  joint_s_[2].create(world);
  joint_s_[2].attach(body_[1],body_[2]);
  joint_s_[2].setAxis(0.0,0.0,1.0);
  joint_s_[2].setParam(dParamFMax,fmaxs);

  joint_s_[3].create(world);
  joint_s_[3].attach(body_[3],body_[4]);
  joint_s_[3].setAxis(-1.0,0.0,0.0);
  joint_s_[3].setParam(dParamFMax,fmaxs);

  joint_s_[4].create(world);
  joint_s_[4].attach(body_[3],body_[5]);
  joint_s_[4].setAxis(1.0,0.0,0.0);
  joint_s_[4].setParam(dParamFMax,fmaxs);

  joint_h_.resize(1);
  dReal fmax(1000.0);  // NOTE: set zero to control by joint torque
  joint_h_[0].create(world);
  joint_h_[0].attach(body_[2],body_[3]);
  joint_h_[0].setAnchor(B[0], B[1]+5.0*dummy_sp_rad, B[2]);
  joint_h_[0].setAxis(1.0,0.0,0.0);
  joint_h_[0].setParam(dParamFMax,fmax);
}
//-------------------------------------------------------------------------------------------

#endif


//===========================================================================================
// class TSrcContainer1 : public TDynRobot
//===========================================================================================

/*override*/void TSrcContainer1::Create(dWorldID world, dSpaceID space)
{
  double B[3];
  B[0]= 0.0;  B[1]= 0.0;  B[2]= 0.0;

  body_.clear();
  link_b_.clear();
  link_ca_.clear();
  link_cy_.clear();
  link_sp_.clear();
  link_tm_.clear();

  // dReal side(0.2),mass(1.0);

  body_.resize(9+2+3);
  link_b_.resize(9+2);  // +2: gripper
  link_sp_.resize(3);  // xyz sliders

  int ib(0),ibox(0);

  // Some container dimensions:
  double sizexy(0.3), sizez(0.5);

  // XYZ sliders:
  double dummy_sp_rad= 0.005;
  double sp_poss[3][3]={
      { 0.0, -1.0+0.0, 10.0},
      { 0.0, -1.0+2.0*dummy_sp_rad, 10.0},
      { 0.0, -1.0+4.0*dummy_sp_rad, 10.0}
      };
  for(int i(0); i<3; ++i,++ib)
  {
    link_sp_[i].create(space, dummy_sp_rad);
    body_[ib].create(world);
    body_[ib].setPosition(B[0]+sp_poss[i][0], B[1]+sp_poss[i][1], B[2]+sp_poss[i][2]);
    dMass m;
    m.setSphere(1.0, dummy_sp_rad);
    body_[ib].setMass(&m);
    link_sp_[i].setBody(body_[ib]);
    link_sp_[i].ColorCode= 5;
  }

  // Gripper:
  double g_thickness(0.15),g_margin(0.005);
  double gripper_sizes[2][3]={
      {g_thickness, 0.35, 0.1},
      {g_thickness, 0.35, 0.1}};
  double gripper_poss[2][3]={
      {+0.5*sizexy+0.5*g_thickness+g_margin, -0.05, GripperHeight},
      {-0.5*sizexy-0.5*g_thickness-g_margin, -0.05, GripperHeight}};
  for(int i(0); i<2; ++i,++ib,++ibox)
  {
    link_b_[ibox].create(space, gripper_sizes[i][0], gripper_sizes[i][1], gripper_sizes[i][2]);
    body_[ib].create(world);
    body_[ib].setPosition(B[0]+gripper_poss[i][0], B[1]+gripper_poss[i][1], B[2]+gripper_poss[i][2]);
    dMass m;
    m.setBox(1.0, gripper_sizes[i][0], gripper_sizes[i][1], gripper_sizes[i][2]);
    body_[ib].setMass(&m);
    link_b_[ibox].setBody(body_[ib]);
    link_b_[ibox].ColorCode= 12;
  }

  // Container dimensions:
  double size2s(0.3), size2h(0.03);
  double thickness= BoxThickness;
  double bottom_z= 0.0;
  double box_sizes[9][3]={
      {sizexy-2.0*thickness, sizexy-2.0*thickness, thickness},
      {thickness, sizexy, sizez},
      {thickness, sizexy, sizez},
      {sizexy-2.0*thickness, thickness, sizez},
      {sizexy-2.0*thickness, thickness, sizez},
      {thickness, size2s, M_SQRT2*size2h},
      {thickness, size2s, M_SQRT2*size2h},
      {size2s-2.0*thickness, thickness, M_SQRT2*size2h},
      {size2s-2.0*thickness, thickness, M_SQRT2*size2h}
      };
  double box_poss[9][3]={
      {0.0, 0.0, 0.5*thickness+bottom_z},
      {0.5*sizexy-0.5*thickness, 0.0, 0.5*sizez+bottom_z},
      {-0.5*sizexy+0.5*thickness, 0.0, 0.5*sizez+bottom_z},
      {0.0, 0.5*sizexy-0.5*thickness, 0.5*sizez+bottom_z},
      {0.0, -0.5*sizexy+0.5*thickness, 0.5*sizez+bottom_z},
      {0.5*sizexy-0.5*size2h-0.5*thickness, 0.0, 0.5*size2h+sizez+bottom_z},
      {-0.5*sizexy+0.5*size2h+0.5*thickness, 0.0, 0.5*size2h+sizez+bottom_z},
      {0.0, 0.5*sizexy-0.5*size2h-0.5*thickness, 0.5*size2h+sizez+bottom_z},
      {0.0, -0.5*sizexy+0.5*size2h+0.5*thickness, 0.5*size2h+sizez+bottom_z}
      };
  double box_rots[9][4]={  // angle, axis-x, axis-y, axis-z
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.25*M_PI, 0.0,-1.0, 0.0},
      {0.25*M_PI, 0.0, 1.0, 0.0},
      {0.25*M_PI, 1.0, 0.0, 0.0},
      {0.25*M_PI,-1.0, 0.0, 0.0}
      };

  // Container:
  // body_[0].create(world);
  // body_[0].setPosition(B[0], B[1], B[2]+0.5*sizez);
  // dMass m;
  // m.setBox(1.0, sizexy, sizexy, sizez);
  // body_[0].setMass(&m);
  for(int i(0); i<9; ++i,++ib,++ibox)
  {
    link_b_[ibox].create(space, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[ib].create(world);
    body_[ib].setPosition(B[0]+box_poss[i][0], B[1]+box_poss[i][1], B[2]+box_poss[i][2]);
    // link_b_[ibox].setPosition(B[0]+box_poss[i][0], B[1]+box_poss[i][1], B[2]+box_poss[i][2]);
    if(box_rots[i][0]!=0.0)
    {
      dMatrix3 R;
      dRFromAxisAndAngle(R,box_rots[i][1],box_rots[i][2],box_rots[i][3],box_rots[i][0]);
      body_[ib].setRotation (R);
      // link_b_[ibox].setRotation (R);
    }
    dMass m;
    m.setBox(1.0, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[ib].setMass(&m);
    link_b_[ibox].setBody(body_[ib]);
    // link_b_[ibox].setBody(body_[0]);
    link_b_[ibox].ColorCode= 5;
  }

  // Update the bounding box info:
  // We use body_[5] (base of container) as the frame base pose.
  // Values are not accurate.
  double *min(BoundBoxSrc.Min), *max(BoundBoxSrc.Max);
  min[0]= -0.5*sizexy; min[1]= -0.5*sizexy; min[2]= -0.5*thickness;
  max[0]= +0.5*sizexy; max[1]= +0.5*sizexy; max[2]= sizez+size2h  ;

  // int dir(1);
  // dMatrix3 R;
  // dRSetIdentity (R);  // Z
  // if(dir==1) dRFromAxisAndAngle (R,0.0,1.0,0.0,0.5*M_PI);  // X
  // if(dir==2) dRFromAxisAndAngle (R,1.0,0.0,0.0,0.5*M_PI);  // Y
  // body_[0].setRotation (R);

  joint_f_.resize(8+2);
  for(int i(3),j(0); i<3+9+2; ++i)
  {
    if(i==5)  continue;
    joint_f_[j].create(world);
    joint_f_[j].attach(body_[5],body_[i]);
    joint_f_[j].set();
    ++j;
  }

  dReal fmaxs(100.0);  // NOTE: set zero to control by joint torque
  joint_s_.resize(5);
  joint_s_[0].create(world);
  joint_s_[0].attach(body_[0],0);
  joint_s_[0].setAxis(1.0,0.0,0.0);
  joint_s_[0].setParam(dParamFMax,fmaxs);

  joint_s_[1].create(world);
  joint_s_[1].attach(body_[1],body_[0]);
  joint_s_[1].setAxis(0.0,1.0,0.0);
  joint_s_[1].setParam(dParamFMax,fmaxs);

  joint_s_[2].create(world);
  joint_s_[2].attach(body_[2],body_[1]);
  joint_s_[2].setAxis(0.0,0.0,1.0);
  joint_s_[2].setParam(dParamFMax,fmaxs);

  joint_h_.resize(1);
  int bid= 9, boxid= 4;  // Attaching to this body
  // int bid= 0;  // Attaching to this body
  dReal fmax(1000.0);  // NOTE: set zero to control by joint torque
  joint_h_[0].create(world);
  joint_h_[0].attach(body_[bid],body_[2]);
  joint_h_[0].setAnchor(B[0]+box_poss[boxid][0]+0.5*sizexy, B[1]+box_poss[boxid][1], B[2]+box_poss[boxid][2]+0.5*sizez);
  joint_h_[0].setAxis(0.0,1.0,0.0);
  joint_h_[0].setParam(dParamFMax,fmax);

  // Update pouring point offset:
  dVector3 p_joint;
  joint_h_[0].getAnchor(p_joint);
  PPourOffset[0]= p_joint[0];
  PPourOffset[1]= 0.0;
  PPourOffset[2]= p_joint[2];
  TargetPourX= PPourOffset[0];
  TargetPourY= PPourOffset[1];
  TargetPourZ= PPourOffset[2];
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TRcvContainer1 : public TDynRobot
//===========================================================================================

/*override*/void TRcvContainer1::Create(dWorldID world, dSpaceID space)
{
  double *B(RcvPos), *Size(RcvSize);

  body_.clear();
  link_b_.clear();
  link_ca_.clear();
  link_cy_.clear();
  link_sp_.clear();
  link_tm_.clear();

  // dReal side(0.2),mass(1.0);

  body_.resize(5);
  link_b_.resize(5);

  // double outer_size[]= {0.3,0.3,0.4};
  double thickness= BoxThickness;
  double bottom_z= 0.0;
  double box_sizes[5][3]={
      {Size[0]-2.0*thickness, Size[1]-2.0*thickness, thickness},
      {thickness, Size[1], Size[2]},
      {thickness, Size[1], Size[2]},
      {Size[0]-2.0*thickness, thickness, Size[2]},
      {Size[0]-2.0*thickness, thickness, Size[2]}
      };
  double box_poss[5][3]={
      {0.0, 0.0, 0.5*thickness+bottom_z},
      {0.5*Size[0]-0.5*thickness, 0.0, 0.5*Size[2]+bottom_z},
      {-0.5*Size[0]+0.5*thickness, 0.0, 0.5*Size[2]+bottom_z},
      {0.0, 0.5*Size[1]-0.5*thickness, 0.5*Size[2]+bottom_z},
      {0.0, -0.5*Size[1]+0.5*thickness, 0.5*Size[2]+bottom_z}
      };

  for(int i(0); i<5; ++i)
  {
    link_b_[i].create(space, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[i].create(world);
    body_[i].setPosition(B[0]+box_poss[i][0], B[1]+box_poss[i][1], B[2]+box_poss[i][2]);
    dMass m;
    // TEST: receiving container is heavier
    m.setBox(100.0, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    // m.setBox(1.0, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[i].setMass(&m);
    link_b_[i].setBody(body_[i]);
    link_b_[i].ColorCode= 3;
  }

  // Update the bounding box info:
  // We use body_[0] (base of container) as the frame base pose.
  // Values are not accurate.
  double *min(BoundBoxRcv.Min), *max(BoundBoxRcv.Max);
  min[0]= -0.5*Size[0]; min[1]= -0.5*Size[1]; min[2]= -0.5*thickness;
  max[0]= +0.5*Size[0]; max[1]= +0.5*Size[1]; max[2]= Size[2]       ;

  // int dir(1);
  // dMatrix3 R;
  // dRSetIdentity (R);  // Z
  // if(dir==1) dRFromAxisAndAngle (R,0.0,1.0,0.0,0.5*M_PI);  // X
  // if(dir==2) dRFromAxisAndAngle (R,1.0,0.0,0.0,0.5*M_PI);  // Y
  // body_[0].setRotation (R);

  joint_f_.resize(4);
  int j(0);
  for(int i(1); i<5; ++i,++j)
  {
    joint_f_[j].create(world);
    joint_f_[j].attach(body_[0],body_[i]);
    joint_f_[j].set();
  }

}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TBalls1 : public TDynRobot
//===========================================================================================

/*override*/void TBalls1::Create(dWorldID world, dSpaceID space)
{
  body_.clear();
  link_b_.clear();
  link_ca_.clear();
  link_cy_.clear();
  link_sp_.clear();
  link_tm_.clear();

  // dReal side(0.2),mass(1.0);

  int N(BallNum);
  double rad(BallRad);
  double init_z(0.30);

  body_.resize(N);
  switch(BallType)
  {
  case 0: link_sp_.resize(N);  break;
  case 1: link_b_.resize(N);  break;
  }

  for(int i(0); i<N; ++i)
  {
    double z= init_z + 0.3*rad*double(i);
    double xy_rad= 3.0*rad;
    double th= 0.73*M_PI*double(i);
    switch(BallType)
    {
    case 0: link_sp_[i].create(space, rad);  break;
    case 1: link_b_[i].create(space, BallBoxRatio*rad,BallBoxRatio*rad,BallBoxRatio*rad);  break;
    }
    body_[i].create(world);
    body_[i].setPosition(xy_rad*std::cos(th), xy_rad*std::sin(th), z);
    dMass m;
    m.setSphere(1.0, rad);
    body_[i].setMass(&m);
    switch(BallType)
    {
    case 0:
      link_sp_[i].setBody(body_[i]);
      link_sp_[i].ColorCode= 1;
      break;
    case 1:
      link_b_[i].setBody(body_[i]);
      link_b_[i].ColorCode= 1;
      break;
    }
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TEnvironment
//===========================================================================================

void TEnvironment::Create()
{
  contactgroup_.create ();
  world_.setGravity (0,0,Gravity);
  dWorldSetCFM (world_.id(),1e-5);
  plane_.create (space_,0,0,1,0);

  // gripper_.Create(world_,space_);
  source_.Create(world_,space_);
  receiver_.Create(world_,space_);
  balls_.Create(world_,space_);
  // geom_.Create(world_,space_);

  time_= 0.0;
  sensors_.Clear();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::StepSim(const double &time_step)
{
  ControlCallback(time_step);

  sensors_.Clear();

  space_.collide(0,&NearCallback);
  world_.step(time_step);
  time_+= time_step;

  contactgroup_.empty();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::Draw()
{
  if(Running)  DrawCallback();

  balls_.Draw();
  // gripper_.Draw();
  source_.Draw();
  receiver_.Draw();
  // geom_.Draw();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::ControlCallback(const double &time_step)
{
  // // static double angle = 0;
  // // angle += 0.01;
  // // robot_.Body().back().addForce (0.1*(std::sin(angle)+1.0), 0.1*(std::sin(angle*1.7)+1.0) ,0.1*(std::sin(angle*0.7)+1.0));
  // // robot_.Body().back().addForce (0,0,1.0*(std::sin(angle)+1.0));
  // // robot_.Body().front().addForce (0,0,0.01*(std::cos(angle)+1.0));
  dReal Kp(10.0);
  source_.SetVelH(0, Kp*(TargetAngle-source_.GetAngleH(0)));
  dReal Kps(10.0);
  source_.SetVelS(0, Kps*(TargetPourX-PPourOffset[0]-source_.GetPosS(0)));
  source_.SetVelS(1, Kps*(TargetPourY-PPourOffset[1]-source_.GetPosS(1)));
  source_.SetVelS(2, Kps*(TargetPourZ-PPourOffset[2]-source_.GetPosS(2)));
  #if 0
  dReal Kps(10.0);
  gripper_.SetVelS(0, Kps*(TargetGripperX-gripper_.GetPosS(0)));
  gripper_.SetVelS(1, Kps*(TargetGripperY-gripper_.GetPosS(1)));
  gripper_.SetVelS(2, Kps*(TargetGripperZ-gripper_.GetPosS(2)));
  gripper_.SetVelS(3, Kps*(0.5*TargetGripperPos-gripper_.GetPosS(3)));
  gripper_.SetVelS(4, Kps*(0.5*TargetGripperPos-gripper_.GetPosS(4)));
  dReal Kph(10.0);
  gripper_.SetVelH(0, Kph*(TargetGripperTh-gripper_.GetAngleH(0)));
  // dBody &gb(gripper_.Body().front());
  // gb.addForce(0,0,0.01*(TargetGripperZ-gb.getPosition()[2]));
  #endif
}
//-------------------------------------------------------------------------------------------

void TEnvironment::DrawCallback()
{
  ODEBodyToX(source_.Body(5), BoundBoxSrc.X);
  ODEBodyToX(receiver_.Body(0), BoundBoxRcv.X);
  Eigen::Affine3d  inv_x_src(BoundBoxSrc.InvX()), inv_x_rcv(BoundBoxRcv.InvX());
  std::vector<TNCBody> &balls_b(balls_.BallsB());
  // std::vector<TNCSphere> &balls_g(balls_.BallsG());
  sensors_.BallSt.resize(balls_b.size());
  sensors_.BallX.resize(balls_b.size());
  int num_src(0), num_rcv(0), num_flow(0), num_spill(0);
  double speed;
  std::list<double> z_rcv_data;
  for(size_t i(0); i<balls_b.size(); ++i)
  {
    const dReal *pos(balls_b[i].getPosition());
    const dReal *vel(balls_b[i].getLinearVel());
    for(int d(0);d<3;++d)  sensors_.BallX[i].X[d]=pos[d];
    for(int d(0);d<3;++d)  sensors_.BallX[i].X[3+d]=vel[d];
    // dReal angle= std::atan2(-(pos[0]-0.15),-(pos[2]-0.6));
    speed= std::sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
    // if(angle<0.0)  angle+= 2.0*M_PI;
    // dReal angle_base= TargetAngle+0.5*M_PI;

    sensors_.BallSt[i]= 0;
    if(BoundBoxSrc.IsIn(pos,&inv_x_src))
    {
      balls_.SetBallCol(i,1);
      ++num_src;
      sensors_.BallSt[i]= 1;
    }
    else if(BoundBoxRcv.IsIn(pos,&inv_x_rcv))
    {
      if(speed<0.1)
      {
        balls_.SetBallCol(i,0);
        ++num_rcv;
        sensors_.BallSt[i]= 2;
        if(speed<0.02)
          InsertDescending(z_rcv_data, pos[2]-(BoundBoxRcv.X[2]+0.5*BoxThickness+0.5*BallRad));
      }
      else
      {
        balls_.SetBallCol(i,2);
        ++num_flow;
        sensors_.BallSt[i]= 3;
      }
    }
    else if(pos[2]<2.0*BallRad/*speed<0.1*/)
    {
      balls_.SetBallCol(i,3);
      ++num_spill;
      sensors_.BallSt[i]= 4;
    }
    else
    {
      balls_.SetBallCol(i,2);
      ++num_flow;
      sensors_.BallSt[i]= 3;
    }
    if(speed>0.1)  balls_.SetBallCol(i, balls_.BallCol(i)+6);
  }
  // std::cerr<<"angle= "<<angle<<"  ";
  double z_rcv(0.0);
  int num_big= std::min(int(z_rcv_data.size()), 10);
  std::list<double>::iterator z_rcv_itr(z_rcv_data.begin());
  for(int i(0); i<num_big; ++z_rcv_itr,++i)  z_rcv+= *z_rcv_itr;
  if(num_big>0)  z_rcv/= double(num_big);

  sensors_.NumSrc= num_src;
  sensors_.NumRcv= num_rcv;
  sensors_.NumFlow= num_flow;
  sensors_.NumSpill= num_spill;
  sensors_.ZRcv= z_rcv;

  for(int d(0);d<7;++d)  sensors_.XSrc[d]= BoundBoxSrc.X[d];
  for(int d(0);d<7;++d)  sensors_.XRcv[d]= BoundBoxRcv.X[d];
  sensors_.PPour[0]= TargetPourX;
  sensors_.PPour[1]= TargetPourY;
  sensors_.PPour[2]= TargetPourZ;
  sensors_.Theta= TargetAngle;

  sensors_.Time= time_;

  std::cerr<<"#src, #flow, #rcv, #spill, z_rcv= "
    <<num_src<<", "<<num_flow<<", "<<num_rcv<<", "<<num_spill<<", "<<z_rcv/*<<", "<<speed*/<<std::endl;
  if(sensors_.GripperColliding || sensors_.SrcColliding)
    std::cerr<<"Collision: "
        <<(sensors_.GripperColliding?"[Gripper]":"")
        <<(sensors_.SrcColliding?"[Source]":"")<<std::endl;

  if(SensingCallback!=NULL)  SensingCallback(sensors_);

}
//-------------------------------------------------------------------------------------------

/* Called when b1 and b2 are colliding.
    Return whether we ignore this collision (true: ignore collision). */
bool TEnvironment::CollisionCallback(dBodyID &b1, dBodyID &b2, std::valarray<dContact> &contact)
{
  const dBodyID b_grippers[2]= {source_.Body(3).id(), source_.Body(4).id()};
  for(int i(0); i<2; ++i)
  {
    if(b1==b_grippers[i] || b2==b_grippers[i])
    {
      sensors_.GripperColliding= true;
      return true;  // Ignore gripper collision
    }
  }

  // If a ball is colliding, return false (does not ignore the collision)
  for(std::vector<TNCBody>::const_iterator itr(balls_.Body().begin()),itrend(balls_.Body().end());
      itr!=itrend; ++itr)
  {
    if(b1==itr->id() || b2==itr->id())  return false;
  }

  bool sb1(false),sb2(false);  // Becomes true if b* is a part of src container
  for(int i(5),iend(source_.Body().size()); i<iend; ++i)
  {
    const dBodyID b_src= source_.Body(i).id();
    if(b1==b_src)  sb1= true;
    if(b2==b_src)  sb2= true;
    if(sb1 && sb2)  return true;  // Ignore collision inside src container blocks
  }
  if(sb1 || sb2)  // except for sb1 && sb2
    sensors_.SrcColliding= true;

  return false;
}
//-------------------------------------------------------------------------------------------

// Save ball state into file
void TEnvironment::DumpBallStat(const char *file_name)
{
  std::ofstream ofs(file_name);
  std::vector<TNCBody> &balls_b(balls_.BallsB());
  double speed;
  for(size_t i(0); i<balls_b.size(); ++i)
  {
    const dReal *pos(balls_b[i].getPosition());
    const dReal *vel(balls_b[i].getLinearVel());
    ofs<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" "
      <<vel[0]<<" "<<vel[1]<<" "<<vel[2]<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


static void NearCallback(void *data, dGeomID o1, dGeomID o2)
{
  assert(Env!=NULL);

  // do nothing if the two bodies are connected by a joint
  dBodyID b1= dGeomGetBody(o1);
  dBodyID b2= dGeomGetBody(o2);
  if(b1 && b2 && dAreConnected(b1,b2)) return;

  std::valarray<dContact> contact(MaxContacts);   // up to MaxContacts contacts per link
  for(int i(0); i<MaxContacts; ++i)
  {
    contact[i].surface.mode= dContactBounce | dContactSoftCFM;
    contact[i].surface.mu= 0.001; // dInfinity;
    contact[i].surface.mu2= 0.1;
    contact[i].surface.bounce= ContactBounce;  // 0.1;
    contact[i].surface.bounce_vel= ContactBounceVel;  // 0.01;
    contact[i].surface.soft_cfm= ContactSoftCFM;  // 0.1
  }
  if(int numc=dCollide(o1,o2,MaxContacts,&contact[0].geom,sizeof(dContact)))
  {
    if(Env->CollisionCallback(b1,b2,contact))  return;  // ignore if the callback returns true
    for(int i(0); i<numc; ++i)
    {
      dJointID c= dJointCreateContact(Env->WorldID(),Env->ContactGroupID(),&contact[i]);
      dJointAttach (c,b1,b2);
    }
  }
}
//-------------------------------------------------------------------------------------------

static void SimStart()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.4405,-0.4452,0.8200};
  static float hpr[3] = {123.5000,-35.0000,0.0000};
  dsSetViewpoint (xyz,hpr);
}
//-------------------------------------------------------------------------------------------

static void SimLoop (int pause)
{
  assert(Env!=NULL);

  if (!pause && Running)
  {
    Env->StepSim(TimeStep);
  }

  Env->Draw();
  if(StepCallback!=NULL)  StepCallback(Env->Time(), TimeStep);
}
//-------------------------------------------------------------------------------------------

static void SimKeyevent (int command)
{
  assert(Env!=NULL);

  #define CH(x,d)  x+= d; std::cerr<<#x "= "<<x<<std::endl;
  switch(command)
  {
  case 'r':
  case 'R': Create(); break;
  case ' ': Running= !Running; break;
  case 'z': CH(TargetAngle,  0.01); break;
  case 'x': CH(TargetAngle, -0.01); break;
  case 'd': CH(TargetPourX, +0.01); break;
  case 'a': CH(TargetPourX, -0.01); break;
  case 'w': CH(TargetPourY, +0.01); break;
  case 's': CH(TargetPourY, -0.01); break;
  case 'e': CH(TargetPourZ, +0.01); break;
  case 'q': CH(TargetPourZ, -0.01); break;
  // case 'c': CH(TargetGripperPos, -0.01); break;
  // case 'v': CH(TargetGripperPos,  0.01); break;
  // case 'd': CH(TargetGripperX, +0.01); break;
  // case 'a': CH(TargetGripperX, -0.01); break;
  // case 'w': CH(TargetGripperY, +0.01); break;
  // case 's': CH(TargetGripperY, -0.01); break;
  // case 'e': CH(TargetGripperZ, +0.01); break;
  // case 'q': CH(TargetGripperZ, -0.01); break;
  // case '2': CH(TargetGripperTh, +0.01); break;
  // case '1': CH(TargetGripperTh, -0.01); break;
  case 'n':
    std::cerr<<"Input number of balls > ";
    std::cin>>BallNum;
    std::cerr<<"New number ("<<BallNum<<") is effective after reset"<<std::endl;
    break;
  case 'b':
    ++BallType;
    if(BallType>1)  BallType= 0;
    Create();
    break;
  case '/':
    Env->DumpBallStat("/tmp/ballstat.dat");
    std::cerr<<"Saved ball status into: /tmp/ballstat.dat"<<std::endl;
  }
  #undef CH
}
//-------------------------------------------------------------------------------------------

void Create()
{
  assert(Env!=NULL);
  TargetAngle= 0.0;
  TargetPourX= 0.0;
  TargetPourY= 0.0;
  TargetPourZ= 0.0;
  // TargetGripperPos= 0.0;
  // TargetGripperX= 0.0;
  // TargetGripperY= 0.0;
  // TargetGripperZ= 0.0;
  // TargetGripperTh= 0.0;
  Env->Create();
}
//-------------------------------------------------------------------------------------------

void Reset()
{
  Create();
}
//-------------------------------------------------------------------------------------------

void Run(int argc, char **argv, const char *texture_path)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &SimStart;
  fn.step = &SimLoop;
  fn.command = &SimKeyevent;
  fn.stop = 0;
  fn.path_to_textures = texture_path;

  dInitODE2(0);

  TEnvironment env;
  Env= &env;
  // env.Create();
  Create();

  dsSimulationLoop (argc,argv,500,400,&fn);

  dCloseODE();
}
//-------------------------------------------------------------------------------------------

void Stop()
{
  dsStop();
}
//-------------------------------------------------------------------------------------------


}  // end of ode_pour
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

