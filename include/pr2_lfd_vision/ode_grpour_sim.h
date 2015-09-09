//-------------------------------------------------------------------------------------------
/*! \file    ode_pour_sim.h
    \brief   Grasping and pouring simulator using ODE.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.18, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef ode_pour_sim_h
#define ode_pour_sim_h
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/geom_util.h"
//-------------------------------------------------------------------------------------------
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawTriangle dsDrawTriangleD
#define dsDrawLine dsDrawLineD
#endif
//-------------------------------------------------------------------------------------------
#include <vector>
#include <valarray>
//-------------------------------------------------------------------------------------------
#ifndef override
#define override
#endif
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace ode_pour
{
struct TSensors1;

// extern int MaxContacts;  // maximum number of contact points per body
extern int MaxContacts;  // maximum number of contact points per body
extern double VizJointLen;
extern double VizJointRad;
extern int    BallNum;
extern int    BallType;  // 0: Sphere, 1: Box
extern double BallRad;
extern double BallBoxRatio;  // If BallType is box, its size = BallRad*this value
extern double BoxThickness;
// extern double GripThickness;
extern double RcvPos[3];
extern double RcvSize[3];
extern double ContactBounce;
extern double ContactBounceVel;
extern double ContactSoftCFM;
extern double GripperHeight;
extern double TargetAngle;
extern double TargetPourX,TargetPourY,TargetPourZ;
// extern double TargetGripperPos;
// extern double TargetGripperX,TargetGripperY,TargetGripperZ,TargetGripperTh;
extern double TimeStep;
extern double Gravity;
extern bool Running;
extern void (*SensingCallback)(const TSensors1 &sensors);
extern void (*DrawCallback)(void);
extern void (*StepCallback)(const double &time, const double &time_step);
//-------------------------------------------------------------------------------------------

// the following classes have a copy constructor and operator= that do nothing;
// these classes are defined in order to use std::vector of them
#define DEF_NC(x_class, x_col) \
  class TNC##x_class : public d##x_class  \
  {                                       \
  public:                                 \
    int ColorCode;                        \
    TNC##x_class() : d##x_class(), ColorCode(x_col) {} \
    TNC##x_class(const TNC##x_class&)     \
      : d##x_class(), ColorCode(x_col){}  \
    const TNC##x_class& operator=(const TNC##x_class&) {return *this;} \
  private:                                \
  };
DEF_NC(Body,0)
DEF_NC(BallJoint,0)
DEF_NC(HingeJoint,1)
DEF_NC(Hinge2Joint,2)
DEF_NC(SliderJoint,3)
DEF_NC(FixedJoint,4)
DEF_NC(Box,0)
DEF_NC(Capsule,1)
DEF_NC(Cylinder,2)
DEF_NC(Sphere,3)
#undef DEF_NC
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TTriMeshGeom : public dGeom
//===========================================================================================
{
public:
  int ColorCode;
  TTriMeshGeom() : dGeom(), ColorCode(4) {}

  TTriMeshGeom(const TTriMeshGeom&) : dGeom(), ColorCode(4) {}
  const TTriMeshGeom& operator=(const TTriMeshGeom&) {return *this;}

  dGeomID& create() {return _id;}

  void create(dSpaceID space)
    {
      dTriMeshDataID new_tmdata= dGeomTriMeshDataCreate();
      dGeomTriMeshDataBuildSingle(new_tmdata, &vertices_[0], 3*sizeof(float), vertices_.size()/3,
                  &indices_[0], indices_.size(), 3*sizeof(dTriIndex));
      _id= dCreateTriMesh(space, new_tmdata, NULL, NULL, NULL);
    }
  void setBody(dBodyID body)  {dGeomSetBody (_id,body);}

  const std::valarray<float>& getVertices() const {return vertices_;}
  const std::valarray<dTriIndex>& getIndices() const {return indices_;}

  std::valarray<float>& setVertices() {return vertices_;}
  std::valarray<dTriIndex>& setIndices() {return indices_;}

  void setVertices(float *array, int size)
    {
      vertices_.resize(size);
      for(float *itr(&vertices_[0]);size>0;--size,++itr,++array)
        *itr= *array;
    }
  void setIndices(dTriIndex *array, int size)
    {
      indices_.resize(size);
      for(dTriIndex *itr(&indices_[0]);size>0;--size,++itr,++array)
        *itr= *array;
    }

  void getMass(dMass &m, dReal density)
    {
      dMassSetTrimesh(&m, density, _id);
      printf("mass at %f %f %f\n", m.c[0], m.c[1], m.c[2]);
      dGeomSetPosition(_id, -m.c[0], -m.c[1], -m.c[2]);
      dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    }

protected:

  std::valarray<float> vertices_;
  std::valarray<dTriIndex> indices_;

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TDynRobot
//===========================================================================================
{
public:

  std::vector<TNCBody>& Body() {return body_;}
  dBody& Body(int j) {return body_[j];}

  virtual void Create(dWorldID world, dSpaceID space) = 0;
  void Draw();

  const dReal GetAngleH(int j) const {return joint_h_[j].getAngle();}
  const dReal GetAngVelH(int j) const {return joint_h_[j].getAngleRate();}
  void SetVelH(int j, dReal vel)  {joint_h_[j].setParam(dParamVel,vel);}

  const dReal GetPosS(int j) const {return joint_s_[j].getPosition();}
  const dReal GetVelS(int j) const {return joint_s_[j].getPositionRate();}
  void SetVelS(int j, dReal vel)  {joint_s_[j].setParam(dParamVel,vel);}

protected:
  std::vector<TNCBody> body_;
  std::vector<TNCBox> link_b_;
  std::vector<TNCCapsule> link_ca_;
  std::vector<TNCCylinder> link_cy_;
  std::vector<TNCSphere> link_sp_;
  std::vector<TTriMeshGeom> link_tm_;
  std::vector<TNCBallJoint> joint_b_;
  std::vector<TNCHingeJoint> joint_h_;
  std::vector<TNCHinge2Joint> joint_h2_;
  std::vector<TNCSliderJoint> joint_s_;
  std::vector<TNCFixedJoint> joint_f_;
};
//-------------------------------------------------------------------------------------------

#if 0
//===========================================================================================
class TGripper1 : public TDynRobot
//===========================================================================================
{
public:
  override void Create(dWorldID world, dSpaceID space);
};
//-------------------------------------------------------------------------------------------
#endif

//===========================================================================================
class TSrcContainer1 : public TDynRobot
//===========================================================================================
{
public:
  override void Create(dWorldID world, dSpaceID space);
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TRcvContainer1 : public TDynRobot
//===========================================================================================
{
public:
  override void Create(dWorldID world, dSpaceID space);
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TBalls1 : public TDynRobot
//===========================================================================================
{
public:
  override void Create(dWorldID world, dSpaceID space);
  // std::vector<TNCSphere>& BallsG()  {return link_sp_;}
  // const std::vector<TNCSphere>& BallsG() const {return link_sp_;}
  // std::vector<TNCBox>& BallsG()  {return link_b_;}
  // const std::vector<TNCBox>& BallsG() const {return link_b_;}
  int BallCol(int idx) const
    {
      switch(BallType)
      {
      case 0: return link_sp_[idx].ColorCode;
      case 1: return link_b_[idx].ColorCode;
      }
      return -1;
    }
  void SetBallCol(int idx, int col)
    {
      switch(BallType)
      {
      case 0: link_sp_[idx].ColorCode= col;  break;
      case 1: link_b_[idx].ColorCode= col;  break;
      }
    }
  std::vector<TNCBody>& BallsB()  {return body_;}
  const std::vector<TNCBody>& BallsB() const {return body_;}
};
//-------------------------------------------------------------------------------------------

struct TSensors1
{
  std::vector<int> BallSt;  // Ball status kinds (1:in src, 2:in rcv, 3:flow, 4:spill, 5:bounce, 0:unknown)
  std::vector<TPosVel> BallX;  // Ball positions and velocities
  std::vector<int> BallColliding;  // Ball colliding (0: not colliding, 1: colliding, 2: colliding with ground)
  int NumSrc, NumRcv, NumFlow, NumSpill, NumBounce;  // Number of balls in each status kind
  double ZRcv;

  double XSrc[7], XRcv[7];  // Poses (x,y,z,qx,qy,qz,qw) of containers
  double PPour[3];  // Position of pouring point on source container
  double Theta;  // Orientation of pouring point

  bool GripperColliding;
  bool SrcColliding;  // If source container is colliding except for balls

  double Time;  // Simulation time
  bool   OnInit;  // True during initializing

  void Clear()
    {
      BallSt.clear();
      BallX.clear();
      BallColliding.clear();
      SetZeros(-1);
    }

  void SetZeros(int num_balls)
    {
      if(num_balls>=0)
      {
        BallSt.resize(num_balls);
        BallX.resize(num_balls);
        BallColliding.resize(num_balls);
      }
      for(std::vector<int>::iterator itr(BallSt.begin()),itr_end(BallSt.end());
          itr!=itr_end; ++itr)
        *itr= 0;
      for(std::vector<TPosVel>::iterator itr(BallX.begin()),itr_end(BallX.end());
          itr!=itr_end; ++itr)
        for(int d(0);d<6;++d)  itr->X[d]= 0.0;
      for(std::vector<int>::iterator itr(BallColliding.begin()),itr_end(BallColliding.end());
          itr!=itr_end; ++itr)
        *itr= 0;
      NumSrc= 0; NumRcv= 0; NumFlow= 0; NumSpill= 0; NumBounce= 0;
      ZRcv= 0.0;
      for(int d(0);d<7;++d) XSrc[d]=0.0;  XSrc[6]=1.0;
      for(int d(0);d<7;++d) XRcv[d]=0.0;  XRcv[6]=1.0;
      for(int d(0);d<3;++d) PPour[d]=0.0;
      Theta= 0.0;
      GripperColliding= false;
      SrcColliding= false;
      Time= 0.0;
      OnInit= true;
    }

  // Reset values for each new physics computation step.
  // Only collision flags are reset. Other values are kept.
  void ResetForStep()
    {
      for(std::vector<int>::iterator itr(BallColliding.begin()),itr_end(BallColliding.end());
          itr!=itr_end; ++itr)
        *itr= 0;
      GripperColliding= false;
      SrcColliding= false;
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TEnvironment
//===========================================================================================
{
public:
  TEnvironment()
    : space_(0) {}

  dWorldID WorldID() {return world_.id();}
  dSpaceID SpaceID() {return space_.id();}
  dJointGroupID ContactGroupID() {return contactgroup_.id();}

  // TDynRobot& Robot() {return robot_;}

  const dReal& Time() const {return time_;}

  void Create();
  void StepSim(const double &time_step);
  void Draw();

  void ControlCallback(const double &time_step);
  void EDrawCallback();
  /* Called when b1 and b2 are colliding.
      Return whether we ignore this collision (true: ignore collision). */
  bool CollisionCallback(dBodyID &b1, dBodyID &b2, dContact *contact);

  // Save ball state into file
  void DumpBallStat(const char *file_name);

private:
  dWorld world_;
  dSimpleSpace space_;
  dJointGroup contactgroup_;

  // TGripper1        gripper_;
  TSrcContainer1   source_;
  TRcvContainer1   receiver_;
  TBalls1          balls_;
  // TGeom1    geom_;
  dPlane    plane_;

  dReal time_;

  TSensors1  sensors_;
};
//-------------------------------------------------------------------------------------------


void Create();
void Reset();
void Run(int argc, char **argv, const char *texture_path="textures", int winx=500, int winy=400);
void Stop();
//-------------------------------------------------------------------------------------------


}  // end of ode_pour
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // ode_pour_sim_h
//-------------------------------------------------------------------------------------------
