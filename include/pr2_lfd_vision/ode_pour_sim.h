//-------------------------------------------------------------------------------------------
/*! \file    ode_pour_sim.h
    \brief   Pouring simulator using ODE.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Nov.06, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef ode_pour_sim_h
#define ode_pour_sim_h
//-------------------------------------------------------------------------------------------
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawTriangle dsDrawTriangleD
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

extern int MAX_CONTACTS;  // maximum number of contact points per body
extern double VIZ_JOINT_LEN;
extern double VIZ_JOINT_RAD;
extern int    BALL_NUM;
extern int    BALL_TYPE;  // 0: Sphere, 1: Box
extern double BALL_RAD;
extern double BALL_BOX_RATIO;  // If BALL_TYPE is box, its size = BALL_RAD*this value
extern double BOX_THICKNESS;
extern double TargetAngle;
extern double TimeStep;
extern bool Running;
extern void (*FlowCallback)(int num_src, int num_rcv, int num_flow, const double &z_rcv);
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
DEF_NC(FixedJoint,3)
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
  std::vector<TNCFixedJoint> joint_f_;
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TRobot1 : public TDynRobot
//===========================================================================================
{
public:
  double B[3], Size[3];
  bool   HasHinge;
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
      switch(BALL_TYPE)
      {
      case 0: return link_sp_[idx].ColorCode;
      case 1: return link_b_[idx].ColorCode;
      }
      return -1;
    }
  void SetBallCol(int idx, int col)
    {
      switch(BALL_TYPE)
      {
      case 0: link_sp_[idx].ColorCode= col;  break;
      case 1: link_b_[idx].ColorCode= col;  break;
      }
    }
  std::vector<TNCBody>& BallsB()  {return body_;}
  const std::vector<TNCBody>& BallsB() const {return body_;}
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
  void DrawCallback();

private:
  dWorld world_;
  dSimpleSpace space_;
  dJointGroup contactgroup_;

  TRobot1   source_;
  TRobot1   receiver_;
  TBalls1   balls_;
  // TGeom1    geom_;
  dPlane    plane_;

  dReal time_;
};
//-------------------------------------------------------------------------------------------


void Create();
void Reset();
void Run(int argc, char **argv, const char *texture_path="textures");
void Stop();
//-------------------------------------------------------------------------------------------


}  // end of ode_pour
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // ode_pour_sim_h
//-------------------------------------------------------------------------------------------
