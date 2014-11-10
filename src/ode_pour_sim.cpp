//-------------------------------------------------------------------------------------------
/*! \file    ode_pour_sim.cpp
    \brief   Pouring simulator using ODE.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Nov.06, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/ode_pour_sim.h"
//-------------------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <iostream>
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

// int MAX_CONTACTS(10);  // maximum number of contact points per body
int MAX_CONTACTS(1);  // maximum number of contact points per body
double VIZ_JOINT_LEN(0.1);
double VIZ_JOINT_RAD(0.02);
double BALL_RAD(0.025);
double BOX_THICKNESS(0.02);
double TargetAngle(0.0);
double TimeStep(0.04);
bool Running(true);
void (*FlowCallback)(int num_src, int num_rcv, int num_flow, const double &z_rcv)= NULL;
void (*StepCallback)(const double &time, const double &time_step)= NULL;
//-------------------------------------------------------------------------------------------

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
          {0.0, 1.0, 1.0, 0.3}};
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
  for (std::vector<TNCSphere>::const_iterator itr(link_sp_.begin()),last(link_sp_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    dsDrawSphere (itr->getPosition(), itr->getRotation(), itr->getRadius());
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
    dsDrawCylinder (pos, rot, VIZ_JOINT_LEN,VIZ_JOINT_RAD);
  }
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TRobot1 : public TDynRobot
//===========================================================================================

/*override*/void TRobot1::Create(dWorldID world, dSpaceID space)
{
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
  double thickness= BOX_THICKNESS;
  double bottom_z= 0.0;
  double box_sizes[5][3]={
      {thickness, Size[1], Size[2]},
      {thickness, Size[1], Size[2]},
      {Size[0]-2.0*thickness, thickness, Size[2]},
      {Size[0]-2.0*thickness, thickness, Size[2]},
      {Size[0]-2.0*thickness, Size[1]-2.0*thickness, thickness}};
  double box_poss[5][3]={
      {0.5*Size[0]-0.5*thickness, 0.0, 0.5*Size[2]+bottom_z},
      {-0.5*Size[0]+0.5*thickness, 0.0, 0.5*Size[2]+bottom_z},
      {0.0, 0.5*Size[1]-0.5*thickness, 0.5*Size[2]+bottom_z},
      {0.0, -0.5*Size[1]+0.5*thickness, 0.5*Size[2]+bottom_z},
      {0.0, 0.0, 0.5*thickness+bottom_z}};

  for(int i(0); i<5; ++i)
  {
    link_b_[i].create(space, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[i].create(world);
    body_[i].setPosition(B[0]+box_poss[i][0], B[1]+box_poss[i][1], B[2]+box_poss[i][2]);
    dMass m;
    m.setBox(1.0, box_sizes[i][0], box_sizes[i][1], box_sizes[i][2]);
    body_[i].setMass(&m);
    link_b_[i].setBody(body_[i]);
    link_b_[i].ColorCode= 5;
  }

  // int dir(1);
  // dMatrix3 R;
  // dRSetIdentity (R);  // Z
  // if(dir==1) dRFromAxisAndAngle (R,0.0,1.0,0.0,0.5*M_PI);  // X
  // if(dir==2) dRFromAxisAndAngle (R,1.0,0.0,0.0,0.5*M_PI);  // Y
  // body_[0].setRotation (R);

  joint_f_.resize(4);
  for(int i(0); i<4; ++i)
  {
    joint_f_[i].create(world);
    joint_f_[i].attach(body_[4],body_[i]);
    joint_f_[i].set();
  }

  if(HasHinge)
  {
    joint_h_.resize(1);
    int bid= 3;  // Attaching to this body
    dReal fmax(1000.0);  // NOTE: set zero to control by joint torque
    joint_h_[0].create(world);
    joint_h_[0].attach(body_[bid],0);
    joint_h_[0].setAnchor(B[0]+box_poss[bid][0]+0.5*Size[0], B[1]+box_poss[bid][1], B[2]+box_poss[bid][2]+0.5*Size[2]);
    joint_h_[0].setAxis(0.0,1.0,0.0);
    joint_h_[0].setParam(dParamFMax,fmax);
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

  int N(100);
  double rad(BALL_RAD);
  double init_z(0.30);

  body_.resize(N);
  link_sp_.resize(N);

  for(int i(0); i<N; ++i)
  {
    double z= init_z + 0.3*rad*double(i);
    double xy_rad= 3.0*rad;
    double th= 0.73*M_PI*double(i);
    link_sp_[i].create(space, rad);
    body_[i].create(world);
    body_[i].setPosition(xy_rad*std::cos(th), xy_rad*std::sin(th), z);
    dMass m;
    m.setSphere(1.0, rad);
    body_[i].setMass(&m);
    link_sp_[i].setBody(body_[i]);
    link_sp_[i].ColorCode= 1;
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TEnvironment
//===========================================================================================

void TEnvironment::Create()
{
  contactgroup_.create ();
  world_.setGravity (0,0,-0.5);
  dWorldSetCFM (world_.id(),1e-5);
  plane_.create (space_,0,0,1,0);

  source_.B[0]= 0.0;  source_.B[1]= 0.0;  source_.B[2]= 0.2;
  source_.Size[0]= 0.3;  source_.Size[1]= 0.3;  source_.Size[2]= 0.4;
  source_.HasHinge= true;
  source_.Create(world_,space_);
  receiver_.B[0]= 0.32;  receiver_.B[1]= 0.0;  receiver_.B[2]= 0.0;
  receiver_.Size[0]= 0.3;  receiver_.Size[1]= 0.4;  receiver_.Size[2]= 0.4;
  receiver_.HasHinge= false;
  receiver_.Create(world_,space_);
  balls_.Create(world_,space_);
  // geom_.Create(world_,space_);

  time_= 0.0;
}
//-------------------------------------------------------------------------------------------

void TEnvironment::StepSim(const double &time_step)
{
  ControlCallback(time_step);

  space_.collide (0,&NearCallback);
  world_.step (time_step);
  time_+= time_step;

  contactgroup_.empty();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::Draw()
{
  if(Running)  DrawCallback();

  source_.Draw();
  receiver_.Draw();
  balls_.Draw();
  // geom_.Draw();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::ControlCallback(const double &time_step)
{
  // static double angle = 0;
  // angle += 0.01;
  // robot_.Body().back().addForce (0.1*(std::sin(angle)+1.0), 0.1*(std::sin(angle*1.7)+1.0) ,0.1*(std::sin(angle*0.7)+1.0));
  // robot_.Body().back().addForce (0,0,1.0*(std::sin(angle)+1.0));
  // robot_.Body().front().addForce (0,0,0.01*(std::cos(angle)+1.0));
  dReal Kp(10.0);
  source_.SetVelH(0, Kp*(TargetAngle-source_.GetAngleH(0)));
}
//-------------------------------------------------------------------------------------------

void TEnvironment::DrawCallback()
{
  std::vector<TNCBody> &balls_b(balls_.BallsB());
  std::vector<TNCSphere> &balls_g(balls_.BallsG());
  int num_src(0), num_rcv(0), num_flow(0);
  double speed;
  std::list<double> z_rcv_data;
  for(size_t i(0); i<balls_b.size(); ++i)
  {
    const dReal *pos(balls_b[i].getPosition());
    const dReal *vel(balls_b[i].getLinearVel());
    dReal angle= std::atan2(-(pos[0]-0.15),-(pos[2]-0.6));
    speed= std::sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);
    if(angle<0.0)  angle+= 2.0*M_PI;
    dReal angle_base= TargetAngle+0.5*M_PI;
    if(pos[0]>0.15 && pos[2]<0.4 && speed<0.2)
    {
      balls_g[i].ColorCode= 0;
      ++num_rcv;
      if(speed<0.02)  InsertDescending(z_rcv_data, pos[2]-(BOX_THICKNESS+0.5*BALL_RAD));
    }
    else if(angle>angle_base)
    {
      balls_g[i].ColorCode= 2;
      ++num_flow;
    }
    else
    {
      balls_g[i].ColorCode= 1;
      ++num_src;
    }
    if(speed>0.1)  balls_g[i].ColorCode+= 6;
  }
  // std::cerr<<"angle= "<<angle<<"  ";
  double z_rcv(0.0);
  int num_big= std::min(int(z_rcv_data.size()), 10);
  std::list<double>::iterator z_rcv_itr(z_rcv_data.begin());
  for(int i(0); i<num_big; ++z_rcv_itr,++i)  z_rcv+= *z_rcv_itr;
  if(num_big>0)  z_rcv/= double(num_big);
  double amount= z_rcv;
  // double amount= 0.0002*double(num_rcv);
  std::cerr<<"#src, #flow, #rcv, amount= "<<num_src<<", "<<num_flow<<", "<<num_rcv<<", "<<amount/*<<", "<<speed*/<<std::endl;
  if(FlowCallback!=NULL)  FlowCallback(num_src, num_rcv, num_flow, z_rcv);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


static void NearCallback(void *data, dGeomID o1, dGeomID o2)
{
  assert(Env!=NULL);

  // do nothing if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected (b1,b2)) return;

  std::valarray<dContact> contact(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per link
  for (int i=0; i<MAX_CONTACTS; i++)
  {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = 0.001; // dInfinity;
    contact[i].surface.mu2 = 0.1;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.01;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc=dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact)))
  {
    for (int i=0; i<numc; i++)
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

  switch(command)
  {
  case 'r':
  case 'R': Create(); break;
  case ' ': Running= !Running; break;
  case 'z': TargetAngle+= 0.01; std::cerr<<"TargetAngle= "<<TargetAngle<<std::endl; break;
  case 'x': TargetAngle-= 0.01; std::cerr<<"TargetAngle= "<<TargetAngle<<std::endl; break;
  }
}
//-------------------------------------------------------------------------------------------

void Create()
{
  assert(Env!=NULL);
  TargetAngle= 0.0;
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

