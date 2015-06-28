//-------------------------------------------------------------------------------------------
/*! \file    ode_pour_sim_node.cpp
    \brief   Grasping and pouring simulator using ODE.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.18, 2015
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/ode_grpour_sim.h"
#include "pr2_lfd_vision/ODEConfig.h"
#include "pr2_lfd_vision/ODESensor.h"
#include "pr2_lfd_vision/ODEViz.h"
#include "pr2_lfd_vision/ODEVizPrimitive.h"
#include "pr2_lfd_vision/ODEGetConfig.h"
#include "pr2_lfd_vision/ODEReset2.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

ros::Publisher *PRatioPub(NULL);
ros::Publisher *PFlowPub(NULL);
ros::Publisher *PSensorsPub(NULL);

std::vector<pr2_lfd_vision::ODEVizPrimitive> VizObjs;

void OnSensingCallback(const ode_pour::TSensors1 &sensors)
{
  int num_rcv(sensors.NumRcv); int num_flow(sensors.NumFlow);
  const double &z_rcv(sensors.ZRcv);
  std_msgs::Float64 ratio_msg;
  // ratio_msg.data= 5.0*z_rcv;
  /*TEST*/ratio_msg.data= 0.0055*static_cast<double>(num_rcv);
  PRatioPub->publish(ratio_msg);
  std_msgs::Float64MultiArray flow_msg;
  flow_msg.data.resize(2);
  flow_msg.data[0]= 40.0*static_cast<double>(num_flow);
  flow_msg.data[1]= M_PI*0.5;
  PFlowPub->publish(flow_msg);

  // Copy sensors to sensor message
  pr2_lfd_vision::ODESensor sensors_msg;
  sensors_msg.ball_st.resize(sensors.BallSt.size());
  for(int i(0),iend(sensors.BallSt.size());i<iend;++i)
    sensors_msg.ball_st[i]= sensors.BallSt[i];
  sensors_msg.ball_x.resize(sensors.BallX.size()*6);
  for(int i(0),iend(sensors.BallX.size());i<iend;++i)
    for(int d(0);d<6;++d)
      sensors_msg.ball_x[6*i+d]= sensors.BallX[i].X[d];
  sensors_msg.ball_colliding.resize(sensors.BallColliding.size());
  for(int i(0),iend(sensors.BallColliding.size());i<iend;++i)
    sensors_msg.ball_colliding[i]= sensors.BallColliding[i];
  sensors_msg.num_src              = sensors.NumSrc;
  sensors_msg.num_rcv              = sensors.NumRcv;
  sensors_msg.num_flow             = sensors.NumFlow;
  sensors_msg.num_spill            = sensors.NumSpill;
  sensors_msg.num_bounce           = sensors.NumBounce;
  sensors_msg.z_rcv                = sensors.ZRcv;

  XToGPose(sensors.XSrc, sensors_msg.x_src);
  XToGPose(sensors.XRcv, sensors_msg.x_rcv);
  sensors_msg.p_pour.resize(3);
  for(int d(0);d<3;++d) sensors_msg.p_pour[d]= sensors.PPour[d];
  sensors_msg.theta= sensors.Theta;

  sensors_msg.gripper_colliding    = sensors.GripperColliding;
  sensors_msg.src_colliding        = sensors.SrcColliding;
  sensors_msg.time                 = sensors.Time;
  sensors_msg.on_init              = sensors.OnInit;
  PSensorsPub->publish(sensors_msg);
}
//-------------------------------------------------------------------------------------------

void OnDrawCallback()
{
  dReal x[7]={0.0,0.0,0.0, 1.0,0.0,0.0,0.0};
  dReal sides[4]={0.0,0.0,0.0,0.0};
  dMatrix3 R;
  dVector3 p;
  for(std::vector<pr2_lfd_vision::ODEVizPrimitive>::const_iterator
      itr(VizObjs.begin()),itr_e(VizObjs.end()); itr!=itr_e; ++itr)
  {
    dsSetColorAlpha(itr->color.r,itr->color.g,itr->color.b,itr->color.a);
    GPoseToX(itr->pose, x);
    dReal q[4]= {x[6],x[3],x[4],x[5]};
    switch(itr->type)
    {
    case pr2_lfd_vision::ODEVizPrimitive::LINE:
      p[0]= x[0]+itr->param[0];
      p[1]= x[1]+itr->param[1];
      p[2]= x[2]+itr->param[2];
      dsDrawLine(x, p);
      break;
    case pr2_lfd_vision::ODEVizPrimitive::SPHERE:
      dRfromQ(R,q);
      dsDrawSphere(x, R, /*rad=*/itr->param[0]);
      break;
    case pr2_lfd_vision::ODEVizPrimitive::CYLINDER:
      dRfromQ(R,q);
      dsDrawCylinder(x, R, /*len=*/itr->param[1], /*rad=*/itr->param[0]);
      break;
    case pr2_lfd_vision::ODEVizPrimitive::CUBE:
      dRfromQ(R,q);
      sides[0]= itr->param[0];
      sides[1]= itr->param[1];
      sides[2]= itr->param[2];
      dsDrawBox(x, R, sides);
      break;
    default:
      std::cerr<<"Unknown type:"<<itr->type<<std::endl;
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------

void OnStepCallback(const double &time, const double &time_step)
{
  if(ode_pour::Running)
    std::cerr<<"@"<<time<<std::endl;
  else
    usleep(500*1000);
  ros::spinOnce();
  if(!ros::ok())  ode_pour::Stop();
}
//-------------------------------------------------------------------------------------------

void ThetaCallback(const std_msgs::Float64 &msg)
{
  ode_pour::TargetAngle= msg.data;
}
//-------------------------------------------------------------------------------------------

void PPourCallback(const std_msgs::Float64MultiArray &msg)
{
  if(msg.data.size()!=3)
  {
    std::cerr<<"PPourCallback: Bad target: "<<msg<<std::endl;
    return;
  }
  ode_pour::TargetPourX= msg.data[0];
  ode_pour::TargetPourY= msg.data[1];
  ode_pour::TargetPourZ= msg.data[2];
}
//-------------------------------------------------------------------------------------------

void ODEVizCallback(const pr2_lfd_vision::ODEViz &msg)
{
  VizObjs= msg.objects;
}
//-------------------------------------------------------------------------------------------

bool GetConfig(pr2_lfd_vision::ODEGetConfig::Request &req, pr2_lfd_vision::ODEGetConfig::Response &res)
{
  using namespace ode_pour;
  pr2_lfd_vision::ODEConfig &msg(res.config);
  msg.RcvPos.resize(3);
  msg.RcvSize.resize(3);
  msg.MaxContacts       = MaxContacts      ;
  msg.BallNum           = BallNum          ;
  msg.BallType          = BallType         ;
  msg.BallRad           = BallRad          ;
  msg.BallBoxRatio      = BallBoxRatio     ;
  msg.BoxThickness      = BoxThickness     ;
  msg.RcvPos[0]         = RcvPos[0]        ;
  msg.RcvPos[1]         = RcvPos[1]        ;
  msg.RcvPos[2]         = RcvPos[2]        ;
  msg.RcvSize[0]        = RcvSize[0]       ;
  msg.RcvSize[1]        = RcvSize[1]       ;
  msg.RcvSize[2]        = RcvSize[2]       ;
  msg.ContactBounce     = ContactBounce    ;
  msg.ContactBounceVel  = ContactBounceVel ;
  msg.ContactSoftCFM    = ContactSoftCFM   ;
  msg.GripperHeight     = GripperHeight    ;
  msg.TimeStep          = TimeStep         ;
  msg.Gravity           = Gravity          ;
  return true;
}
//-------------------------------------------------------------------------------------------

bool ResetSim2(pr2_lfd_vision::ODEReset2::Request &req, pr2_lfd_vision::ODEReset2::Response &res)
{
  using namespace ode_pour;
  std::cerr<<"Resetting simulator..."<<std::endl;
  const pr2_lfd_vision::ODEConfig &msg(req.config);
  MaxContacts       = msg.MaxContacts      ;
  BallNum           = msg.BallNum          ;
  BallType          = msg.BallType         ;
  BallRad           = msg.BallRad          ;
  BallBoxRatio      = msg.BallBoxRatio     ;
  BoxThickness      = msg.BoxThickness     ;
  RcvPos[0]         = msg.RcvPos[0]        ;
  RcvPos[1]         = msg.RcvPos[1]        ;
  RcvPos[2]         = msg.RcvPos[2]        ;
  RcvSize[0]        = msg.RcvSize[0]       ;
  RcvSize[1]        = msg.RcvSize[1]       ;
  RcvSize[2]        = msg.RcvSize[2]       ;
  ContactBounce     = msg.ContactBounce    ;
  ContactBounceVel  = msg.ContactBounceVel ;
  ContactSoftCFM    = msg.ContactSoftCFM   ;
  GripperHeight     = msg.GripperHeight    ;
  TimeStep          = msg.TimeStep         ;
  Gravity           = msg.Gravity          ;
  ode_pour::Reset();
  std::cerr<<"done."<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

bool ResetSim(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resetting simulator..."<<std::endl;
  ode_pour::Reset();
  std::cerr<<"done."<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Simulator paused..."<<std::endl;
  ode_pour::Running= false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Simulator resumed..."<<std::endl;
  ode_pour::Running= true;
  return true;
}
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "ode_grpour_sim");
  ros::NodeHandle node("~");

  std::string texture_path;
  int winx(500), winy(400);
  node.param("texture_path",texture_path,std::string("config/textures"));
  node.param("winx",winx,winx);
  node.param("winy",winy,winy);

  ros::Publisher ratio_pub= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);
  PRatioPub= &ratio_pub;
  ros::Publisher flow_pub= node.advertise<std_msgs::Float64MultiArray>("/flow_speed_angle", 1);
  PFlowPub= &flow_pub;
  ros::Publisher sensors_pub= node.advertise<pr2_lfd_vision::ODESensor>("sensors", 1);
  PSensorsPub= &sensors_pub;

  ros::Subscriber sub_theta= node.subscribe("theta", 1, &ThetaCallback);
  ros::Subscriber sub_ppour= node.subscribe("ppour", 1, &PPourCallback);
  ros::Subscriber sub_viz= node.subscribe("viz", 1, &ODEVizCallback);
  ros::ServiceServer srv_get_config= node.advertiseService("get_config", &GetConfig);
  ros::ServiceServer srv_reset= node.advertiseService("reset", &ResetSim);
  ros::ServiceServer srv_reset2= node.advertiseService("reset2", &ResetSim2);
  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);

  ode_pour::SensingCallback= &OnSensingCallback;
  ode_pour::DrawCallback= &OnDrawCallback;
  ode_pour::StepCallback= &OnStepCallback;
  ode_pour::Run(argc, argv, texture_path.c_str(), winx, winy);

  return 0;
}
//-------------------------------------------------------------------------------------------
