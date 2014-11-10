//-------------------------------------------------------------------------------------------
/*! \file    ode_pour_sim_node.cpp
    \brief   Pouring simulator using ODE.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Nov.06, 2014
*/
//-------------------------------------------------------------------------------------------
#include "pr2_lfd_vision/ode_pour_sim.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

ros::Publisher *PRatioPub(NULL), *PFlowPub(NULL);
int NumSrc(0);

void OnFlowCallback(int num_src, int num_rcv, int num_flow, const double &z_rcv)
{
  NumSrc= num_src;
  std_msgs::Float64 ratio_msg;
  ratio_msg.data= 5.0*z_rcv;
  PRatioPub->publish(ratio_msg);
  // std::cerr<<"#src, #flow, #rcv, amount= "<<num_src<<", "<<num_flow<<", "<<num_rcv<<", "<<z_rcv<<std::endl;
  std_msgs::Float64MultiArray flow_msg;
  flow_msg.data.resize(2);
  flow_msg.data[0]= 40.0*static_cast<double>(num_flow);
  flow_msg.data[1]= M_PI*0.5;
  PFlowPub->publish(flow_msg);
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

bool ResetSim(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resetting simulator..."<<std::endl;
  NumSrc= 0;
  ode_pour::Reset();
  // usleep(50*1000);
  // while(NumSrc<100)
  // {
    // std::cerr<<"NumSrc= "<<NumSrc<<std::endl;
    // ros::spinOnce();
    // usleep(50*1000);
  // }
  std::cerr<<"done."<<std::endl;
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
  ros::init(argc, argv, "ode_pour_sim");
  ros::NodeHandle node("~");

  std::string texture_path;
  node.param("texture_path",texture_path,std::string("config/textures"));

  ros::Publisher ratio_pub, flow_pub;
  ratio_pub= node.advertise<std_msgs::Float64>("/color_occupied_ratio", 1);
  PRatioPub= &ratio_pub;
  flow_pub= node.advertise<std_msgs::Float64MultiArray>("/flow_speed_angle", 1);
  PFlowPub= &flow_pub;

  ros::Subscriber sub_theta= node.subscribe("theta", 1, &ThetaCallback);
  ros::ServiceServer srv_reset= node.advertiseService("reset", &ResetSim);

  ode_pour::FlowCallback= &OnFlowCallback;
  ode_pour::StepCallback= &OnStepCallback;
  ode_pour::Run(argc, argv, texture_path.c_str());

  return 0;
}
//-------------------------------------------------------------------------------------------
