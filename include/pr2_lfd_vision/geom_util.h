//-------------------------------------------------------------------------------------------
/*! \file    geom_util.h
    \brief   Geometry utilities
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.18, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef geom_util_h
#define geom_util_h
//-------------------------------------------------------------------------------------------
#include <Eigen/Core>
#include <Eigen/Geometry>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline t_value Sq(const t_value &val)
{
  return val*val;
}
//-------------------------------------------------------------------------------------------

/* Pose to Eigen::Affine3d.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline Eigen::Affine3d XToEigMat(const t_value x[7])
{
  Eigen::Affine3d res;
  res= Eigen::Translation3d(x[0],x[1],x[2])
          * Eigen::Quaterniond(x[6], x[3],x[4],x[5]);
  return res;
}
//-------------------------------------------------------------------------------------------

/* Orientation to Eigen::Quaterniond.
    x: [0-3]: orientation x,y,z,w. */
template <typename t_value>
inline Eigen::Quaterniond QToEigMat(const t_value x[4])
{
  return Eigen::Quaterniond(x[3], x[0],x[1],x[2]);
}
//-------------------------------------------------------------------------------------------

/* Eigen::Affine3d to position.
    x: [0-2]: position x,y,z. */
template <typename t_value>
inline void EigMatToP(const Eigen::Affine3d T, t_value x[3])
{
  const Eigen::Vector3d p= T.translation();
  x[0]= p[0];
  x[1]= p[1];
  x[2]= p[2];
}
//-------------------------------------------------------------------------------------------

/* Eigen::Quaterniond to orientation.
    x: [0-3]: orientation x,y,z,w. */
template <typename t_value>
inline void EigMatToQ(const Eigen::Quaterniond q, t_value x[4])
{
  x[0]= q.x();
  x[1]= q.y();
  x[2]= q.z();
  x[3]= q.w();
}
/* Eigen::Affine3d to orientation.
    x: [0-3]: orientation x,y,z,w. */
template <typename t_value>
inline void EigMatToQ(const Eigen::Affine3d T, t_value x[4])
{
  EigMatToQ(T.rotation(), x);
}
//-------------------------------------------------------------------------------------------

/* Eigen::Affine3d to pose.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void EigMatToX(const Eigen::Affine3d T, t_value x[7])
{
  const Eigen::Vector3d p= T.translation();
  x[0]= p[0];
  x[1]= p[1];
  x[2]= p[2];
  Eigen::Quaterniond q(T.rotation());
  x[3]= q.x();
  x[4]= q.y();
  x[5]= q.z();
  x[6]= q.w();
}
//-------------------------------------------------------------------------------------------

/* Compute xout = x2 * x1
    where x1,x2,xout: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void TransformX(const t_value x2[7], const t_value x1[7], t_value xout[7])
{
  EigMatToX(XToEigMat(x2)*XToEigMat(x1), xout);
}
//-------------------------------------------------------------------------------------------

/* Compute xout = x2 * p1
    where p1,xout: [0-2]: position x,y,z,
      x2: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void TransformP(const t_value x2[7], const t_value p1[3], t_value xout[3])
{
  EigMatToP(XToEigMat(x2)*Eigen::Translation3d(p1[0],p1[1],p1[2]), xout);
}
//-------------------------------------------------------------------------------------------

/* Rotate q_in with quaternion(angle,axis) then store to q_out.
    where q_in,q_out: [0-3]: orientation x,y,z,w,
      axis: [0-2]: axis x,y,z. */
template <typename t_value>
inline void RotateAngleAxis(const t_value &angle, const t_value axis[3], const t_value q_in[4], t_value q_out[4])
{
  Eigen::Quaterniond q= Eigen::AngleAxisd(angle, Eigen::Vector3d(axis)) * QToEigMat(q_in);
  EigMatToQ(q, q_out);
}
//-------------------------------------------------------------------------------------------

// Convert geometry_msgs/Point to p; usually, t_point==geometry_msgs::Point
template <typename t_point, typename t_value>
inline void GPointToP(const t_point &point, t_value p[7])
{
  p[0]= point.x;
  p[1]= point.y;
  p[2]= point.z;
}
//-------------------------------------------------------------------------------------------


// Convert x to geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_value, typename t_pose>
inline void XToGPose(const t_value x[7], t_pose &pose)
{
  pose.position.x= x[0];
  pose.position.y= x[1];
  pose.position.z= x[2];
  pose.orientation.x= x[3];
  pose.orientation.y= x[4];
  pose.orientation.z= x[5];
  pose.orientation.w= x[6];
}
// Convert x to geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_pose, typename t_value>
inline t_pose XToGPose(const t_value x[7])
{
  t_pose pose;
  XToGPose<t_value,t_pose>(x, pose);
  return pose;
}
//-------------------------------------------------------------------------------------------

// Convert geometry_msgs/Pose to x; usually, t_pose==geometry_msgs::Pose
template <typename t_pose, typename t_value>
inline void GPoseToX(const t_pose &pose, t_value x[7])
{
  x[0]= pose.position.x;
  x[1]= pose.position.y;
  x[2]= pose.position.z;
  x[3]= pose.orientation.x;
  x[4]= pose.orientation.y;
  x[5]= pose.orientation.z;
  x[6]= pose.orientation.w;
}
//-------------------------------------------------------------------------------------------

// Visualize a normal vector with RGB color.
template <typename t_1, typename t_2>
inline void GetVisualNormal(
    const t_1 &nx, const t_1 &ny, const t_1 &nz,
    t_2 &r, t_2 &g, t_2 &b)
{
  r= 0.5*(1.0-nx);
  g= 0.5*(1.0-ny);
  b= 0.5*(1.0-nz);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // geom_util_h
//-------------------------------------------------------------------------------------------
