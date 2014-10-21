//-------------------------------------------------------------------------------------------
/*! \file    pcl_util.h
    \brief   PCL utility
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.17, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef pcl_util_h
#define pcl_util_h
//-------------------------------------------------------------------------------------------
#include <pcl/point_types.h>
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
inline Eigen::Affine3d XToEigMat(const t_value x[])
{
  Eigen::Affine3d res;
  res= Eigen::Translation3d(x[0],x[1],x[2])
          * Eigen::Quaterniond(x[6], x[3],x[4],x[5]);
  return res;
}
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



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // pcl_util_h
//-------------------------------------------------------------------------------------------
