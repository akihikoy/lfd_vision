//-------------------------------------------------------------------------------------------
/*! \file    optimizer.h
    \brief   General optimizers.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jun.28, 2016
*/
//-------------------------------------------------------------------------------------------
#ifndef optimizer_h
#define optimizer_h
//-------------------------------------------------------------------------------------------
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

// Subset of CMA-ES parameters
struct TCMAESParams
{
  // Initialize
  int lambda;  // Number of population; 0: default

  // From cmaes_readpara_t
  double stopMaxFunEvals;
  double stopTolFun;
  double stopTolFunHist;
  double diagonalCov;

  // Extra
  int PrintLevel;  // Print level; 0: no output, 1: standard, 2: verbose.

  TCMAESParams();
};
//-------------------------------------------------------------------------------------------

/* Minimize an objective function f with CMA-ES.
    f(x,is_feasible): Objective function to be minimized
        (x: input parameters, is_feasible: output bool to store if x is feasible).
    x0, sig0, dim: Initial x and std-dev, and their dimension
    xmin, xmax, bound_len: Bounds and the length of xmin and xmax
    xres: Result (should be allocated by user)
    params: CMA-ES parameters
*/
void MinimizeF(
    boost::function<double(const double x[], bool &is_feasible)> f,
    double x0[], double sig0[], int dim,
    const double xmin[], const double xmax[], int bound_len,
    double *xres,
    const TCMAESParams &params
  );
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // optimizer_h
//-------------------------------------------------------------------------------------------
