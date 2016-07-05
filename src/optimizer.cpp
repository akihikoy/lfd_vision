//-------------------------------------------------------------------------------------------
/*! \file    optimizer.cpp
    \brief   General optimizers.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jun.28, 2016
*/
//-------------------------------------------------------------------------------------------
#include <cstdio>
#include <cstdlib> /* free() */
#include <iostream>
#include <cma_es/cmaes_interface.h>
extern "C" {
#include <cma_es/boundary_transformation.h>
}
//-------------------------------------------------------------------------------------------
#include "lfd_vision/optimizer.h"
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


TCMAESParams::TCMAESParams()
{
  lambda= 0;

  stopMaxFunEvals= 22500;
  stopTolFun= 1e-12;
  stopTolFunHist= 1e-13;
  diagonalCov= 0.0;

  PrintLevel= 1;
}
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
  )
{
  /* be sure that initialSigma is smaller than upper minus lower bound */
  cmaes_boundary_transformation_t boundaries;
  cmaes_boundary_transformation_init(&boundaries, xmin, xmax, /*len=*/bound_len);
  /* Initialize everything into the struct evo, 0 means default */
  cmaes_t evo; /* an CMA-ES type struct or "object" */
  double *arFunvals= cmaes_init(&evo, /*dim=*/dim, /*x0=*/x0, /*sig0=*/sig0,
                        /*seed=*/0, /*lambda=*/params.lambda, /*param_file=*/"");
  #define COPY_PARAM(p)  evo.sp.p= params.p;
  COPY_PARAM(stopMaxFunEvals);
  COPY_PARAM(stopTolFun);
  COPY_PARAM(stopTolFunHist);
  #undef COPY_PARAM

  if(params.PrintLevel>=1)
  {
    printf("%s\n", cmaes_SayHello(&evo));
    cmaes_ReadSignals(&evo, "cmaes_signals.par");  /* write header and initial values */
  }

  // temporary value
  double *x_in_bounds= cmaes_NewDouble(dim); /* calloc another vector */

  int counter(params.stopMaxFunEvals);
  bool is_terminated(false);
  /* Iterate until stop criterion holds */
  while(!cmaes_TestForTermination(&evo))
  {
    /* generate lambda new search points, sample population */
    double *const *pop= cmaes_SamplePopulation(&evo); /* do not change content of pop */

    /* evaluate the new search points using fitfun */
    for(int i(0),i_end(cmaes_Get(&evo,"lambda")); i<i_end; ++i)
    {
      while(true)
      {
        cmaes_boundary_transformation(&boundaries, pop[i], x_in_bounds, dim);
        double value; bool is_feasible;
        value= f(x_in_bounds, is_feasible);
        --counter;
        if(counter<=0)  {is_terminated= true;}
        if(is_feasible) {arFunvals[i]= value; break;}
        if(is_terminated)  break;
        cmaes_ReSampleSingle(&evo, i);
      }
      if(is_terminated)  break;
    }
    if(is_terminated)  break;
    cmaes_UpdateDistribution(&evo, arFunvals);
    if(params.PrintLevel>=1)  cmaes_ReadSignals(&evo, "cmaes_signals.par");
  }
  if(params.PrintLevel>=1)
  {
    if(!is_terminated)  printf("Stop:\n%s\n",  cmaes_TestForTermination(&evo)); /* print termination reason */
    else  printf("Stop:\n%s\n",  "Number of function evaluations researched max");
  }
  if(params.PrintLevel>=1)  cmaes_WriteToFile(&evo, "all", "/tmp/allcmaes.dat"); /* write final results */
  if(params.PrintLevel>=2)  cmaes_WriteToFile(&evo, "all", "/dev/stdout");

  /* get best estimator for the optimum, xmean or xbestever */
  cmaes_boundary_transformation(&boundaries, (double const*)cmaes_GetPtr(&evo, "xbestever"), x_in_bounds, dim);

  for(int d(0);d<dim;++d)  xres[d]= x_in_bounds[d];

  cmaes_exit(&evo); /* release memory */
  cmaes_boundary_transformation_exit(&boundaries); /* release memory */
  free(x_in_bounds);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

