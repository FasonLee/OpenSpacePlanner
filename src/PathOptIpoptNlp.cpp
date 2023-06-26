/***********************************
 * File Name   : PathOptIpoptNlp.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description :
 ***********************************/

#include "math.h"
#include "PathOptIpoptNlp.h"

#include <cassert>
#include <iostream>
#include <fstream>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

using namespace Ipopt;
using namespace path_optimizer;

/* Constructor. */
PathOptIpoptNlp::PathOptIpoptNlp()
{
}

PathOptIpoptNlp::~PathOptIpoptNlp()
{
}

bool PathOptIpoptNlp::get_nlp_info(
    Ipopt::Index &n,
    Ipopt::Index &m,
    Ipopt::Index &nnz_jac_g,
    Ipopt::Index &nnz_h_lag,
    IndexStyleEnum &index_style)
{
   // define opt_var like this:
   // [ x0 y0 x1 y1 ... xN yN ]
   n = N * NX;

   m = 4;

   nnz_jac_g = 4; // nonzero elements in constraints jacobian

   nnz_h_lag = 0; // quasi-newton

   index_style = C_STYLE;

   // std::cout<<"=====================get_nlp_info done!"<<std::endl;
   return true;
}

bool PathOptIpoptNlp::get_bounds_info(
    Ipopt::Index n,
    Number *x_l,
    Number *x_u,
    Ipopt::Index m,
    Number *g_l,
    Number *g_u)
{
   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
   // If desired, we could assert to make sure they are what we think they are.
   // assert(n == N * NXU + NX);
   // assert(m == N * NX + NX);

   for (size_t i = 0; i < N; ++i)
   {
      x_l[i * NX] = -2.0e19;
      x_u[i * NX] = +2.0e19;
      x_l[i * NX + 1] = -2.0e19;
      x_u[i * NX + 1] = +2.0e19;
   }
   g_l[0] = g_u[0] = ref_x[0];
   g_l[1] = g_u[1] = ref_y[0];
   g_l[2] = g_u[2] = ref_x[N - 1];
   g_l[3] = g_u[3] = ref_y[N - 1];
   // std::cout<<"=====================get_bounds_info done!"<<std::endl;
   return true;
}

bool PathOptIpoptNlp::get_starting_point(
    Ipopt::Index n,
    bool init_x,
    Number *x,
    bool init_z,
    Number *z_L,
    Number *z_U,
    Ipopt::Index m,
    bool init_lambda,
    Number *lambda)
{
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   for (size_t i = 0; i < N; ++i)
   {
      x[i * NX] = ref_x[i];
      x[i * NX + 1] = ref_y[i];
   }

   // std::cout<<"=====================get_starting_point done!"<<std::endl;
   return true;
}

bool PathOptIpoptNlp::eval_f(
    Ipopt::Index n,
    const Number *x,
    bool new_x,
    Number &obj_value)
{
   obj_value = 0.0; // initialize

   /* smooth term */
   if (static_cast<std::underlying_type<OptimizeType>::type>(opt_type) & (1 << static_cast<std::underlying_type<OptimizeType>::type>(OptimizeType::Smooth)))
   {
      for (size_t i = 1; i < N - 2; ++i)
      {
         obj_value += Q_smooth * pow(x[(i + 1) * NX] - 2 * x[i * NX] + x[(i - 1) * NX], 2);             // delta x(i+1) - delta x(i)
         obj_value += Q_smooth * pow(x[(i + 1) * NX + 1] - 2 * x[i * NX + 1] + x[(i - 1) * NX + 1], 2); // delta y(i+1) - delta y(i)
      }
   }

   /* close to ref term */
   if (static_cast<std::underlying_type<OptimizeType>::type>(opt_type) & (1 << static_cast<std::underlying_type<OptimizeType>::type>(OptimizeType::CloseToRef)))
   {
      for (size_t i = 1; i < N - 2; ++i) // 0 and N are handled by equality constraints
      {
         obj_value += Q_ref * pow(x[i * NX] - ref_x[i], 2);
         obj_value += Q_ref * pow(x[i * NX + 1] - ref_y[i], 2);
      }
   }

   /* collision avoidance term */
   // if (static_cast<std::underlying_type<OptimizeType>::type>(opt_type) & (1 << static_cast<std::underlying_type<OptimizeType>::type>(OptimizeType::AvoidCollision)))
   // {
   //    float obs_dist = 0.0f;
   //    for (size_t i = 1; i < N - 2; ++i) // 0 and N are handled by equality constraints
   //    {
   //       obs_dist = hypot(x[i * NX] - nearest_obs_x[i], x[i * NX + 1] - nearest_obs_y[i]);
   //       // if (obs_dist < d_max)
   //       {
   //          obj_value += Q_collision * pow(obs_dist - d_max, 2);
   //       }
   //    }
   // }
   // std::cout<<"=====================eval_f done!"<<std::endl;
   return true;
}

bool PathOptIpoptNlp::eval_grad_f(
    Ipopt::Index n,
    const Number *x,
    bool new_x,
    Number *grad_f)
{
   // return the gradient of the objective function grad_{x} f(x)

   /* init */
   for (size_t i = 0; i < N; ++i)
   {
      grad_f[i * NX] = 0;
      grad_f[i * NX + 1] = 0;
   }

   /* smooth term */
   if (static_cast<std::underlying_type<OptimizeType>::type>(opt_type) & (1 << static_cast<std::underlying_type<OptimizeType>::type>(OptimizeType::Smooth)))
   {
      // dJ/dx(0)
      grad_f[0] += Q_smooth * (2 * x[0] - 4 * x[NX] + 2 * x[2 * NX]);
      // dJ/dy(0)
      grad_f[1] += Q_smooth * (2 * x[1] - 4 * x[NX + 1] + 2 * x[2 * NX + 1]);
      // dJ/dx(1)
      grad_f[NX] += Q_smooth * (-4 * x[0] + 10 * x[NX] - 8 * x[2 * NX] + 2 * x[3 * NX]);
      // dJ/dy(1)
      grad_f[NX + 1] += Q_smooth * (-4 * x[1] + 10 * x[NX + 1] - 8 * x[2 * NX + 1] + 2 * x[3 * NX + 1]);
      // dJ/dx(N-2)
      grad_f[(N - 2) * NX] += Q_smooth * (2 * x[(N - 4) * NX] - 8 * x[(N - 3) * NX] + 10 * x[(N - 2) * NX] - 4 * x[(N - 1) * NX]);
      // dJ/dy(N-2)
      grad_f[(N - 2) * NX + 1] += Q_smooth * (2 * x[(N - 4) * NX + 1] - 8 * x[(N - 3) * NX + 1] + 10 * x[(N - 2) * NX + 1] - 4 * x[(N - 1) * NX + 1]);
      // dJ/dx(N-1)
      grad_f[(N - 1) * NX] += Q_smooth * (2 * x[(N - 3) * NX] - 4 * x[(N - 2) * NX] + 2 * x[(N - 1) * NX]);
      // dJ/dy(N-1)
      grad_f[(N - 1) * NX + 1] += Q_smooth * (2 * x[(N - 3) * NX + 1] - 4 * x[(N - 2) * NX + 1] + 2 * x[(N - 1) * NX + 1]);
      // other
      for (size_t i = 2; i < N - 2; ++i)
      {
         // dJ/dx(i)
         grad_f[i * NX] += Q_smooth * (2 * x[(i - 2) * NX] - 8 * x[(i - 1) * NX] + 12 * x[i * NX] - 8 * x[(i + 1) * NX] + 2 * x[(i + 2) * NX]);
         // dJ/dy(i)
         grad_f[i * NX + 1] += Q_smooth * (2 * x[(i - 2) * NX + 1] - 8 * x[(i - 1) * NX + 1] + 12 * x[i * NX + 1] - 8 * x[(i + 1) * NX + 1] + 2 * x[(i + 2) * NX + 1]);
      }
   }

   /* close to ref term */
   if (static_cast<std::underlying_type<OptimizeType>::type>(opt_type) & (1 << static_cast<std::underlying_type<OptimizeType>::type>(OptimizeType::CloseToRef)))
   {
      for (size_t i = 0; i < N; ++i)
      {
         grad_f[i * NX] += Q_ref * 2 * (x[i * NX] - ref_x[i]);
         grad_f[i * NX + 1] += Q_ref * 2 * (x[i * NX + 1] - ref_y[i]);
      }
   }

   /* collision avoidance term */
   // if (static_cast<std::underlying_type<OptimizeType>::type>(opt_type) & (1 << static_cast<std::underlying_type<OptimizeType>::type>(OptimizeType::AvoidCollision)))
   // {
   //    float obs_dist = 0.0f;
   //    for (size_t i = 1; i < N - 2; ++i) // 0 and N are handled by equality constraints
   //    {
   //       obs_dist = hypot(x[i * NX] - nearest_obs_x[i], x[i * NX + 1] - nearest_obs_y[i]);
   //       // obs_dist = std::max(obs_dist, 0.001f); // avoid 0 value
   //       // if (obs_dist < d_max)
   //       {
   //          grad_f[i * NX] += Q_collision * 2 * (x[i * NX] - nearest_obs_x[i]) * (1 - d_max / obs_dist);

   //          grad_f[i * NX + 1] += Q_collision * 2 * (x[i * NX + 1] - nearest_obs_y[i]) * (1 - d_max / obs_dist);
   //       }
   //    }
   // }
   // std::cout<<"=====================eval_grad_f done!"<<std::endl;
   return true;
}

bool PathOptIpoptNlp::eval_g(
    Ipopt::Index n,
    const Number *x,
    bool new_x,
    Ipopt::Index m,
    Number *g)
{
   g[0] = x[0]; // x[0] = ref_x[0];
   g[1] = x[1]; // x[1] = ref_y[0];
   g[2] = x[(N - 1) * NX];
   g[3] = x[(N - 1) * NX + 1];
   // std::cout<<"=====================eval_g done!"<<std::endl;
   return true;
}

bool PathOptIpoptNlp::eval_jac_g(
    Ipopt::Index n,
    const Number *x,
    bool new_x,
    Ipopt::Index m,
    Ipopt::Index nele_jac,
    Ipopt::Index *iRow,
    Ipopt::Index *jCol,
    Number *values)
{
   if (values == NULL)
   {
      // return the structure of the jacobian of the constraints
      iRow[0] = 0;
      jCol[0] = 0;
      iRow[1] = 1;
      jCol[1] = 1;
      iRow[2] = 2;
      jCol[2] = (N - 1) * NX;
      iRow[3] = 3;
      jCol[3] = (N - 1) * NX + 1;
   }
   else
   {
      values[0] = 1.0f;
      values[1] = 1.0f;
      values[2] = 1.0f;
      values[3] = 1.0f;
   }

   return true;
}

bool PathOptIpoptNlp::eval_h(
    Ipopt::Index n,
    const Number *x,
    bool new_x,
    Number obj_factor,
    Ipopt::Index m,
    const Number *lambda,
    bool new_lambda,
    Ipopt::Index nele_hess,
    Ipopt::Index *iRow,
    Ipopt::Index *jCol,
    Number *values)
{
   return true;
}

void PathOptIpoptNlp::finalize_solution(
    SolverReturn status,
    Ipopt::Index n,
    const Number *x,
    const Number *z_L,
    const Number *z_U,
    Ipopt::Index m,
    const Number *g,
    const Number *lambda,
    Number obj_value,
    const IpoptData *ip_data,
    IpoptCalculatedQuantities *ip_cq)
{
   // std::cout << "finalize_solution" << std::endl;
   for (int i = 0; i < n; ++i)
   {
      x_optimized.push_back(x[i]);
   }
}

void PathOptIpoptNlp::set_ref_path(const std::vector<float> &ref_x_, const std::vector<float> &ref_y_)
{
   ref_x = ref_x_;
   ref_y = ref_y_;
}

// void PathOptIpoptNlp::set_nearest_obs(const std::vector<float> &nearest_obs_x_, const std::vector<float> &nearest_obs_y_)
// {
//    nearest_obs_x = nearest_obs_x_;
//    nearest_obs_y = nearest_obs_y_;
// }

void PathOptIpoptNlp::set_Q_smooth(const float &Q_smooth_)
{
   Q_smooth = Q_smooth_;
}

void PathOptIpoptNlp::set_Q_ref(const float &Q_ref_)
{
   Q_ref = Q_ref_;
}

// void PathOptIpoptNlp::set_Q_collision_d_max(const float &Q_collision_, const float &d_max_)
// {
//    d_max = d_max_;
// }

void PathOptIpoptNlp::set_opt_param(const size_t &NX_, const size_t &N_, const OptimizeType &opt_type_)
{
   NX = NX_;
   N = N_;
   opt_type = opt_type_;
}

void PathOptIpoptNlp::get_opt_res(std::vector<float> &result_)
{
   result_.clear();
   result_ = x_optimized;
}
