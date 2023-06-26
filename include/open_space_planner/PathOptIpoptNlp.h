/***********************************
 * File Name   : PathOptIpoptNlp.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/

#ifndef OPEN_SPACE_PATH_OPTIMIZER_NLP_H
#define OPEN_SPACE_PATH_OPTIMIZER_NLP_H

#include "IpTNLP.hpp"
#include "PathOptimizerMisc.h"

using namespace Ipopt;

namespace path_optimizer
{
    class PathOptIpoptNlp : public TNLP
    {
    public:
        /** default constructor */
        PathOptIpoptNlp();

        /** default destructor */
        virtual ~PathOptIpoptNlp();

        /**@name Overloaded from TNLP */
        //@{
        /** Method to return some info about the nlp */
        virtual bool get_nlp_info(
            Ipopt::Index &n,
            Ipopt::Index &m,
            Ipopt::Index &nnz_jac_g,
            Ipopt::Index &nnz_h_lag,
            IndexStyleEnum &index_style);

        /** Method to return the bounds for my problem */
        virtual bool get_bounds_info(
            Ipopt::Index n,
            Number *x_l,
            Number *x_u,
            Ipopt::Index m,
            Number *g_l,
            Number *g_u);

        /** Method to return the starting point for the algorithm */
        virtual bool get_starting_point(
            Ipopt::Index n,
            bool init_x,
            Number *x,
            bool init_z,
            Number *z_L,
            Number *z_U,
            Ipopt::Index m,
            bool init_lambda,
            Number *lambda);

        /** Method to return the objective value */
        virtual bool eval_f(
            Ipopt::Index n,
            const Number *x,
            bool new_x,
            Number &obj_value);

        /** Method to return the gradient of the objective */
        virtual bool eval_grad_f(
            Ipopt::Index n,
            const Number *x,
            bool new_x,
            Number *grad_f);

        /** Method to return the constraint residuals */
        virtual bool eval_g(
            Ipopt::Index n,
            const Number *x,
            bool new_x,
            Ipopt::Index m,
            Number *g);

        /** Method to return:
         *   1) The structure of the Jacobian (if "values" is NULL)
         *   2) The values of the Jacobian (if "values" is not NULL)
         */
        virtual bool eval_jac_g(
            Ipopt::Index n,
            const Number *x,
            bool new_x,
            Ipopt::Index m,
            Ipopt::Index nele_jac,
            Ipopt::Index *iRow,
            Ipopt::Index *jCol,
            Number *values);

        /** Method to return:
         *   1) The structure of the Hessian of the Lagrangian (if "values" is NULL)
         *   2) The values of the Hessian of the Lagrangian (if "values" is not NULL)
         */
        virtual bool eval_h(
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
            Number *values);

        /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
        virtual void finalize_solution(
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
            IpoptCalculatedQuantities *ip_cq);
        //@}

        void set_ref_path(const std::vector<float> &ref_x_, const std::vector<float> &ref_y_);
        // void set_nearest_obs(const std::vector<float> &nearest_obs_x_, const std::vector<float> &nearest_obs_y_);

        void set_Q_smooth(const float &Q_smooth_);
        void set_Q_ref(const float &Q_smooth_);
        // void set_Q_collision_d_max(const float &Q_collision_, const float &d_max_);

        void set_opt_param(const size_t &NX_, const size_t &N_, const OptimizeType &opt_type_);

        void get_opt_res(std::vector<float> &result_);

    private:
        /**@name Methods to block default compiler methods.
         *
         * The compiler automatically generates the following three methods.
         *  Since the default compiler implementation is generally not what
         *  you want (for all but the most simple classes), we usually
         *  put the declarations of these methods in the private section
         *  and never implement them. This prevents the compiler from
         *  implementing an incorrect "default" behavior without us
         *  knowing. (See Scott Meyers book, "Effective C++")
         */
        //@{
        PathOptIpoptNlp(
            const PathOptIpoptNlp &);

        PathOptIpoptNlp &operator=(
            const PathOptIpoptNlp &);

        size_t N; // must: N >= 5
        size_t NX;
        std::vector<float> x_optimized;

        OptimizeType opt_type;

        float Q_smooth;
        float Q_ref;
        // float Q_collision;

        std::vector<float> ref_x;
        std::vector<float> ref_y;
        // std::vector<float> nearest_obs_x;
        // std::vector<float> nearest_obs_y;
        // float d_max;

        //@}
    };
}
#endif
