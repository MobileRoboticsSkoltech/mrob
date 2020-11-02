/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * planeRegistration.hpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef PLANEREGISTRATION_HPP_
#define PLANEREGISTRATION_HPP_

#include <vector>
#include "mrob/SE3.hpp"
#include <Eigen/StdVector>
#include "mrob/plane.hpp"
#include "mrob/optimizer.hpp"
#include "mrob/time_profiling.hpp"

#include <unordered_map>
#include <memory>


namespace mrob{

/**
 * class PlaneRegistration introduced a class for the alignment of
 * planes.
 */
class PlaneRegistration: public Optimizer{

  public:
    // XXX is this mode used anymore? deprecated?
    enum TrajectoryMode{SEQUENCE=0, INTERPOLATION};
    // XXX Solve method is almost deprecated
    //enum SolveModeGrad{GRADIENT_DESCENT_NAIVE=0, GRADIENT_DESCENT_INCR, STEEPEST, HEAVYBALL, MOMENTUM, MOMENTUM_SEQ, BENGIOS_NAG, GRADIENT_DESCENT_BACKTRACKING, BFGS};
    enum SolveMode{INITIALIZE=0,
                   GRADIENT,
                   GRADIENT_BENGIOS_NAG,
                   GRADIENT_ALL_POSES,
                   GN_HESSIAN,
                   GN_CLAMPED_HESSIAN,
                   LM_SPHER,
                   LM_ELLIP};

  public:
    PlaneRegistration();
    //PlaneRegistration(uint_t numberPlanes , uint_t numberPoses);
    ~PlaneRegistration();

    // Function from the parent class Optimizer
    virtual matData_t calculate_error() override;
    virtual void calculate_gradient_hessian() override;
    virtual void update_state(const MatX1 &dx) override;
    virtual void bookkeep_state() override;
    virtual void update_state_from_bookkeep() override;


    // Specific methods
    void set_number_planes_and_poses(uint_t numPlanes, uint_t numPoses);
    uint_t get_number_planes() const {return numberPlanes_;};
    uint_t get_number_poses() const {return numberPoses_;};

    /**
     * solve() calculates the poses on trajectory such that the minimization objective
     * is met: J = sum (lamda_min_plane)
     */
    uint_t solve(SolveMode mode, bool singleIteration = false);
    /**
     * solve_interpolate() calculates the poses on trajectory such that the minimization objective
     * is met: J = sum (lamda_min_plane), and the trajectory is described as an interpolation from I to T_f
     */
    uint_t solve_interpolate_gradient(bool singleIteration = false);
    //Gradient for all poses
    uint_t solve_gradient_all_poses(bool singleIteration = false);
    /**
     * solve_interpolate_hessian() calculates the poses on trajectory such that the minimization objective
     * is met: J = sum (lamda_min_plane), and the trajectory is described as an interpolation from I to T_f
     * using second order methods with Hessian. Very similar to solve_interpolate
     */
    //uint_t solve_interpolate_hessian(bool singleIteration = false);
    /**
     * Initialization_solve give a first guess on all poses by using classical point-point
     * methods SVD-based to calculate an initial condition closer to the true solution
     */
    uint_t solve_initialize();
    /**
     * Solve quaternion plane uses a paramteric representation for each plane, a quaternion,
     * and optimizes both the plane parameters and the trajectory variables
     */
    uint_t solve_quaternion_plane();
    /**
     * reset_solution, resets the current calculated solution while maintainting all data (planes)
     * This function is intended for comparing different solvers without replicating data
     */
    void reset_solution();
    double get_current_error() const;
    /**
     * Get trajectory returns a SE3 transformations,
     */
    SE3 get_trajectory(uint_t time);

    // XXX Used anywhere??
    SE3 get_last_pose() {return trajectory_->back();}

    /**
     * Sets the trajectory (current solution) by addint the last pose
     */
    void set_last_pose(SE3 &last);

    SE3 get_last_pose() {return trajectory_->back();}

    /**
     * Sets the trajectory (current solution) by addint the last pose
     */
    void set_last_pose(SE3 &last);

    /**
     * add_plane adds a plane structure already initialized and filled with data
     */
    void add_plane(uint_t id, std::shared_ptr<Plane> &plane);
    /**
     * add new plane is an ALTERNATIVE (for py) to the above function
     * which creates a new plane inside the class and then points are added one by one
     *
     */
    void add_new_plane(uint_t id);
    /**
     * add a point to the new plane. An ALTERNATIVE (for py) to add points into the sover
     * class (instead of adding the point fully as in add_plane())
     *
     */
    void plane_push_back_point(uint_t id, uint_t t, Mat31 &point);

    uint_t calculate_total_number_points();
    std::shared_ptr<Plane> & get_plane(uint_t id);

    std::unordered_map<uint_t, std::shared_ptr<Plane>>& get_all_planes() {return planes_;};

    void set_alpha_parameter(double alpha) {alpha_ = alpha;};
    void set_beta_parameter(double beta) {beta_ = beta;};

    double calculate_poses_rmse(std::vector<SE3> & groundTruth) const;

    void print(bool plotPlanes = true) const;

    /**
     * print evaluate looks for degenerate cases, such as planes normal vectors,
     * Hessian rank, det of all normals, etc. Basically this function tries to answer
     * if the problem is ill-conditioned
     *
     * Returns: 0) current error
     *          1) number of iters,
     *          2) determinant
     *          3) number of negative eigenvalues
     *          4) conditioning number
     */
    std::vector<double> print_evaluate();

    /**
     * add point_cloud requires a complete set of points observed at a given time
     * stamp (XXX now only an integer) and fills in the registration structure.
     */
    void add_point_cloud_planes(uint_t time, std::vector<Mat31>& points, std::vector<uint_t>& point_ids);
    /**
     * get_point_cloud gets all raw point, according to the current time index
     * from trajectory. It does not distinguish between planes.
     */
    std::vector<Mat31> get_point_cloud(uint_t time);
    std::vector<Mat31> get_point_plane_ids(uint_t time);

  protected:
    // flag for detecting when is has been solved
    uint_t numberPlanes_, numberPoses_, numberPoints_;
    uint_t isSolved_;
    PlaneRegistration::TrajectoryMode trajMode_;
    uint_t time_;
    std::unordered_map<uint_t, std::shared_ptr<Plane>> planes_;
    std::shared_ptr<std::vector<SE3>> trajectory_;
    SE3 bookept_trajectory_;//last pose is stored/bookept
    double tau_;//variable for weighting the number of poses in traj
    uint_t solveIters_;

    // 1st order parameters methods if used
    PlaneRegistration::SolveMode solveMode_;
    std::vector<Mat61> previousState_;
    double c1_, c2_;    //parameters for the Wolfe conditions DEPRECATED?
    double alpha_, beta_;


    //2nd order data (if used) TODO remove since they are defined in parent class
    Mat61 gradient__;
    Mat6 hessian__;

    // time profiling
    TimeProfiling time_profiles_;
    double initial_error_; // for benchmark purposes


    // alternative structure to keep planes. This is only for the python bindings

};



}// namespace
#endif /* PLANEREGISTRATION_HPP_ */
