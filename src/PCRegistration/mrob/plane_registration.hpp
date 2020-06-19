/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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

#include <unordered_map>
#include <memory>


namespace mrob{

/**
 * class PlaneRegistration introduced a class for the alignment of
 * planes.
 */
class PlaneRegistration{

  public:
    // XXX is this mode used anymore? deprecated?
    enum TrajectoryMode{SEQUENCE=0, INTERPOLATION};
    // XXX Solve method is almost deprecated
    //enum SolveModeGrad{GRADIENT_DESCENT_NAIVE=0, GRADIENT_DESCENT_INCR, STEEPEST, HEAVYBALL, MOMENTUM, MOMENTUM_SEQ, BENGIOS_NAG, GRADIENT_DESCENT_BACKTRACKING, BFGS};
    enum SolveMode{INITIALIZE=0,
                   GRADIENT,
                   GRADIENT_BENGIOS_NAG,
                   GN_HESSIAN,
                   LM_HESSIAN,
                   GN_CLAMPED_HESSIAN,
                   LM_CLAMPED_HESSIAN};

  public:
    PlaneRegistration();
    //PlaneRegistration(uint_t numberPlanes , uint_t numberPoses);
    ~PlaneRegistration();

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
    /**
     * solve_interpolate_hessian() calculates the poses on trajectory such that the minimization objective
     * is met: J = sum (lamda_min_plane), and the trajectory is described as an interpolation from I to T_f
     * using second order methods with Hessian. Very similar to solve_interpolate
     */
    uint_t solve_interpolate_hessian(bool singleIteration = false);
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
     * Get trajectory returns a smart pointer to the vector of transformations,
     * which is already shared by all Plane objects.
     * It serves for checking the solution and for modifying the initial conditions for optimization (if any).
     */
    //std::shared_ptr<std::vector<SE3>>& get_trajectory() {return trajectory_;};//if solved
    Mat4 get_trajectory(uint_t time);

    /**
     * add_plane adds a plane structure already initialized and filled with data
     */
    void add_plane(uint_t id, std::shared_ptr<Plane> &plane);
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
    std::vector<double> print_evaluate() const;

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
    uint_t numberPlanes_, numberPoses_;
    uint_t isSolved_;
    PlaneRegistration::TrajectoryMode trajMode_;
    uint_t time_;
    std::unordered_map<uint_t, std::shared_ptr<Plane>> planes_;
    std::shared_ptr<std::vector<SE3>> trajectory_;
    uint_t solveIters_;

    // 1st order parameters methods if used
    PlaneRegistration::SolveMode solveMode_;
    std::vector<Mat61> previousState_;
    double c1_, c2_;    //parameters for the Wolfe conditions DEPRECATED?
    double alpha_, beta_;

    //2nd order data (if used)
    Mat61 gradient_;
    Mat6 hessian_;

};



}// namespace
#endif /* PLANEREGISTRATION_HPP_ */
