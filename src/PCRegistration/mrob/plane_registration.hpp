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
    enum TrajectoryMode{SEQUENCE=0, INTERPOLATION};
    enum SolveMode{GRADIENT_DESCENT_NAIVE=0, GRADIENT_DESCENT_INCR, STEEPEST, HEAVYBALL, MOMENTUM, MOMENTUM_SEQ, BENGIOS_NAG, GRADIENT_DESCENT_BACKTRACKING, BFGS};

  public:
    PlaneRegistration();
    PlaneRegistration(uint_t numberPlanes , uint_t numberPoses);
    ~PlaneRegistration();

    void set_number_planes_and_poses(uint_t numPlanes, uint_t numPoses);
    uint_t get_number_planes() const {return numberPlanes_;};
    uint_t get_number_poses() const {return numberPoses_;};

    void set_solving_method(SolveMode mode) {solveMode_ = mode;};

    /**
     * solve() calculates the poses on trajectory such that the minimization objective
     * is met: J = sum (lamda_min_plane)
     */
    uint_t solve(bool singleIteration = false);
    /**
     * solve_interpolate() calculates the poses on trajectory such that the minimization objective
     * is met: J = sum (lamda_min_plane), and the trajectory is described as an interpolation from I to T_f
     */
    uint_t solve_interpolate(bool singleIteration = false);
    /**
     * Initialization_solve give a first guess on all poses by using classical point-point
     * methods SVD-based to calculate an initial condition closer to the true solution
     */
    uint_t solve_initialize();
    double get_current_error();
    /**
     * Get transformation returns a smart pointer to the vector of transformations,
     * which is already shared by all Plane objects.
     * It serves for checking the solution and for modyfying the initial conditions for optimization (if any).
     */
    std::shared_ptr<std::vector<SE3>>& get_transformations() {return trajectory_;};//if solved

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


  protected:
    // flag for detecting when is has been solved
    uint_t numberPlanes_, numberPoses_;
    uint_t isSolved_;
    PlaneRegistration::TrajectoryMode trajMode_;
    uint_t time_;
    std::unordered_map<uint_t, std::shared_ptr<Plane>> planes_;
    std::shared_ptr<std::vector<SE3>> trajectory_;

    // Quasi Newton methods if used
    PlaneRegistration::SolveMode solveMode_;
    std::vector<Mat6> inverseHessian_;
    std::vector<Mat61> previousJacobian_, previousState_;
    double c1_, c2_;    //parameters for the Wolfe conditions DEPRECATED?
    double alpha_, beta_;

};



}// namespace
#endif /* PLANEREGISTRATION_HPP_ */
