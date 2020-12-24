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
 * plane_to_pc_registration.hpp
 *
 *  Created on: Dec 24, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#ifndef PLANE_TO_PC_REGISTRATION_HPP_
#define PLANE_TO_PC_REGISTRATION_HPP_


#include "mrob/SE3.hpp"
#include <vector>
#include "mrob/factor_graph_solve_dense.hpp"


namespace mrob{

/**
 * class PlaneToPcRegistration solves the registration problem between a point cloud and
 * and plane estimation. A single observation of this problem would yield a rank deficient
 * solution, is for that reason that we require several observations, with changing conditions.
 *
 * In general, this class aims to solve the (calibrated) camera to range sensor calibration problem,
 * where we want to estimate the transformation between both sensors.
 *
 * As inputs, we require:
 *  - Observation of points in Camera frame and plane parameters, in Depth sensor.
 *  The points are generated externally by solving the PnP problem between a known pattern and the calibrated camera
 *  The plane is obtained after segmentation of the PC and extraction of plane parameters, also externally
 *
 *
 * The algorithm returns the solution transformation {Camera}T_{Depth}
 *
 * TODO: add uncertainty on each observation, camera and
 *
 */

class PlaneToPcRegistration {
  public:
    PlaneToPcRegistration();
    ~PlaneToPcRegistration();

  private:
    FGraphSolveDense graph_;
    SE3 T_from_camera_to_range_;
};


}

#endif /* PLANE_TO_PC_HPP_ */
