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
 * estimate_plane.hpp
 *
 *  Created on: Dec 28, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

//TODO this file name should be changed since it provides more functionaloties for PCs
#ifndef ESTIMATE_PLANE_HPP_
#define ESTIMATE_PLANE_HPP_

#include "mrob/matrix_base.hpp"

namespace mrob{

/**
 * Estimate plane: given a set of points, in a Nx3 array,
 * calculates the plane as
 *    pi = [n,
 *          d] \in P^3
 *
 * For now this is a unique function, TODO: add more methods here
 */

Mat41 estimate_plane(MatRefConst X);


/**
 * Estimate normal: given a set of points, in a Nx3 array,
 * calculates the noamrl plane
 *    n \in R^3
 *
 */

Mat31 estimate_normal(MatRefConst X);


/**
 * Estimate centroid: given a set of points, in a Nx3 array,
 * calculates the centroid point
 *    p \in R^3
 *
 */

Mat31 estimate_centroid(MatRefConst X);


}

#endif /* ESTIMATE_PLANE_HPP_ */
