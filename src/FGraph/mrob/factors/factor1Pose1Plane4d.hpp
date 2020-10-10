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
 * factor1Pose1Plane4d.hpp
 *
 *  Created on: Oct 10, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTOR1POSE1PLANE4D_HPP_
#define FACTOR1POSE1PLANE4D_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"

namespace mrob{

/**
 * Factor 1 Pose (3D) and plane 4d relates a pose with a plane observation, extracted from
 * segmented points in a point cloud.
 *
 *
 */

class Factor1Poses1Plane4d : public Factor
{
  public:
    Factor1Poses1Plane4d();
    ~Factor1Poses1Plane4d();
};

}

#endif /* FACTOR1POSE1PLANE4D_HPP_ */
