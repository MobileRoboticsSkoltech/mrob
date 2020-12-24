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
 * plane_to_pc_registration.cpp
 *
 *  Created on: Dec 24, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/plane_to_pc_registration.hpp"
#include "mrob/factors/nodePose3d.hpp"

using namespace mrob;

PlaneToPcRegistration::PlaneToPcRegistration():
        graph_(), T_from_camera_to_range_()
{
    // Initialialization, single node in the Fgraph
    std::shared_ptr<mrob::Node> n0(new mrob::NodePose3d(T_from_camera_to_range_));
    graph_.add_node(n0);
}

PlaneToPcRegistration::~PlaneToPcRegistration()
{

}



