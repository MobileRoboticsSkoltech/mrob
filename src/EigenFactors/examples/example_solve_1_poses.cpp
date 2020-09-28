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
 * example_solve_1_poses.cpp
 *
 *  Created on: Oct 23, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


/** This code is a toy example to show how to use the basics for plane alignment.
 *  For a more realistic example take a look at the python example
 */


#include "mrob/factor_graph_ef.hpp"


using namespace mrob;

// This examples show how to optimize 1 pose of a stream of planes

int main()
{
    // 1) create mock scene with some planes and GT poses
    // 2) Add information to the EF Fgraph structure. For now only planes
    // 3) Optimize, Measure distance to GT
}
