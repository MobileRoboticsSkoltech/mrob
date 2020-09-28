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
 * example_synthetic_plane_traj.cpp
 *
 *  Created on: Feb 8, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/create_points.hpp"
#include "mrob/plane_registration.hpp"
#include <iostream>


using namespace mrob;

int main()
{
    uint_t numPlanes = 4, numPoses = 2;
    // 1) define problem conditions
    mrob::CreatePoints scene(40,numPlanes,numPoses,0.001, 0.1);
    //scene.print();

    mrob::PlaneRegistration contPlanes;
    scene.create_plane_registration(contPlanes);
    contPlanes.print();


    // 3) evaluate alignment
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.print(false);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.print(false);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.print(false);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.solve(PlaneRegistration::SolveMode::GRADIENT);
    contPlanes.print(false);


    return 1;
}
