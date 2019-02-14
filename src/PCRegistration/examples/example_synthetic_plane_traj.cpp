/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
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


int main()
{
    uint_t numPlanes = 6, numPoses = 6;
    // 1) define problem conditions
    mrob::CreatePoints scene(100,numPlanes,numPoses,0.0001);
    //scene.print();

    mrob::PlaneRegistration contPlanes(numPlanes,numPoses);
    scene.create_plane_registration(contPlanes);
    contPlanes.print();


    // 3) evaluate alignment
    contPlanes.solve();
    contPlanes.print(false);
    contPlanes.solve();
    contPlanes.print(false);
    contPlanes.solve();
    contPlanes.print(false);
    contPlanes.solve();
    contPlanes.print(false);
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();
    contPlanes.solve();


    return 1;
}
