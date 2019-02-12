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


int main()
{
    // 1) define problem conditions
    mrob::CreatePoints scene(10,1,2,0.0001);
    //scene.print();

    mrob::PlaneRegistration contPlanes(1,2);
    auto planes = scene.get_planes();
    for (auto plane_element : planes)
    {
        contPlanes.add_plane(plane_element.first, plane_element.second);
    }
    contPlanes.print();


    // 2) evaluate planes
    if (1)
    {
        planes[0].second->estimate_plane();
        planes[0].second->calculate_jacobian(0);
    }




    return 1;
}
