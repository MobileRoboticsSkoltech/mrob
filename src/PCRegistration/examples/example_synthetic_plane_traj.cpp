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


int main()
{
    mrob::CreatePoints scene(100,1);
    scene.print();

    return 1;
}
