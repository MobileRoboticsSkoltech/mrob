/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * example_align_mth.cpp
 *
 *  Created on: Feb 2, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "base_T.hpp"
#include "arun.hpp"
#include "create_points.hpp"

int main()
{
    // test data structures
    std::shared_ptr<Eigen::MatrixXd> X (new Eigen::MatrixXd(3,5));
    *X << 0, 1, 1, 2, 5,
          0, 2, 3, 2, 3,
          3, -1, 3, -4 ,4;
    std::cout << "1st Arun" << *X << std::endl;
    align_mth::Carun arun(X);

    align_mth::Csample_uniform_SE3 s(0.3, 4);
    std::cout << "Csample = " << s.sample() << std::endl;




    return 1;
}

