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


#include "skmr/base_T.hpp"
#include "skmr/arun.hpp"
#include "skmr/create_points.hpp"

int main()
{
    // test data structures
    std::shared_ptr<MatX> X (new MatX(3,6));//Initializes a zero matrix
    *X << 0, 1, 1, 2, 5, 10,
          0, 2, 3, 2, 3, -1,
          3, -1, 3, -4 ,4, 7;

    std::cout << "1st Arun\n" << *X << std::endl;
    //skmr::Carun arun(X,X);//TODO there is a problem here on the determinant!
    //arun.getT().print();
    //arun.solve();



    //align_mth::Csample_uniform_SE3 s(0.3, 4);
    //std::cout << "Csample = " << s.sample() << std::endl;




    return 1;
}

