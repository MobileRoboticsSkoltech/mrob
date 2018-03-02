/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR_HPP_
#define FACTOR_HPP_

#include "Eigen/Dense"
#include <vector>
#include "node.hpp"

namespace fg{

/**
 * Factor class is a base pure abstract class defining factors,
 * the second type of vertexes on factor graphs (bipartite).
 * Factors keep track of all their neighbour nodes they are connected to.
 *
 * Because the number of Nodes they point to is fixed, we only allow
 * to indicate its node neighbours at the object declaration.
 * On the abstract class constructor they are not indicated, but should
 * be on any child class.
 */

class Factor{
public:
    /**
     * On the derived class constructor we will specify the (ordered)
     * nodes that the factor is connected to.
     */
    Factor(uint_t potNumberNodes = 5);
    virtual ~Factor();
    virtual int getDim(void) const = 0;
    const std::vector<std::shared_ptr<Node> >*
            getNeighbourFactors(void) const {return &neighbourNodes_;};
protected:
    std::vector<std::shared_ptr<Node> > neighbourNodes_;
};

}

#endif /* FACTOR_HPP_ */
