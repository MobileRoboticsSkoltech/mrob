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

namespace skmr{

/**
 * Factor class is a base pure abstract class to contain factors, the second type of vertexes
 * on factor graphs (bipartite).
 * Factors encode their Ids and all the neighbour nodes they are connected to.
 */

class Factor{
public:
    Factor(unsigned int id, unsigned int potNumberNodes = 5);
    virtual ~Factor();
    virtual int getDim(void) const = 0;

    unsigned int getId(void) const {return id_;};
    /**
     * Adds a node to the connected nodes in this factor (usually 2)
     */
    void addNode(std::shared_ptr<Node> &node);
    void rmNode(std::shared_ptr<Node> &node);
    const std::vector<std::shared_ptr<Node> >*
            getNeighbourFactors(void) const {return &neighbourNodes_;};
protected:
    unsigned int id_;
    std::vector<std::shared_ptr<Node> > neighbourNodes_;
};

}

#endif /* FACTOR_HPP_ */
