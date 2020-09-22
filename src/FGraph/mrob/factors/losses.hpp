/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
 *
 * losses.hpp
 *
 *  Created on: Apr 22, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef FACTORS_LOSSES_HPP_
#define FACTORS_LOSSES_HPP_



/**
 * This header defines Robust Loss functions that could be adopted by factors.
 * The idea is to specify for every factor the particular loss function.
 *
 * BaseLoss is the abstract class defining the interface for other loss functions
 */


class BaseLoss
{
  public:
    BaseLoss();
    ~BaseLoss();
    double loss(double r) = 0;
  private:
    double threshold_;
};

/**
 * No robust Loss
 */

/**
 * Huber Loss
 */

/**
 * Tukey Loss
 */
#endif /* FACTORS_LOSSES_HPP_ */
