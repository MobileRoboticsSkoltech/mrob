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
