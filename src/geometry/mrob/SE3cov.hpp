/** Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
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
 * @brief SE3 compounding class
 * @file SE3.hpp
 *
 * @date Created on: July, 2021
 * @author Gonzalo Ferrer, Aleksei Panchenko
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef SE3COV_HPP_
#define SE3COV_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"

/**
 * @brief Class \b SE3Cov for Special eualidean group poses compounding.
 * Class \b SE3Cov inherits properties of \b SE3 class and containes additional attribute
 * \a covariance_ to store the corresponding uncertainty. Beside that, \b SE3Cov has two 
 * compounding methods of different order.
 **/
namespace mrob
{
    class SE3Cov : public SE3
    {
    public:
        /**
         * @brief Default construct a new SE3Cov object
         * @return SE3Cov object.
         */
        SE3Cov(void);

        /** 
         * @brief Construtor of SE3Cov object.
         * @param[in] pose - SE3 oject - pose;
         * @param[in] cov - covariance matrix of the pose uncertainty;
         * @return SE3Cov object.
         * **/
        SE3Cov(const SE3 &pose, const Mat6 &cov);

        /** @brief Copy constructor of SE3Cov object
         *  @param[in] pose - SE3 oject - pose
         **/
        SE3Cov(const SE3Cov &pose);
        Mat6 covariance_;

        /** 
         * @brief \b cov() returns current covariance matrix state
         * @return Mat6 - uncertainty covariance matrix
         * **/
        Mat6 cov() const;

        /** 
         * @brief SE3 pose uncertainty compounding of the second order.
         * Implemented according to the paper:
         * @url: http:// #TODO
         * @param[in] pose - SE3Cov object: incremental pose + increment uncertainty
         * @return void
         **/
        void compound_2nd_order(const SE3Cov &pose);

        /** 
         * @brief SE3 pose uncertainty compounding of the second order.
         * Implemented according to the paper:
         * @url: http:// #TODO
         * @param[in] pose - pose increment;
         * @param[in] cov - increment uncertainty.
         * @return void
         **/
        void compound_2nd_order(const SE3 &pose, const Mat6 &cov); // does right hand side update

        /**
         * @brief SE3pose uncertainy compounding of the fourth order.
         * Implemented according to the paper:
         * @url: http:// #TODO
         * @param[in] pose - SE3Cov object with incremental pose and increment uncertainty.
         * @return void
         */
        void compound_4th_order(const SE3Cov &pose);

        /**
         * @brief SE3pose uncertainy compounding of the fourth order.
         * Implemented according to the paper:
         * @url: http:// #TODO
         * @param[in] pose - pose increment;
         * @param[in] cov - increment uncertainty.
         * @return void
         */
        void compound_4th_order(const SE3 &pose, const Mat6 &cov);

        /** @brief Prints current state of pose and covariance.
         * @return void
         **/
        void print();

        /** @brief Multiplication method \b mul as an interface for compounding.
         *  @param[in] rhs - SE3Cov object that corresponds to increment.
         *  @return SE3Cov - new updated uncertainty SE3Cov object.
         */
        SE3Cov mul(const SE3Cov &rhs) const;

        /**
         * @brief Transforms covariance matrix to notation from Barfoot's papers
         * Self-inverse:
         * \code
         * transform_notation(transform_notation(cov)) = cov
         * \endcode
         * @param[in] cov - covariance matrix;
         * @return Mat6 - covariance matrix with permuted blocks.
         */
        static Mat6 notation_transform(const Mat6 &cov);
    };

} // end namespace

#endif // SE3COV_HPP_