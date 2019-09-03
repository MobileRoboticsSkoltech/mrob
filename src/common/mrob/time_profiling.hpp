/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * time_profiling.hpp
 *
 *  Created on: Aug 14, 2019
 *      Author: Gonzalo Ferrer
 */

#ifndef TIME_PROFILING_HPP_
#define TIME_PROFILING_HPP_

#include <chrono>
#include <vector>
#include <string>


namespace mrob {
typedef std::chrono::microseconds Ttim;


/**
 * Class TimeProfiling creates a simple object that stores time
 * profiles for different functions and displays them.
 */


class TimeProfiling
{
public:
    /**
     * Constructor, no parameters reqauired
     */
    TimeProfiling();
    /**
     * Destructor, nothing to free
     */
    ~TimeProfiling();
    /**
     * Reset method
     */
    void reset();
    /**
     * start
     * TODO add here the key?
     */
    void start();
    /**
     * stop() records given the string the time spent since last start() call
     */
    void stop(const std::string &key);
    /**
     * print: displays the information gathered so far
     */
    void print();

protected:
    std::chrono::steady_clock::time_point t1_;
    std::vector<std::pair<std::string, double>> time_profiles_;
};

}


#endif /* TIME_PROFILING_HPP_ */
