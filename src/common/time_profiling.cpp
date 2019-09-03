/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * time_profiling.cpp
 *
 *  Created on: Aug 14, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#include "mrob/time_profiling.hpp"
#include <iostream>

using namespace mrob;

TimeProfiling::TimeProfiling()
{

}

TimeProfiling::~TimeProfiling()
{

}

void TimeProfiling::reset()
{
    time_profiles_.clear();
    t1_ = std::chrono::steady_clock::now();
}

void TimeProfiling::start()
{
    t1_ = std::chrono::steady_clock::now();
}

void TimeProfiling::stop(const std::string &key)
{
    auto t2 = std::chrono::steady_clock::now();
    auto dif = std::chrono::duration_cast<Ttim>(t2 - t1_);
    time_profiles_.push_back( std::make_pair(key,  dif.count()) );
}


void TimeProfiling::print()
{
    double sum = 0;
    for (auto t : time_profiles_)
        sum += t.second;

    std::cout << "\nTime profile for " << sum/1e3 << " [ms]: ";
    for (auto t : time_profiles_)
        std::cout << t.first << " = " << t.second/sum *100 << "%, ";
    std::cout << "\n";
}
