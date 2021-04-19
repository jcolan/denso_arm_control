/*
# Copyright (c) 2016
# Mitsuishi Sugita Laboratory (NML) at University of Tokyo
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#ifndef REALTIME_CLOCK_INTERFACE_HEADER_GUARD
#define REALTIME_CLOCK_INTERFACE_HEADER_GUARD

#include <ros/ros.h>

#define NSEC_TO_SEC   1000000000
#define NSEC_TO_SEC_D 1000000000.0

class RealtimeClock
{
public:

    struct timespec loop_time_;
    struct timespec before_loop_;
    struct timespec after_loop_;
    struct timespec elapsed_time_;
    struct timespec computation_time_;
    int    thread_sampling_time_nsec_;
    double thread_sampling_time_nsec_d_;
    double thread_sampling_time_sec_d_;
    double initial_time_;

    double sleep_time_d_;
    double computation_time_d_;
    double after_loop_d_;
    double before_loop_d_;


    bool* kill_this_node_;


    RealtimeClock(){}

    RealtimeClock(int thread_sampling_time_nsec)
    {
        thread_sampling_time_nsec_   = thread_sampling_time_nsec;
        thread_sampling_time_nsec_d_ = (double)thread_sampling_time_nsec_;
        thread_sampling_time_sec_d_  = thread_sampling_time_nsec_d_/NSEC_TO_SEC_D;
    }

    void init()
    {
        clock_gettime(CLOCK_MONOTONIC,&loop_time_);
        initial_time_ = (loop_time_.tv_sec)+(double(loop_time_.tv_nsec)/NSEC_TO_SEC_D) ;
    }

    void updateAndSleep()
    {
        clock_gettime(CLOCK_MONOTONIC,&before_loop_);
        before_loop_d_ = (before_loop_.tv_sec)+(double(before_loop_.tv_nsec)/NSEC_TO_SEC_D);

        timespec_diff(after_loop_,before_loop_,computation_time_);
        computation_time_d_ = (computation_time_.tv_sec)+(double(computation_time_.tv_nsec)/NSEC_TO_SEC_D);
        //Update clock and sleep
        if (loop_time_.tv_nsec + thread_sampling_time_nsec_ >= NSEC_TO_SEC)
        {
            loop_time_.tv_nsec = loop_time_.tv_nsec - NSEC_TO_SEC + thread_sampling_time_nsec_;
            loop_time_.tv_sec  = loop_time_.tv_sec  + 1;
        }
        else
            loop_time_.tv_nsec += thread_sampling_time_nsec_;
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&loop_time_,NULL);
        clock_gettime(CLOCK_MONOTONIC,&after_loop_);
        after_loop_d_ = (after_loop_.tv_sec)+(double(after_loop_.tv_nsec)/NSEC_TO_SEC_D);
        timespec_diff(before_loop_,after_loop_,elapsed_time_);
        sleep_time_d_ = (elapsed_time_.tv_sec)+(double(elapsed_time_.tv_nsec)/NSEC_TO_SEC_D);
//        printf(" B: %f  A: %f \n",before_loop_d_,after_loop_d_);
    }

    double getInitialTime()
    {
        return initial_time_;
    }

    double getSleepTime()
    {
        return sleep_time_d_;
    }

    double getLastUpdateTime()
    {
        return after_loop_d_;
    }

    double getComputationTime()
    {
        return computation_time_d_;
    }

    //Auxiliar function
    void timespec_diff(struct timespec start, struct timespec stop,
                       struct timespec& result)
    {
        if ((stop.tv_nsec - start.tv_nsec) < 0)
        {
            result.tv_sec = stop.tv_sec - start.tv_sec - 1;
            result.tv_nsec = stop.tv_nsec - start.tv_nsec + 1000000000;
        }
        else
        {
            result.tv_sec = stop.tv_sec - start.tv_sec;
            result.tv_nsec = stop.tv_nsec - start.tv_nsec;
        }

        return;
    }
};


#endif
