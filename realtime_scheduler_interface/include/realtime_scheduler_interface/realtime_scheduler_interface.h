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

#ifndef REALTIME_SCHEDULER_HEADER_GUARD
#define REALTIME_SCHEDULER_HEADER_GUARD

#include <ros/ros.h>
#include <linux/types.h>
#include <sys/syscall.h>

#define SCHED_DEADLINE	6

/* XXX use the proper syscall numbers */
//Tables are foun in: ~/kernel/linux-4.4.23/arch/x86/entry/syscalls
#ifdef __x86_64__
#define __NR_sched_setattr		314
#define __NR_sched_getattr		315
#endif

#ifdef __i386__
#define __NR_sched_setattr		351
#define __NR_sched_getattr		352
#endif

#ifdef __arm__
#define __NR_sched_setattr		380
#define __NR_sched_getattr		381
#endif

class RealtimeSchedulerInterface
{
public:
    static void display_sched_attr(int policy, struct sched_param *param)
    {
        ROS_INFO("    policy=%s, priority=%d\n",
        (policy == SCHED_FIFO)  ? "SCHED_FIFO" :
        (policy == SCHED_RR)    ? "SCHED_RR" :
        (policy == SCHED_OTHER) ? "SCHED_OTHER" :
        (policy == SCHED_DEADLINE) ? "SCHED_DEADLINE" :
        "???",
        param->sched_priority);
    }

    static void display_thread_sched_attr(const char *msg)
    {
        int policy, s;
        struct sched_param param;

        s = pthread_getschedparam(pthread_self(), &policy, &param);
        if (s != 0)
            ROS_ERROR_STREAM("bla");

        ROS_INFO("%s\n", msg);
        display_sched_attr(policy, &param);
    }


    struct sched_attr {
        __u32 size;

        __u32 sched_policy;
        __u64 sched_flags;

        /* SCHED_NORMAL, SCHED_BATCH */
        __s32 sched_nice;

        /* SCHED_FIFO, SCHED_RR */
        __u32 sched_priority;

        /* SCHED_DEADLINE (nsec) */
        __u64 sched_runtime;
        __u64 sched_deadline;
        __u64 sched_period;
     };

    static int sched_setattr(pid_t pid,
          const struct sched_attr *attr,
          unsigned int flags)
    {
        return syscall(__NR_sched_setattr, pid, attr, flags);
    }

    static int sched_getattr(pid_t pid,
              struct sched_attr *attr,
              unsigned int size,
              unsigned int flags)
     {
        return syscall(__NR_sched_getattr, pid, attr, size, flags);
     }

    static void activateDeadlineScheduler(int sched_runtime, int sched_period)
    {
        struct sched_attr attr;
        int x = 0;
        int ret;
        unsigned int flags = 0;

        //printf("deadline thread started [%ld]\n", gettid());

        attr.size = sizeof(attr);
        attr.sched_flags = 0;
        attr.sched_nice = 0;
        attr.sched_priority = 0;

        /* This creates a 100us/1ms reservation */
        attr.sched_policy = SCHED_DEADLINE;
        attr.sched_runtime = sched_runtime;
        attr.sched_period = attr.sched_deadline = sched_period;

        ret = sched_setattr(0, &attr, flags);
        if (ret < 0) {
            perror("sched_setattr");
            exit(-1);
        }
    }

    static void activateFIFOScheduler(int sched_priority_)
    {
        struct sched_attr attr;
        struct sched_param parameter;
        int ret;
        unsigned int flags = 0;

        //printf("deadline thread started [%ld]\n", gettid());

        attr.size = sizeof(attr);
        attr.sched_flags = 0;
        attr.sched_nice = 0;

        attr.sched_policy = SCHED_FIFO;
        attr.sched_priority = sched_priority_;

        parameter.sched_priority = sched_priority_;

//        ret = sched_setattr(0, &attr, flags);
        ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameter);

        if (ret < 0) {
            perror("sched_setattr");
            exit(-1);
        }
    }

    static void activateRRScheduler(int sched_priority_)
    {
        struct sched_attr attr;
        struct sched_param parameter;
        int ret;
        unsigned int flags = 0;

        //printf("deadline thread started [%ld]\n", gettid());

        attr.size = sizeof(attr);
        attr.sched_flags = 0;
        attr.sched_nice = 0;

        attr.sched_policy = SCHED_RR;
        attr.sched_priority = sched_priority_;

        parameter.sched_priority = sched_priority_;

//        ret = sched_setattr(0, &attr, flags);
        ret = pthread_setschedparam(pthread_self(), SCHED_RR, &parameter);

        if (ret < 0) {
            perror("sched_setattr");
            exit(-1);
        }
    }

};

#endif
