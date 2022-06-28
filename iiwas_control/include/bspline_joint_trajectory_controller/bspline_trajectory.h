/*
 * MIT License
 * Copyright (c) 2022 Piotr Kicki
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef SRC_BSPLINETRAJECTORY_H
#define SRC_BSPLINETRAJECTORY_H

#include <iiwas_control/BsplineTrajectoryMsg.h>
#include <trajectory_interface/pos_vel_acc_state.h>
#include "bspline_segment.h"

template<class ScalarType>
class BsplineTrajectory {
public:
    typedef ScalarType Scalar;
    typedef Scalar Time;
    typedef trajectory_interface::PosVelAccState<Scalar> State;
    typedef BsplineSegment<Scalar> Segment;

    BsplineTrajectory(iiwas_control::BsplineTrajectoryMsg msg){
        time_offset_ = msg.header.stamp.toSec();
        for (int i = 0; i < msg.segments.size(); i++) {
            segments_.push_back(Segment(msg.segments[i]));
            times_accumulated_.push_back(times_accumulated_.back() + segments_.back().getDuration());
        }
    }

    bool sample(Time t, State& s){
        t = t - time_offset_; // start trajectory after the time offset
        auto it = std::upper_bound(times_accumulated_.begin(), times_accumulated_.end(), t); // find current segment
        auto i = std::max(it - times_accumulated_.begin() - 1., 0.); // get segment index
        if (i >= segments_.size()) return true; // return true if finished
        segments_[i].sample(t - times_accumulated_[i], s); // sample current segment
        return t > times_accumulated_.back(); // return true if finished
    }

private:
    std::vector<Segment> segments_;
    std::vector<Time> times_accumulated_{0.};
    Time time_offset_;

};



#endif //SRC_BSPLINETRAJECTORY_H
