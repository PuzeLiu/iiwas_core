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

#ifndef SRC_BSPLINESEGMENT_H
#define SRC_BSPLINESEGMENT_H

#include <iiwas_control/BsplineSegmentMsg.h>
#include <trajectory_interface/pos_vel_acc_state.h>
#include "tinysplinecxx.h"

template<class ScalarType>
class BsplineSegment {
public:
    typedef ScalarType Scalar;
    typedef Scalar Time;
    typedef trajectory_interface::PosVelAccState<Scalar> State;

    BsplineSegment(iiwas_control::BsplineSegmentMsg msg){
        // initialize bsplines
        q_spline_ = tinyspline::BSpline(15, 6, 7, tinyspline::BSpline::Type::Clamped);
        t_spline_ = tinyspline::BSpline(20, 1, 7, tinyspline::BSpline::Type::Clamped);
        // fill bsplines with control points
        std::vector<tinyspline::real> q_ctrlp = q_spline_.controlPoints();
        for (auto i = 0; i < msg.q_control_points.size(); i++) {
            q_ctrlp[i] = msg.q_control_points[i];
        }
        q_spline_.setControlPoints(q_ctrlp);
        std::vector<tinyspline::real> t_ctrlp = t_spline_.controlPoints();
        for (auto i = 0; i < msg.t_control_points.size(); i++) {
            t_ctrlp[i] = msg.t_control_points[i];
        }
        t_spline_.setControlPoints(t_ctrlp);
        // compute derivatives
        dq_spline_ = q_spline_.derive();
        ddq_spline_ = dq_spline_.derive();
        dt_spline_ = t_spline_.derive();
        // compute arrays that will be helpful in interpolating from time to Bspline argument
        for (int i = 0; i < 1023; i++){
            Scalar arg = (float)i / 1024.;
            std::vector<tinyspline::real> dtau_dt = t_spline_.eval(arg).result();
            ts_.push_back(ts_.back() + 1. / dtau_dt[0] / 1024.);
            args_.push_back(args_.back() + 1. / 1024.);
        }
    }

    bool sample(Time t, State& s){
        Scalar argument = interpolate(t);
        std::vector<tinyspline::real> q = q_spline_.eval(argument).result();
        std::vector<tinyspline::real> dq_tau = dq_spline_.eval(argument).result();
        std::vector<tinyspline::real> ddq_tau = ddq_spline_.eval(argument).result();
        std::vector<tinyspline::real> dtau_dt = t_spline_.eval(argument).result();
        std::vector<tinyspline::real> ddtau_dtt = dt_spline_.eval(argument).result();
        s.position = q;
        for (auto i = 0; i < q.size(); i++) {
            s.velocity[i] = (t <= 0. or t > ts_.back()) ? 0. : dq_tau[i] * dtau_dt[0];
            s.acceleration[i] = (t <= 0. or t > ts_.back()) ? 0. : ddq_tau[i] * dtau_dt[0] * dtau_dt[0] + ddtau_dtt[0] * dq_tau[i] * dtau_dt[0];
        }
        return t > ts_.back(); // return true if finished
    }

    double interpolate(Time t)
    {
        auto it = std::upper_bound(ts_.begin(), ts_.end(), t);                  // find appropriate segment
        auto i = it - ts_.begin() - 1;
        double xL = ts_[i], yL = args_[i], xR = ts_[i+1], yR = args_[i+1];      // points on either side (unless beyond ends)
        if ( t < xL ) yR = yL;                                                  // boundary clippings
        if ( t > xR ) yL = yR;
        double dydx = ( yR - yL ) / ( xR - xL );                                // gradient
        return yL + dydx * ( t - xL );                                          // linear interpolation
    }

    Scalar getDuration() {
        return ts_.back();
    }

private:
    tinyspline::BSpline q_spline_;
    tinyspline::BSpline dq_spline_;
    tinyspline::BSpline ddq_spline_;
    tinyspline::BSpline t_spline_;
    tinyspline::BSpline dt_spline_;
    std::vector<Scalar> ts_{0.};
    std::vector<Scalar> args_{0.};
};



#endif //SRC_BSPLINESEGMENT_H
