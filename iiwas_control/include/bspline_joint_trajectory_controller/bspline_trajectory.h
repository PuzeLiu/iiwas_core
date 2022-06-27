//
// Created by piotr on 27.06.2022.
//

#ifndef SRC_BSPLINETRAJECTORY_H
#define SRC_BSPLINETRAJECTORY_H

#include <air_hockey_neural_planner/BsplineTrajectoryMsg.h>
#include <trajectory_interface/pos_vel_acc_state.h>
#include "tinysplinecxx.h"

template<class ScalarType>
class BsplineTrajectory {
public:
    typedef ScalarType Scalar;
    typedef Scalar Time;
    typedef trajectory_interface::PosVelAccState<Scalar> State;

    BsplineTrajectory(air_hockey_neural_planner::BsplineTrajectoryMsg msg){
        std::cout << "[RUNNING]     BsplineTrajectory constructor" << std::endl;
        std::cout << "Q_CPS" << std::endl;
        for (auto x: msg.q_control_points) {
            std::cout << x << "  ";
        }
        std::cout << std::endl;

        q_spline_ = tinyspline::BSpline(15, 6, 7, tinyspline::BSpline::Type::Clamped);
        t_spline_ = tinyspline::BSpline(20, 1, 7, tinyspline::BSpline::Type::Clamped);
        std::cout << "[RUNNING]     tinyspline initialized" << std::endl;
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
        //std::cout << "T CPS" << std::endl;
        //for (auto x: t_spline_.controlPoints()) {
        //    std::cout << x << "  ";
        //}
        //std::cout << std::endl;
        std::cout << "[RUNNING]     tinyspline filled" << std::endl;
        dq_spline_ = q_spline_.derive();
        ddq_spline_ = dq_spline_.derive();
        dt_spline_ = t_spline_.derive();
        start_time_ = msg.header.stamp.toSec();
        std::cout << "[RUNNING]     tinyspline derived" << std::endl;
        ts_.push_back(0.);
        args_.push_back(0.);
        for (int i = 0; i < 1023; i++){
            Scalar arg = (float)i / 1024.;
            std::vector<tinyspline::real> dtau_dt = t_spline_.eval(arg).result();
//            std::cout << i << "  DTAU: " << dtau_dt[0] << std::endl;
            ts_.push_back(ts_.back() + 1. / dtau_dt[0] / 1024.);
            args_.push_back(args_.back() + 1. / 1024.);
        }
        //std::cout << "T VECTOR" << std::endl;
        //for (auto t: ts_) {
        //    std::cout << t << "  ";
        //}
        //std::cout << std::endl;
    }

    bool sample(Time t, State& s){
//        std::cout << "[RUNNING]     sampling started" << std::endl;
        Scalar argument = interpolate(t - start_time_);
        std::cout << "BSpline argument: " << argument << std::endl;
        std::vector<tinyspline::real> q = q_spline_.eval(argument).result();
        std::vector<tinyspline::real> dq_tau = dq_spline_.eval(argument).result();
        std::vector<tinyspline::real> ddq_tau = ddq_spline_.eval(argument).result();
        std::vector<tinyspline::real> dtau_dt = t_spline_.eval(argument).result();
        std::vector<tinyspline::real> ddtau_dtt = t_spline_.eval(argument).result();
//        std::cout << "[RUNNING]     sampling ended" << std::endl;
//        std::cout << "[RUNNING]     multiplications" << std::endl;
        s.position = q;
        for (auto i = 0; i < q.size(); i++) {
            s.velocity[i] = dq_tau[i] * dtau_dt[i];
            s.acceleration[i] = ddq_tau[i] * dtau_dt[i] * dtau_dt[i] + ddtau_dtt[i] * dq_tau[i] * dtau_dt[i];
        }
//        std::cout << "[RUNNING]     end of multiplications" << std::endl;
        return (t - start_time_) > ts_[ts_.size() - 1]; // return true if finished
    }

    double interpolate(Time t)
    {
//        std::cout << "[RUNNING] INTERPOLATION STARTED" << std::endl;
        auto it = std::upper_bound(ts_.begin(), ts_.end(), t);
        auto i = it - ts_.begin() - 1;
        double xL = ts_[i], yL = args_[i], xR = ts_[i+1], yR = args_[i+1];      // points on either side (unless beyond ends)
        std::cout << "INTERPOLATE VALUES" << std::endl;
        std::cout << t << " " << xL << " " << xR << " " << std::endl;

        if ( t < xL ) yR = yL;
        if ( t > xR ) yL = yR;

        double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

//        std::cout << "[RUNNING] INTERPOLATION ENDED" << std::endl;
        return yL + dydx * ( t - xL );                                              // linear interpolation
    }

private:
    tinyspline::BSpline q_spline_;
    tinyspline::BSpline dq_spline_;
    tinyspline::BSpline ddq_spline_;
    tinyspline::BSpline t_spline_;
    tinyspline::BSpline dt_spline_;
    Time start_time_;
    Time end_time_;
    std::vector<Scalar> ts_;
    std::vector<Scalar> args_;

};



#endif //SRC_BSPLINETRAJECTORY_H
