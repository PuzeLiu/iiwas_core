/*
 * MIT License
 * Copyright (c) 2021 Puze Liu, Davide Tateo
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

#ifndef ADRC_EXTENDED_STATE_OBSERVER_H
#define ADRC_EXTENDED_STATE_OBSERVER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_buffer.h>
#include <boost/thread/mutex.hpp>

#include "iiwas_control/ParametersConfig.h"


namespace adrc_controllers{
	struct ADRCGains{
		ADRCGains(){};

		ADRCGains(double b, double omega_c, double omega_o, double k){
			setParam(b, omega_c, omega_o, k);
		}

		void setParam(double b, double omega_c, double omega_o, double k);

		double Kp_;
		double Kd_;
		double b_;
		double beta1_;
		double beta2_;
		double beta3_;
		double omega_c_;
		double omega_o_;
		double k_;
	};

	class ADRCJoint{
	public:
		ADRCJoint(const ADRCJoint &source);

		ADRCJoint(double b=4.0, double omega_c=50.0, double omega_o=200.0, double k=60.0);

		void setGains(double b, double omega_c, double omega_o, double k);
		void setGains(const ADRCGains &gains);

		void getGains(double &b, double &omega_c, double &omega_o, double &k, double &Kp, double &Kd,
					  double &beta_1, double &beta_2, double &beta_3);

		bool init(const ros::NodeHandle& joint_nh, double h);

		double starting(double x);

		double starting();

		double update(double y, double x_r, double v_r, double inertia);

		void initDynamicReconfig(ros::NodeHandle &node);

		void updateDynamicReconfig(iiwas_control::ParametersConfig config);
		void updateDynamicReconfig(ADRCGains gains);

		void dynamicReconfigCallback(iiwas_control::ParametersConfig &config, uint32_t);


	public:
		double z1, z2, z3;
		double x_d, u_old, h;

	protected:
		realtime_tools::RealtimeBuffer<ADRCGains> adrc_buffer_;

		// Dynamics reconfigure
		bool dynamic_reconfig_initialized_;
		typedef dynamic_reconfigure::Server<iiwas_control::ParametersConfig> DynamicReconfigServer;
		boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
		DynamicReconfigServer::CallbackType param_reconfig_callback_;

		boost::recursive_mutex param_reconfig_mutex_;
	};
}
#endif //ADRC_EXTENDED_STATE_OBSERVER_H
