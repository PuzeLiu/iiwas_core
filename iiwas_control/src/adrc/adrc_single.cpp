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

#include <cmath>
#include "adrc/adrc_single.h"
#include <boost/algorithm/clamp.hpp>

void adrc_controllers::ADRCGains::setParam(double b, double omega_c, double omega_o) {
	b_ = b;
	omega_c_ = omega_c;
	omega_o_ = omega_o;

	Kp_ = pow(omega_c_, 2);
	Kd_ = 2 * omega_c_;
	beta1_ = 3 * omega_o_;
	beta2_ = 3 * pow(omega_o_, 2);
	beta3_ = pow(omega_o_, 3);
}

adrc_controllers::ADRCJoint::ADRCJoint(const adrc_controllers::ADRCJoint &source): dynamicReconfigInitialized_(false),
                                                                                   h(0.001) {
	adrcBuffer_ = source.adrcBuffer_;
}

adrc_controllers::ADRCJoint::ADRCJoint(double b, double omega_c, double omega_o) :
		dynamicReconfigInitialized_(false), h(0.001) {
	setGains(b, omega_c, omega_o);
}

void adrc_controllers::ADRCJoint::setGains(double b, double omega_c, double omega_o) {
	ADRCGains gains(b, omega_c, omega_o);
	setGains(gains);
}

void adrc_controllers::ADRCJoint::setGains(const adrc_controllers::ADRCGains &gains) {
	adrcBuffer_.writeFromNonRT(gains);
	updateDynamicReconfig(gains);
}

void adrc_controllers::ADRCJoint::getGains(double &b, double &omega_c, double &omega_o, double &Kp, double &Kd,
										   double &beta_1, double &beta_2, double &beta_3) {
	ADRCGains gains = *adrcBuffer_.readFromNonRT();
	b = gains.b_;
	omega_c = gains.omega_c_;
	omega_o = gains.omega_o_;
	Kp = gains.Kp_;
	Kd = gains.Kd_;
	beta_1 = gains.beta1_;
	beta_2 = gains.beta2_;
	beta_3 = gains.beta3_;
}

double adrc_controllers::ADRCJoint::starting(double x) {
	z1 = x;
	z2 = 0.;
	z3 = 0.;
	error = 0.;
	uOld = 0.;
	return 0;
}

double adrc_controllers::ADRCJoint::starting() {
	return starting(0.);
}

double adrc_controllers::ADRCJoint::update(double y, double x_r, double v_r) {
	ADRCGains gains = *adrcBuffer_.readFromNonRT();

	error = y - z1;

	z1 += h * (z2 + gains.beta1_ * error);
	z2 += h * (z3 + gains.b_ * uOld + gains.beta2_ * error);
	z3 += h * (gains.beta3_ * error);

	z1 = boost::algorithm::clamp(z1, qLow, qHigh);
	z2 = boost::algorithm::clamp(z2, -vMax, vMax);
    z3 = boost::algorithm::clamp(z3, -vMax / h, vMax / h);
	uOld = (gains.Kp_ * (x_r - y) + gains.Kd_ * (v_r - z2) - z3) / gains.b_;
	uOld = boost::algorithm::clamp(uOld, -uMax, uMax);
	return uOld;
}

bool adrc_controllers::ADRCJoint::init(const ros::NodeHandle& joint_nh, double timeStep,
									   double low, double high, double v_max) {
	ros::NodeHandle nh(joint_nh);

	h = timeStep;
	qLow = low;
	qHigh = high;
	vMax = v_max;
	double b, omega_c, omega_o, k;
	if (!nh.getParam("b", b)){
		ROS_ERROR("No b specified for ADRC.  Namespace: %s", nh.getNamespace().c_str());
		return false;
	}
	if (!nh.getParam("omega_c", omega_c)){
		ROS_ERROR("No omega_c specified for ADRC.  Namespace: %s", nh.getNamespace().c_str());
		return false;
	}
	if (!nh.getParam("omega_o", omega_o)){
		ROS_ERROR("No omega_o specified for ADRC.  Namespace: %s", nh.getNamespace().c_str());
		return false;
	}
	if (!nh.getParam("u_max", uMax)){
		ROS_ERROR("No uMax specified for ADRC.  Namespace: %s", nh.getNamespace().c_str());
		return false;
	}

	setGains(b, omega_c, omega_o);
	initDynamicReconfig(nh);

	return true;
}

void adrc_controllers::ADRCJoint::initDynamicReconfig(ros::NodeHandle &node) {
	paramReconfigServer_.reset(new DynamicReconfigServer(paramReconfigMutex_, node));
	dynamicReconfigInitialized_ = true;

	paramReconfigCallback_ = boost::bind(&adrc_controllers::ADRCJoint::dynamicReconfigCallback, this, _1, _2);
	paramReconfigServer_->setCallback(paramReconfigCallback_);
}

void adrc_controllers::ADRCJoint::updateDynamicReconfig(adrc_controllers::ADRCGains gains) {
	if(!dynamicReconfigInitialized_)
		return;
	iiwas_control::ParametersConfig config;
	config.b = gains.b_;
	config.omega_c = gains.omega_c_;
	config.omega_o = gains.omega_o_;

	updateDynamicReconfig(config);
}

void adrc_controllers::ADRCJoint::updateDynamicReconfig(iiwas_control::ParametersConfig config) {
	if(!dynamicReconfigInitialized_)
		return;

	paramReconfigMutex_.lock();
	paramReconfigServer_->updateConfig(config);
	paramReconfigMutex_.unlock();
}

void adrc_controllers::ADRCJoint::dynamicReconfigCallback(iiwas_control::ParametersConfig &config, uint32_t) {
	setGains(config.b, config.omega_c, config.omega_o);
}











