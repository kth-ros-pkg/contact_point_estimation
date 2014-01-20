/*
 *  ContactPointEstimationParams.h
 *
 *
 *  Created on: Jan 14, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef CONTACTPOINTESTIMATIONPARAMS_H_
#define CONTACTPOINTESTIMATIONPARAMS_H_

#include <eigen3/Eigen/Core>
using namespace Eigen;

class ContactPointEstimationParams
{
public:
	ContactPointEstimationParams();
	virtual ~ContactPointEstimationParams();

	// gamma_r is the gain of the contact point (r) estimator
	double getGammaR() const;
	void setGammaR(double gamma_r);

	// gamma_r gain of the robustifying term of the contact point estimator
	double getKappaR() const;
	void setKappaR(double kappa_r);

	// beta_r is the gain for Lr and Qr in the contact point estimator
	double getBetaR() const;
	void setBetaR(double beta_r);

	// initial contact point (r) expressed in FT sensor frame
	Vector3d getInitialR() const;
	void setInitialR(const Vector3d &initial_r);

	// gamma_n is the gain of the surface normal (n) estimator
	double getGammaN() const;
	void setGammaN(double gamma_n);

	// beta_n is the gain of the Ln in the surface normal estimator
	double getBetaN() const;
	void setBetaN(double beta_n);

	// initial surface normal (n) expressed in the base frame
	Vector3d getInitialN() const;
	void setInitialN(const Vector3d &initial_n);

	// force reference in the normal direction of the surface
	double getNormalForceRef() const;
	void setNormalForceRef(double normal_force_ref);

	// control gains (alpha -- P gain, beta -- I gain)for PI force feedback (vf)
	double getAlphaF() const;
	void setAlphaF(double alpha_f);

	double getBetaF() const;
	void setBetaF(double beta_f);

	double getControlFrequency() const;
	void setControlFrequency(double control_frequency);

	double getAlphaVd() const;
	void setAlphaVd(double alpha_v_d);

	double getCircleTrajRadius() const;
	void setCircleTrajRadius(double circle_traj_radius);

	double getCircleTrajPeriod() const;
	void setCircleTrajPeriod(double circle_traj_period);

	double getLineTrajSpeed() const;
	void setLineTrajSpeed(double line_traj_speed);

	double getLineTrajT1() const;
	void setLineTrajT1(double line_traj_t1);

	double getLineTrajT2() const;
	void setLineTrajT2(double line_traj_t2);

	// robot frame IDs
	std::string getRobotBaseFrameID() const;
	void setRobotBaseFrameID(const std::string &robot_base_frame_id);

	std::string getRobotFtFrameID() const;
	void setRobotFtFrameID(const std::string &robot_ft_frame_id);



private:


	double m_gamma_r;
	double m_beta_r;

	double m_kappa_r;

	Vector3d m_initial_r;

	double m_gamma_n;
	double m_beta_n;

	Vector3d m_initial_n;

	double m_normal_force_ref;

	double m_alpha_f;
	double m_beta_f;

	double m_alpha_v_d;
	double m_circle_traj_radius;
	double m_circle_traj_period;

	double m_line_traj_speed;
	double m_line_traj_t1;
	double m_line_traj_t2;

	double m_control_frequency;

	// frame IDs
	std::string m_robot_base_frame_id;
	std::string m_robot_ft_frame_id; // force-torque sensor

};

#endif /* CONTACTPOINTESTIMATIONPARAMS_H_ */
