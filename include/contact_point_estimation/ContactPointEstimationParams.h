/*
 * ToolSurfaceCalibParams.h
 *
 *  Created on: Aug 19, 2013
 *      Author: fevb
 */

#ifndef TOOLSURFACECALIBPARAMS_H_
#define TOOLSURFACECALIBPARAMS_H_

#include <eigen3/Eigen/Core>
using namespace Eigen;

class ToolSurfaceCalibParams
{
public:
	ToolSurfaceCalibParams();
	virtual ~ToolSurfaceCalibParams();

	// gamma_r is the gain of the contact point (r) estimator
	double GetGammaR() const;
	void SetGammaR(double gamma_r);

	// gamma_r gain of the robustifying term of the contact point estimator
	double getKappaR() const;
	void setKappaR(double kappa_r);

	// beta_r is the gain for Lr and Qr in the contact point estimator
	double GetBetaR() const;
	void SetBetaR(double beta_r);

	// initial contact point (r) expressed in FT sensor frame
	Vector3d GetInitialR() const;
	void SetInitialR(const Vector3d &initial_r);

	// gamma_n is the gain of the surface normal (n) estimator
	double GetGammaN() const;
	void SetGammaN(double gamma_n);

	// beta_n is the gain of the Ln in the surface normal estimator
	double GetBetaN() const;
	void SetBetaN(double beta_n);

	// initial surface normal (n) expressed in the base frame
	Vector3d GetInitialN() const;
	void SetInitialN(const Vector3d &initial_n);

	// force reference in the normal direction of the surface
	double GetNormalForceRef() const;
	void SetNormalForceRef(double normal_force_ref);

	// control gains (alpha -- P gain, beta -- I gain)for PI force feedback (vf)
	double GetAlphaF() const;
	void SetAlphaF(double alpha_f);

	double GetBetaF() const;
	void SetBetaF(double beta_f);

	double GetControlFrequency() const;
	void SetControlFrequency(double control_frequency);

	double GetAlphaVd() const;
	void SetAlphaVd(double alpha_v_d);

	double GetCircleTrajRadius() const;
	void SetCircleTrajRadius(double circle_traj_radius);

	double GetCircleTrajPeriod() const;
	void SetCircleTrajPeriod(double circle_traj_period);

	double GetLineTrajSpeed() const;
	void SetLineTrajSpeed(double line_traj_speed);

	double GetLineTrajT1() const;
	void SetLineTrajT1(double line_traj_t1);

	double GetLineTrajT2() const;
	void SetLineTrajT2(double line_traj_t2);

	// robot frame IDs
	std::string GetRobotBaseFrameID() const;
	void SetRobotBaseFrameID(const std::string &robot_base_frame_id);

	std::string GetRobotFtFrameID() const;
	void SetRobotFtFrameID(const std::string &robot_ft_frame_id);



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

#endif /* TOOLSURFACECALIBPARAMS_H_ */
