/*
 * ToolSurfaceCalibParams.cpp
 *
 *  Created on: Aug 19, 2013
 *      Author: fevb
 */

#include <dumbo_tool_surface_calib/ToolSurfaceCalibParams.h>

ToolSurfaceCalibParams::ToolSurfaceCalibParams()
{
	// TODO Auto-generated constructor stub
	m_kappa_r = 0.0;

}

ToolSurfaceCalibParams::~ToolSurfaceCalibParams()
{
	// TODO Auto-generated destructor stub
}


double ToolSurfaceCalibParams::GetGammaR() const
{
	return m_gamma_r;
}

void ToolSurfaceCalibParams::SetGammaR(double gamma_r)
{
	m_gamma_r = gamma_r;
}

double ToolSurfaceCalibParams::getKappaR() const {
	return m_kappa_r;
}

void ToolSurfaceCalibParams::setKappaR(double kappa_r) {
	m_kappa_r = kappa_r;
}


double ToolSurfaceCalibParams::GetBetaR() const
{
	return m_beta_r;
}

void ToolSurfaceCalibParams::SetBetaR(double beta_r)
{
	m_beta_r = beta_r;
}



Vector3d ToolSurfaceCalibParams::GetInitialR() const
{
	return m_initial_r;
}

void ToolSurfaceCalibParams::SetInitialR(const Vector3d &initial_r)
{
	m_initial_r = initial_r;
}


double ToolSurfaceCalibParams::GetGammaN() const
{
	return m_gamma_n;
}

void ToolSurfaceCalibParams::SetGammaN(double gamma_n)
{
	m_gamma_n = gamma_n;
}

double ToolSurfaceCalibParams::GetBetaN() const
{
	return m_beta_n;
}

void ToolSurfaceCalibParams::SetBetaN(double beta_n)
{
	m_beta_n = beta_n;
}

Vector3d ToolSurfaceCalibParams::GetInitialN() const
{
	return m_initial_n;
}

void ToolSurfaceCalibParams::SetInitialN(const Vector3d &initial_n)
{
	m_initial_n = initial_n.normalized();
}




double ToolSurfaceCalibParams::GetNormalForceRef() const
{
	return m_normal_force_ref;
}

void ToolSurfaceCalibParams::SetNormalForceRef(double normal_force_ref)
{
	m_normal_force_ref = normal_force_ref;
}

double ToolSurfaceCalibParams::GetAlphaF() const
{
	return m_alpha_f;
}

void ToolSurfaceCalibParams::SetAlphaF(double alpha_f)
{
	m_alpha_f = alpha_f;
}

double ToolSurfaceCalibParams::GetBetaF() const
{
	return m_beta_f;
}

void ToolSurfaceCalibParams::SetBetaF(double beta_f)
{
	m_beta_f = beta_f;
}

double ToolSurfaceCalibParams::GetControlFrequency() const
{
	return m_control_frequency;
}

void ToolSurfaceCalibParams::SetControlFrequency(double control_frequency)
{
	m_control_frequency = control_frequency;
}


double ToolSurfaceCalibParams::GetAlphaVd() const
{
	return m_alpha_v_d;
}

void ToolSurfaceCalibParams::SetAlphaVd(double alpha_v_d)
{
	m_alpha_v_d = alpha_v_d;
}

double ToolSurfaceCalibParams::GetCircleTrajRadius() const
{
	return m_circle_traj_radius;
}

void ToolSurfaceCalibParams::SetCircleTrajRadius(double circle_traj_radius)
{
	m_circle_traj_radius = circle_traj_radius;
}


double ToolSurfaceCalibParams::GetCircleTrajPeriod() const
{
	return m_circle_traj_period;
}

void ToolSurfaceCalibParams::SetCircleTrajPeriod(double circle_traj_period)
{
	m_circle_traj_period = circle_traj_period;
}

double ToolSurfaceCalibParams::GetLineTrajSpeed() const {
	return m_line_traj_speed;
}

void ToolSurfaceCalibParams::SetLineTrajSpeed(double line_traj_speed) {
	m_line_traj_speed = line_traj_speed;
}

double ToolSurfaceCalibParams::GetLineTrajT1() const {
	return m_line_traj_t1;
}

void ToolSurfaceCalibParams::SetLineTrajT1(double line_traj_t1) {
	m_line_traj_t1 = line_traj_t1;
}

double ToolSurfaceCalibParams::GetLineTrajT2() const {
	return m_line_traj_t2;
}

void ToolSurfaceCalibParams::SetLineTrajT2(double line_traj_t2) {
	m_line_traj_t2 = line_traj_t2;
}



std::string ToolSurfaceCalibParams::GetRobotBaseFrameID() const
{
	return m_robot_base_frame_id;
}

void ToolSurfaceCalibParams::SetRobotBaseFrameID(const std::string &robot_base_frame_id)
{
	m_robot_base_frame_id = robot_base_frame_id;
}

std::string ToolSurfaceCalibParams::GetRobotFtFrameID() const
{
	return m_robot_ft_frame_id;
}

void ToolSurfaceCalibParams::SetRobotFtFrameID(const std::string &robot_ft_frame_id)
{
	m_robot_ft_frame_id = robot_ft_frame_id;
}












