/*
 * ToolSurfaceCalibControl.cpp
 *
 *  Created on: Aug 19, 2013
 *      Author: fevb
 */

#include <dumbo_tool_surface_calib/ToolSurfaceCalibControl.h>
#include <dumbo_adaptive_control_math/adaptive_control_math.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>


ToolSurfaceCalibControl::ToolSurfaceCalibControl(ToolSurfaceCalibParams *params)
{
	m_params = params;
	m_tf_listener = new TransformListener();

	m_contact_point_estimate = -m_params->GetInitialR();
	m_r_dot = Vector3d::Zero();
	m_Lr = Matrix3d::Zero();
	m_Qr = Vector3d::Zero();
	m_surface_normal_estimate = m_params->GetInitialN();
	m_Ln = Matrix3d::Zero();
	m_Vf_integral = 0.0;
	m_v_ft = Vector3d::Zero();
	m_init = false;

}

ToolSurfaceCalibControl::~ToolSurfaceCalibControl()
{
	delete m_tf_listener;
}

TwistStamped ToolSurfaceCalibControl::Run(
		const WrenchStamped& FT_compensated,
		const Vector3d& vel)
{
	TwistStamped vel_screw_ft; // vel screw expressed in FT_sensor frame


	Vector3d force(FT_compensated.wrench.force.x,
			FT_compensated.wrench.force.y,
			FT_compensated.wrench.force.z);

	Vector3d torque(FT_compensated.wrench.torque.x,
				FT_compensated.wrench.torque.y,
				FT_compensated.wrench.torque.z);

	vel_screw_ft.header.frame_id = m_params->GetRobotFtFrameID();
	vel_screw_ft.twist.linear.x = 0.0;
	vel_screw_ft.twist.linear.y = 0.0;
	vel_screw_ft.twist.linear.z = 0.0;

	vel_screw_ft.twist.angular.x = 0.0;
	vel_screw_ft.twist.angular.y = 0.0;
	vel_screw_ft.twist.angular.z = 0.0;

	if(!m_init) // collect initial position of FT sensor frame expressed in base frame and initial time for trajectory
	{

		StampedTransform T_base_ft;

		try
		{
			m_tf_listener->lookupTransform(m_params->GetRobotBaseFrameID(),
					m_params->GetRobotFtFrameID(), ros::Time(0), T_base_ft);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s.", ex.what());
			ROS_ERROR("ToolSurfaceCalibControl: error getting initial position of the FT sensor frame with respect to the base frame.");
			return vel_screw_ft;
		}

		m_p_ft_0(0) = T_base_ft.getOrigin().getX();
		m_p_ft_0(1) = T_base_ft.getOrigin().getY();
		m_p_ft_0(2) = T_base_ft.getOrigin().getZ();


		m_t_traj_0 = ros::Time::now();
		m_init = true;
	}

	// get p_ft pos of FT with respect to base frame
	Vector3d p_ft;

	StampedTransform T_base_ft;

	try
	{
		m_tf_listener->lookupTransform(m_params->GetRobotBaseFrameID(),
				m_params->GetRobotFtFrameID(), ros::Time(0), T_base_ft);
	}

	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s.", ex.what());
		ROS_ERROR("ToolSurfaceCalibControl: error getting position of the FT sensor frame expressed in the base frame.");
		vel_screw_ft.header.stamp = ros::Time::now();
		return vel_screw_ft;
	}

	p_ft(0) = T_base_ft.getOrigin().getX();
	p_ft(1) = T_base_ft.getOrigin().getY();
	p_ft(2) = T_base_ft.getOrigin().getZ();


	// update contact point estimate
	UpdateLr(force);

	UpdateQr(force, torque);

	UpdateContactPointEstimate();


	// update surface normal estimate
	UpdateLn(vel);

	UpdateSurfaceNormalEstimate();


	// calculate force feedback
	double v_f;
	if(!Vf(force, v_f))
	{
		vel_screw_ft.header.stamp = ros::Time::now();
		return vel_screw_ft;
	}

	// Update arc trajectory
	UpdateCircleTrajectory(m_p_ft_d, m_v_ft_d);


	// Update Vd
	Vector3d v_d;
	v_d = Vd(p_ft, m_p_ft_d, m_v_ft_d);

	m_v_ft = Vft(v_d, v_f);


	vel_screw_ft.twist.linear.x = m_v_ft(0);
	vel_screw_ft.twist.linear.y = m_v_ft(1);
	vel_screw_ft.twist.linear.z = m_v_ft(2);

	vel_screw_ft.twist.angular.x = 0.0;
	vel_screw_ft.twist.angular.y = 0.0;
	vel_screw_ft.twist.angular.z = 0.0;

	vel_screw_ft.header.stamp = ros::Time::now();
	return vel_screw_ft;
}

void ToolSurfaceCalibControl::Reset()
{
	m_contact_point_estimate = -m_params->GetInitialR();
	m_r_dot = Vector3d::Zero();
	m_Lr = Matrix3d::Zero();
	m_Qr = Vector3d::Zero();
	m_surface_normal_estimate = m_params->GetInitialN();
	m_Ln = Matrix3d::Zero();
	m_Vf_integral = 0.0;
	m_v_ft = Vector3d::Zero();
	m_init = false;
}


void ToolSurfaceCalibControl::UpdateLr(const Vector3d& force)
{
	double beta_r = m_params->GetBetaR();
	double control_frequency = m_params->GetControlFrequency();

	Matrix3d Sf = SkewSymmetric(force);

	m_Lr = m_Lr + (-beta_r*m_Lr - Sf*Sf)*(1/control_frequency);
	m_Lr = 0.5*(m_Lr + m_Lr.transpose()); // to keep it symmetric
}

void ToolSurfaceCalibControl::UpdateQr(const Vector3d &force, const Vector3d& torque)
{
	double beta_r = m_params->GetBetaR();
	double control_frequency = m_params->GetControlFrequency();

	Matrix3d Sf = SkewSymmetric(force);

	m_Qr = m_Qr + (-beta_r*m_Qr + Sf*torque)*(1/control_frequency);
}


void ToolSurfaceCalibControl::UpdateContactPointEstimate()
{
	double gamma_r = m_params->GetGammaR();
	double kappa_r = m_params->getKappaR();
	double control_frequency = m_params->GetControlFrequency();

	// running average to filter out noise in the update of contact point estimate
	m_r_dot = (1/20.0)*(19.0*m_r_dot - 1.0*gamma_r*(m_Lr*m_contact_point_estimate + m_Qr));

	m_contact_point_estimate = m_contact_point_estimate
			+ m_r_dot *(1/control_frequency)
			- kappa_r*m_contact_point_estimate*(1/control_frequency);
}


void ToolSurfaceCalibControl::UpdateLn(const Vector3d &v_ft)
{
	double beta_n = m_params->GetBetaN();
	double control_frequency = m_params->GetControlFrequency();

	m_Ln = m_Ln + (-beta_n*m_Ln + (v_ft)*((v_ft).transpose()))*(1/control_frequency);
}

void ToolSurfaceCalibControl::UpdateSurfaceNormalEstimate()
{
	double gamma_n = m_params->GetGammaN();
	double control_frequency = m_params->GetControlFrequency();
	Matrix3d Pbar_n = OrthProjMatrix(m_surface_normal_estimate);

	m_surface_normal_estimate = m_surface_normal_estimate - gamma_n*Pbar_n*m_Ln*m_surface_normal_estimate*(1/control_frequency);
	m_surface_normal_estimate.normalize();
}



bool ToolSurfaceCalibControl::Vf(const Vector3d& force, double &v_f)
{
	// first transform surface normal to the FT sensor frame
	Vector3Stamped surface_normal_base;
	surface_normal_base.header.frame_id = m_params->GetRobotBaseFrameID();
	surface_normal_base.header.stamp = ros::Time(0);
	surface_normal_base.vector.x = m_surface_normal_estimate(0);
	surface_normal_base.vector.y = m_surface_normal_estimate(1);
	surface_normal_base.vector.z = m_surface_normal_estimate(2);

	Vector3Stamped surface_normal_ft;

	try
	{
		m_tf_listener->transformVector(m_params->GetRobotFtFrameID(),
				surface_normal_base, surface_normal_ft);
	}

	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s.", ex.what());
		ROS_ERROR("ToolSurfaceCalibControl: error transforming surface normal vector to the FT sensor frame.");
		return false;
	}

	// estimate expressed in the FT sensor frame
	Vector3d surface_normal_estimate_ft(surface_normal_ft.vector.x,
			surface_normal_ft.vector.y,
			surface_normal_ft.vector.z);

	double Fn = (double)surface_normal_estimate_ft.dot(force);
	double alpha_f = m_params->GetAlphaF();
	double beta_f = m_params->GetBetaF();
	double control_frequency = m_params->GetControlFrequency();
	m_Fn_error = (Fn - m_params->GetNormalForceRef());

	m_Vf_integral = m_Vf_integral + m_Fn_error*(1/control_frequency);

	v_f = alpha_f*m_Fn_error + beta_f*m_Vf_integral;
	return true;
}



Vector3d ToolSurfaceCalibControl::Vd(const Vector3d &p_ft,
		const Vector3d &p_ft_d, const Vector3d &v_ft_d)
{
	double alpha_v_d = m_params->GetAlphaVd();
	Vector3d v_d = v_ft_d - alpha_v_d*(p_ft-p_ft_d);

	return v_d;
}

void ToolSurfaceCalibControl::UpdateCircleTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d)
{
	// calculate the elapsed time
	ros::Duration t_ = ros::Time::now() - m_t_traj_0;
	double t = t_.toSec();

	double r = m_params->GetCircleTrajRadius();
	double T = m_params->GetCircleTrajPeriod();
	double f = 1/(T);

	Vector3d circle;

	circle(0) = r*sin(2*M_PI*f*t);
	circle(1) = -r*cos(2*M_PI*f*t) + r;
	circle(2) = -0.05; //-0.01*sin(2*M_PI*2*f*t)-0.04; // make the z component a bit beneath the table surface

	p_ft_d  = m_p_ft_0 + circle;

	v_ft_d(0) = r*2*M_PI*f*cos(2*M_PI*f*t);
	v_ft_d(1) = r*2*M_PI*f*sin(2*M_PI*f*t);
	v_ft_d(2) = 0.0; //-0.01*2*M_PI*2*f*cos(2*M_PI*2*f*t);
}

void ToolSurfaceCalibControl::UpdateLineTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d)
{
	// calculate the elapsed time
	ros::Duration t_ = ros::Time::now() - m_t_traj_0;
	double t = t_.toSec();
	double speed = m_params->GetLineTrajSpeed();
	double t1 = m_params->GetLineTrajT1();
	double t2 = m_params->GetLineTrajT2();


	if(t<0.15*t1)
	{
		p_ft_d = m_p_ft_0;
		p_ft_d(1) = p_ft_d(1) + (pow(t, 2.0)*0.5)*speed/(0.15*t1);
		p_ft_d(2) = p_ft_d(2) - 0.05;

		v_ft_d(0) = 0.0;
		v_ft_d(1) = (t)*speed/(0.15*t1);
		v_ft_d(2) = 0.0;
	}

	else if((t>=0.15*t1)&&(t<0.85*t1))
	{
		p_ft_d = m_p_ft_0;
		p_ft_d(1) = p_ft_d(1) + (0.075*t1)*speed + (t-0.15*t1)*speed;
		p_ft_d(2) = p_ft_d(2) - 0.05;

		v_ft_d(0) = 0.0;
		v_ft_d(1) = speed;
		v_ft_d(2) = 0.0;
	}

	else if ((t>=0.85*t1)&&(t<t1))
	{
		p_ft_d = m_p_ft_0;
		p_ft_d(1) = p_ft_d(1) + (0.775*t1)*speed + (t1*t - pow(t, 2.0)*0.5 - (0.85*pow(t1,2.0)-pow(0.85*t1,2.0)*0.5))*speed/(0.15*t1);
		p_ft_d(2) = p_ft_d(2) - 0.05;

		v_ft_d(0) = 0.0;
		v_ft_d(1) = (t1-t)*speed/(0.15*t1);
		v_ft_d(2) = 0.0;
	}

	else if((t>=t1)&&(t<(0.15*t2+t1)))
	{
		p_ft_d = m_p_ft_0;
		p_ft_d(0) = p_ft_d(0) + 0.866*((pow(t,2.0)*0.5)-t1*t-(pow(t1,2.0)*-0.5))*(speed/(0.15*t2));
		p_ft_d(1) = p_ft_d(1) + (0.85*t1*speed) + 0.5*((pow(t,2.0)*0.5)-t1*t-(pow(t1,2.0)*-0.5))*(speed/(0.15*t2));
		p_ft_d(2) = p_ft_d(2) - 0.05;

		v_ft_d(0) = 0.866*(t-t1)*(speed/(0.15*t2));
		v_ft_d(1) = 0.5*(t-t1)*(speed/(0.15*t2));
		v_ft_d(2) = 0.0;

	}

	else if((t>=(0.15*t2+t1)) && (t<0.85*t2+t1))
	{
		p_ft_d = m_p_ft_0;
		p_ft_d(0) = p_ft_d(0) + 0.866*(0.075*t2)*speed + 0.866*(t-(0.15*t2+t1))*speed;
		p_ft_d(1) = p_ft_d(1) + (0.85*t1*speed) + 0.5*(0.075*t2)*speed + 0.5*(t-(0.15*t2+t1))*speed;
		p_ft_d(2) = p_ft_d(2) - 0.05;

		v_ft_d(0) = 0.866*speed;
		v_ft_d(1) = 0.5*speed;
		v_ft_d(2) = 0.0;

	}

	else if(t<t1+t2)
	{
		p_ft_d = m_p_ft_0;
		p_ft_d(0) = p_ft_d(0) + 0.866*(0.775*t2)*speed +
				0.866*(((t2+t1)*t - pow(t,2.0)*0.5)-((t2+t1)*(0.85*t2+t1) - pow((0.85*t2+t1),2.0)*0.5))*speed/(0.15*t2);
		p_ft_d(1) = p_ft_d(1) + (0.85*t1*speed) + 0.5*(0.775*t2)*speed +
				0.5*(((t2+t1)*t - pow(t,2.0)*0.5)-((t2+t1)*(0.85*t2+t1) - pow((0.85*t2+t1),2.0)*0.5))*speed/(0.15*t2);
		p_ft_d(2) = p_ft_d(2) - 0.05;

		v_ft_d(0) = 0.866*((t2+t1)-t)*speed/(0.15*t2);
		v_ft_d(1) = 0.5*((t2+t1)-t)*speed/(0.15*t2);
		v_ft_d(2) = 0.0;
	}

	else
	{
		p_ft_d = m_p_ft_0;
		p_ft_d(0) = p_ft_d(0) + 0.866*(0.85*t2)*speed ;
		p_ft_d(1) =  p_ft_d(1) + (0.85*t1*speed) + 0.5*(0.85*t2)*speed ;
		p_ft_d(2) = p_ft_d(2) - 0.05;

		v_ft_d = Vector3d::Zero();
	}



}

Vector3d ToolSurfaceCalibControl::Vft(const Vector3d &v_d, double v_f)
{
	Matrix3d Pbar_n = OrthProjMatrix(m_surface_normal_estimate);
	Vector3d v_ft = Pbar_n*v_d + m_surface_normal_estimate*v_f;

	return v_ft;
}

PointStamped ToolSurfaceCalibControl::GetContactPointEstimate()
{
	PointStamped contact_point;
	contact_point.header.frame_id = m_params->GetRobotFtFrameID();
	contact_point.header.stamp = ros::Time::now();
	contact_point.point.x = -m_contact_point_estimate(0);
	contact_point.point.y = -m_contact_point_estimate(1);
	contact_point.point.z = -m_contact_point_estimate(2);

	return contact_point;
}

Vector3Stamped ToolSurfaceCalibControl::GetSurfaceNormalEstimate()
{
	Vector3Stamped surface_normal;
	surface_normal.header.frame_id = m_params->GetRobotBaseFrameID();
	surface_normal.header.stamp = ros::Time::now();
	surface_normal.vector.x = m_surface_normal_estimate(0);
	surface_normal.vector.y = m_surface_normal_estimate(1);
	surface_normal.vector.z = m_surface_normal_estimate(2);

	return surface_normal;
}

double ToolSurfaceCalibControl::GetNormalForceError()
{
	return m_Fn_error;
}


Float64MultiArray ToolSurfaceCalibControl::GetLr()
{
	Float64MultiArray Lr;

	tf::matrixEigenToMsg(m_Lr, Lr);
	return Lr;
}

Float64MultiArray ToolSurfaceCalibControl::GetQr()
{
	Float64MultiArray Qr;

	tf::matrixEigenToMsg(m_Qr, Qr);
	return Qr;
}
