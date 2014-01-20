/*
 *  ContactPointEstimation.cpp
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


#include <contact_point_estimation/ContactPointEstimation.h>
#include <eigen_utils/eigen_utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>


ContactPointEstimation::ContactPointEstimation(ContactPointEstimationParams *params)
{
	m_params = params;
	m_tf_listener = new TransformListener();

	m_contact_point_estimate = -m_params->getInitialR();
	m_r_dot = Vector3d::Zero();
	m_Lr = Matrix3d::Zero();
	m_Qr = Vector3d::Zero();
	m_surface_normal_estimate = m_params->getInitialN();
	m_Ln = Matrix3d::Zero();
	m_Vf_integral = 0.0;
	m_v_ft = Vector3d::Zero();
	m_init = false;

}

ContactPointEstimation::~ContactPointEstimation()
{
	delete m_tf_listener;
}

TwistStamped ContactPointEstimation::controlSignal(
		const WrenchStamped& FT_compensated,
		const Vector3d& vel)
{
  // vel screw expressed in FT_sensor frame
	TwistStamped vel_screw_ft; 


	Vector3d force(FT_compensated.wrench.force.x,
			FT_compensated.wrench.force.y,
			FT_compensated.wrench.force.z);

	Vector3d torque(FT_compensated.wrench.torque.x,
			FT_compensated.wrench.torque.y,
			FT_compensated.wrench.torque.z);

	vel_screw_ft.header.frame_id = m_params->getRobotFtFrameID();
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
			m_tf_listener->lookupTransform(m_params->getRobotBaseFrameID(),
					m_params->getRobotFtFrameID(), ros::Time(0), T_base_ft);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s.", ex.what());
			ROS_ERROR("ContactPointEstimation: error getting initial position of the FT sensor frame with respect to the base frame.");
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
		m_tf_listener->lookupTransform(m_params->getRobotBaseFrameID(),
				m_params->getRobotFtFrameID(), ros::Time(0), T_base_ft);
	}

	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s.", ex.what());
		ROS_ERROR("ContactPointEstimation: error getting position of the FT sensor frame expressed in the base frame.");
		vel_screw_ft.header.stamp = ros::Time::now();
		return vel_screw_ft;
	}

	p_ft(0) = T_base_ft.getOrigin().getX();
	p_ft(1) = T_base_ft.getOrigin().getY();
	p_ft(2) = T_base_ft.getOrigin().getZ();


	// update contact point estimate
	updateLr(force);

	updateQr(force, torque);

	updateContactPointEstimate();


	// update surface normal estimate
	updateLn(vel);

	updateSurfaceNormalEstimate();


	// calculate force feedback
	double v_f;
	if(!Vf(force, v_f))
	{
		vel_screw_ft.header.stamp = ros::Time::now();
		return vel_screw_ft;
	}

	// Update arc trajectory
	updateCircleTrajectory(m_p_ft_d, m_v_ft_d);


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

void ContactPointEstimation::reset()
{
	m_contact_point_estimate = -m_params->getInitialR();
	m_r_dot = Vector3d::Zero();
	m_Lr = Matrix3d::Zero();
	m_Qr = Vector3d::Zero();
	m_surface_normal_estimate = m_params->getInitialN();
	m_Ln = Matrix3d::Zero();
	m_Vf_integral = 0.0;
	m_v_ft = Vector3d::Zero();
	m_init = false;
}


void ContactPointEstimation::updateLr(const Vector3d& force)
{
	double beta_r = m_params->getBetaR();
	double control_frequency = m_params->getControlFrequency();

	Matrix3d Sf = eigen_utils::skewSymmetric(force);

	m_Lr = m_Lr + (-beta_r*m_Lr - Sf*Sf)*(1/control_frequency);
	m_Lr = 0.5*(m_Lr + m_Lr.transpose()); // to keep it symmetric
}

void ContactPointEstimation::updateQr(const Vector3d &force, const Vector3d& torque)
{
	double beta_r = m_params->getBetaR();
	double control_frequency = m_params->getControlFrequency();

	Matrix3d Sf = eigen_utils::skewSymmetric(force);

	m_Qr = m_Qr + (-beta_r*m_Qr + Sf*torque)*(1/control_frequency);
}


void ContactPointEstimation::updateContactPointEstimate()
{
	double gamma_r = m_params->getGammaR();
	double kappa_r = m_params->getKappaR();
	double control_frequency = m_params->getControlFrequency();

	// running average to filter out noise in the update of contact point estimate
	m_r_dot = (1/20.0)*(19.0*m_r_dot - 1.0*gamma_r*(m_Lr*m_contact_point_estimate + m_Qr));

	m_contact_point_estimate = m_contact_point_estimate
			+ m_r_dot *(1/control_frequency)
			- kappa_r*m_contact_point_estimate*(1/control_frequency);
}


void ContactPointEstimation::updateLn(const Vector3d &v_ft)
{
	double beta_n = m_params->getBetaN();
	double control_frequency = m_params->getControlFrequency();

	m_Ln = m_Ln + (-beta_n*m_Ln + (v_ft)*((v_ft).transpose()))*(1/control_frequency);
}

void ContactPointEstimation::updateSurfaceNormalEstimate()
{
	double gamma_n = m_params->getGammaN();
	double control_frequency = m_params->getControlFrequency();
	Matrix3d Pbar_n = OrthProjMatrix(m_surface_normal_estimate);

	m_surface_normal_estimate = m_surface_normal_estimate - gamma_n*Pbar_n*m_Ln*m_surface_normal_estimate*(1/control_frequency);
	m_surface_normal_estimate.normalize();
}



bool ContactPointEstimation::Vf(const Vector3d& force, double &v_f)
{
	// first transform surface normal to the FT sensor frame
	Vector3Stamped surface_normal_base;
	surface_normal_base.header.frame_id = m_params->getRobotBaseFrameID();
	surface_normal_base.header.stamp = ros::Time(0);
	surface_normal_base.vector.x = m_surface_normal_estimate(0);
	surface_normal_base.vector.y = m_surface_normal_estimate(1);
	surface_normal_base.vector.z = m_surface_normal_estimate(2);

	Vector3Stamped surface_normal_ft;

	try
	{
		m_tf_listener->transformVector(m_params->getRobotFtFrameID(),
				surface_normal_base, surface_normal_ft);
	}

	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s.", ex.what());
		ROS_ERROR("ContactPointEstimation: error transforming surface normal vector to the FT sensor frame.");
		return false;
	}

	// estimate expressed in the FT sensor frame
	Vector3d surface_normal_estimate_ft(surface_normal_ft.vector.x,
			surface_normal_ft.vector.y,
			surface_normal_ft.vector.z);

	double Fn = (double)surface_normal_estimate_ft.dot(force);
	double alpha_f = m_params->getAlphaF();
	double beta_f = m_params->getBetaF();
	double control_frequency = m_params->getControlFrequency();
	m_Fn_error = (Fn - m_params->getNormalForceRef());

	m_Vf_integral = m_Vf_integral + m_Fn_error*(1/control_frequency);

	v_f = alpha_f*m_Fn_error + beta_f*m_Vf_integral;
	return true;
}



Vector3d ContactPointEstimation::Vd(const Vector3d &p_ft,
		const Vector3d &p_ft_d, const Vector3d &v_ft_d)
{
	double alpha_v_d = m_params->getAlphaVd();
	Vector3d v_d = v_ft_d - alpha_v_d*(p_ft-p_ft_d);

	return v_d;
}

void ContactPointEstimation::updateCircleTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d)
{
	// calculate the elapsed time
	ros::Duration t_ = ros::Time::now() - m_t_traj_0;
	double t = t_.toSec();

	double r = m_params->getCircleTrajRadius();
	double T = m_params->getCircleTrajPeriod();
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

void ContactPointEstimation::updateLineTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d)
{
	// calculate the elapsed time
	ros::Duration t_ = ros::Time::now() - m_t_traj_0;
	double t = t_.toSec();
	double speed = m_params->getLineTrajSpeed();
	double t1 = m_params->getLineTrajT1();
	double t2 = m_params->getLineTrajT2();


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

Vector3d ContactPointEstimation::Vft(const Vector3d &v_d, double v_f)
{
  Matrix3d Pbar_n = eigen_utils::orthProjMatrix(m_surface_normal_estimate);
  Vector3d v_ft = Pbar_n*v_d + m_surface_normal_estimate*v_f;

  return v_ft;
}

PointStamped ContactPointEstimation::getContactPointEstimate()
{
	PointStamped contact_point;
	contact_point.header.frame_id = m_params->getRobotFtFrameID();
	contact_point.header.stamp = ros::Time::now();
	contact_point.point.x = -m_contact_point_estimate(0);
	contact_point.point.y = -m_contact_point_estimate(1);
	contact_point.point.z = -m_contact_point_estimate(2);

	return contact_point;
}

Vector3Stamped ContactPointEstimation::getSurfaceNormalEstimate()
{
	Vector3Stamped surface_normal;
	surface_normal.header.frame_id = m_params->getRobotBaseFrameID();
	surface_normal.header.stamp = ros::Time::now();
	surface_normal.vector.x = m_surface_normal_estimate(0);
	surface_normal.vector.y = m_surface_normal_estimate(1);
	surface_normal.vector.z = m_surface_normal_estimate(2);

	return surface_normal;
}

double ContactPointEstimation::getNormalForceError()
{
	return m_Fn_error;
}


Float64MultiArray ContactPointEstimation::getLr()
{
	Float64MultiArray Lr;

	tf::matrixEigenToMsg(m_Lr, Lr);
	return Lr;
}

Float64MultiArray ContactPointEstimation::getQr()
{
	Float64MultiArray Qr;

	tf::matrixEigenToMsg(m_Qr, Qr);
	return Qr;
}
