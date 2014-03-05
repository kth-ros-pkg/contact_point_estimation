/*
 *  ContactPointEstimator.cpp
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


#include <contact_point_estimation/ContactPointEstimator.h>
#include <eigen_utils/eigen_utils.h>


ContactPointEstimator::ContactPointEstimator(ContactPointEstimatorParams *params)
{
	m_params = params;

	m_contact_point_estimate = -m_params->getInitialR();
	m_r_dot = Vector3d::Zero();
	m_Lr = Matrix3d::Zero();
	m_cr = Vector3d::Zero();

}

ContactPointEstimator::~ContactPointEstimator()
{
}

void ContactPointEstimator::update(const WrenchStamped &ft_compensated)
{
	double gamma_r = m_params->getGammaR();
	double kappa_r = m_params->getKappaR();
    double cpe_update_frequency = m_params->getUpdateFrequency();
    m_ft_compensated = ft_compensated;

	Vector3d force(ft_compensated.wrench.force.x,
			ft_compensated.wrench.force.y,
			ft_compensated.wrench.force.z);

	Vector3d torque(ft_compensated.wrench.torque.x,
			ft_compensated.wrench.torque.y,
			ft_compensated.wrench.torque.z);

	updateLr(force);

	updatecr(force, torque);

	// running average to filter out noise in the update of contact point estimate
	m_r_dot = (1/20.0)*(19.0*m_r_dot - 1.0*gamma_r*(m_Lr*m_contact_point_estimate + m_cr));

	m_contact_point_estimate = m_contact_point_estimate
			+ m_r_dot *(1/cpe_update_frequency)
			- kappa_r*m_contact_point_estimate*(1/cpe_update_frequency);
}


void ContactPointEstimator::reset()
{
	m_contact_point_estimate = -m_params->getInitialR();
	m_r_dot = Vector3d::Zero();
	m_Lr = Matrix3d::Zero();
	m_cr = Vector3d::Zero();
}

PointStamped ContactPointEstimator::getEstimate() const
{
	PointStamped contact_point;
	contact_point.header.frame_id = m_ft_compensated.header.frame_id;
	contact_point.header.stamp = ros::Time::now();
	contact_point.point.x = -m_contact_point_estimate(0);
	contact_point.point.y = -m_contact_point_estimate(1);
	contact_point.point.z = -m_contact_point_estimate(2);

	return contact_point;
}

void ContactPointEstimator::updateLr(const Vector3d& force)
{
	double beta_r = m_params->getBetaR();
	double cpe_update_frequency = m_params->getUpdateFrequency();

	Matrix3d Sf = eigen_utils::skewSymmetric(force);

	m_Lr = m_Lr + (-beta_r*m_Lr - Sf*Sf)*(1/cpe_update_frequency);
	m_Lr = 0.5*(m_Lr + m_Lr.transpose()); // to keep it symmetric
}

void ContactPointEstimator::updatecr(const Vector3d &force, const Vector3d& torque)
{
	double beta_r = m_params->getBetaR();
	double cpe_update_frequency = m_params->getUpdateFrequency();

	Matrix3d Sf = eigen_utils::skewSymmetric(force);

	m_cr = m_cr + (-beta_r*m_cr + Sf*torque)*(1/cpe_update_frequency);
}
