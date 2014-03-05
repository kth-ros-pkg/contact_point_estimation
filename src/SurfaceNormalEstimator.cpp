/*
 *  SurfaceNormalEstimator.cpp
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


#include <contact_point_estimation/SurfaceNormalEstimator.h>
#include <eigen_utils/eigen_utils.h>


SurfaceNormalEstimator::SurfaceNormalEstimator(SurfaceNormalEstimatorParams *params)
{
	m_params = params;

	m_surface_normal_estimate = m_params->getInitialN();
	m_Ln = Matrix3d::Zero();

}

SurfaceNormalEstimator::~SurfaceNormalEstimator()
{
}


void SurfaceNormalEstimator::update(const TwistStamped &twist_ft_sensor)
{
	double gamma_n = m_params->getGammaN();
	double sne_update_frequency = m_params->getUpdateFrequency();
	Matrix3d Pbar_n = eigen_utils::orthProjMatrix(m_surface_normal_estimate);
	m_twist_ft_sensor = twist_ft_sensor;

	Vector3d vel(twist_ft_sensor.twist.linear.x, 
		     twist_ft_sensor.twist.linear.y, 
		     twist_ft_sensor.twist.linear.z);

	updateLn(vel);

	m_surface_normal_estimate = m_surface_normal_estimate - gamma_n*Pbar_n*m_Ln*m_surface_normal_estimate*(1/sne_update_frequency);
	m_surface_normal_estimate.normalize();
}


void SurfaceNormalEstimator::reset()
{
	m_surface_normal_estimate = m_params->getInitialN();
	m_Ln = Matrix3d::Zero();
}


Vector3Stamped SurfaceNormalEstimator::getEstimate() const
{
	Vector3Stamped surface_normal;
	surface_normal.header.frame_id = m_twist_ft_sensor.header.frame_id;
	surface_normal.header.stamp = ros::Time::now();
	surface_normal.vector.x = m_surface_normal_estimate(0);
	surface_normal.vector.y = m_surface_normal_estimate(1);
	surface_normal.vector.z = m_surface_normal_estimate(2);

	return surface_normal;
}

void SurfaceNormalEstimator::updateLn(const Vector3d &v_ft)
{
	double beta_n = m_params->getBetaN();
	double sne_update_frequency = m_params->getUpdateFrequency();

	m_Ln = m_Ln + (-beta_n*m_Ln + (v_ft)*((v_ft).transpose()))*(1/sne_update_frequency);
}
