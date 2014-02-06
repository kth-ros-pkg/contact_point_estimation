/*
 *  SurfaceTracingController.cpp
 *
 *
 *  Created on: Feb 6, 2014
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

#include <contact_point_estimation/SurfaceTracingController.h>
#include <eigen_utils/eigen_utils.h>



SurfaceTracingController::SurfaceTracingController()
{
	m_f_n_error_integral = 0.0;
}

SurfaceTracingController::~SurfaceTracingController()
{

}

void SurfaceTracingController::setNormalForceCompensationGains(double alpha_p,
		double alpha_i)
{
	m_alpha_p = alpha_p;
	m_alpha_i = alpha_i;
}

void SurfaceTracingController::setTrajectoryControlGain(double alpha)
{
	m_alpha = alpha;
}

void SurfaceTracingController::setDesiredNormalForce(double f_d)
{
	m_f_d = f_d;
}

void SurfaceTracingController::setControlFrequency(double control_freq)
{
	m_control_freq = control_freq;
}

void SurfaceTracingController::reset()
{
	m_f_n_error_integral = 0.0;
}

Vector3d SurfaceTracingController::controlSignal(
		const Vector3d& surface_normal, const Matrix<double, 6, 1>ft_compensated,
		const Vector3d &p, const Vector3d& p_d, const Vector3d& p_dot_d)
{
	Vector3d force = ft_compensated.topRows(3);

	double f_n = (surface_normal.dot(force));
	double f_n_error = f_n - m_f_d;
	double dt = 1/m_control_freq;

	m_f_n_error_integral = m_f_n_error_integral + f_n_error*dt;
	double vf = m_alpha_i*m_f_n_error_integral + m_alpha_p*f_n_error;

	Vector3d v_d = p_dot_d - m_alpha*(p-p_d);
	Matrix<double, 3, 3>Pbar_n = eigen_utils::orthProjMatrix(surface_normal);

	Vector3d u = Pbar_n*v_d + surface_normal*vf;

	return u;
}
