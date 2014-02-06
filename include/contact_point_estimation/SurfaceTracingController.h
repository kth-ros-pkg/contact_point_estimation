/*
 *  SurfaceTracingController.h
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

#ifndef SURFACETRACINGCONTROLLER_H_
#define SURFACETRACINGCONTROLLER_H_

#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_kdl.h>

using namespace Eigen;

// Surface tracing controller with PI force compensation
class SurfaceTracingController
{
public:
	SurfaceTracingController();
	virtual ~SurfaceTracingController();

	// set controller parameters
	void setNormalForceCompensationGains(double alpha_p, double alpha_i);
	void setTrajectoryControlGain(double alpha);
	void setDesiredNormalForce(double f_d);
	void setControlFrequency(double control_freq);

	// resets the controller (integrators)
	void reset();

	// Calculates the control signal (translational velocity) of the force-torque sensor for surface tracing.
	// Rotational velocity is not considered, must be set to zero when sending commands to the robot.
	// The twist is expressed in same reference frame as the inputs, thus it is
	// assumed that all input arguments are expressed in the same frame (e.g. the robot base frame)
	//
	// surface_normal: 3D vector representing the surface normal.
	//                 Can be estimated through the contact point estimator class.
	// ft_compensated: gravity compensated force-torque measurement.
	// p: current position of the force-torque sensor
	// p_d: desired FT sensor position. Comes from trajectory generator. Only translational part is used.
	// p_dot_d: desired FT sensor twist. Comes from trajectory generator. Only translational part is used.
	Vector3d controlSignal(const Vector3d &surface_normal,
			const Matrix<double, 6, 1> ft_compensated,
			const Vector3d &p,
			const Vector3d &p_d,
			const Vector3d &p_dot_d);


private:

	// controller parameters
	double m_alpha_p;
	double m_alpha_i;
	double m_alpha;
	double m_f_d;
	double m_control_freq;

	double m_f_n_error_integral;

};

#endif /* SURFACETRACINGCONTROLLER_H_ */
