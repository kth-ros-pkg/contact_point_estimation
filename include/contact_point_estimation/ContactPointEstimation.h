/*
 *  ContactPointEstimation.h
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


#ifndef CONTACTPOINTESTIMATION_H_
#define CONTACTPOINTESTIMATION_H_

#include <contact_point_estimation/ContactPointEstimationParams.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>

using namespace Eigen;
using namespace geometry_msgs;
using namespace tf;
using namespace std_msgs;

class ContactPointEstimation
{
public:
	ContactPointEstimation(ContactPointEstimationParams *params);
	virtual ~ContactPointEstimation();

	// updates the estimate of the contact point, which will be expressed in 
	// the reference frame of the force-torque measurements
	//
	// ft_compensated are gravity-compensated force-torque measurements
	virtual void updateContactPointEstimate(const WrenchStamped &ft_compensated);


	// updates the estimate of the surface normal expressed in the base frame
	// twist_ft_sensor is the twist of the FT sensor frame expressed in the base frame
	virtual void updateSurfaceNormalEstimate(const TwistStamped &twist_ft_sensor);

	// resets the estimator
	virtual void reset();


	virtual PointStamped getContactPointEstimate();

	virtual Vector3Stamped getSurfaceNormalEstimate();

protected:

	ContactPointEstimationParams *m_params;

	// expressed relative to FT sensor frame
	Vector3d m_contact_point_estimate;
	Vector3d m_r_dot;
	Matrix3d m_Lr;
	Vector3d m_cr;


	// expressed with respect to the base frame
	Vector3d m_surface_normal_estimate;
	Matrix3d m_Ln;

	bool m_init;


	// contact point estimate + Lr + cr expressed in FT sensor frame
	void updateLr(const Vector3d &force);

	void updatecr(const Vector3d &force, const Vector3d &torque);


	// velocity of ft sensor frame expressed in base frame
	virtual void updateLn(const Vector3d &v_ft);

private:





};

#endif /* CONTACTPOINTESTIMATION_H_ */
