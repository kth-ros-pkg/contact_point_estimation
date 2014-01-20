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

	// executes the controller
	// calculates the velocity screw of FT sensor frame expressed in the base frame)
	// and updates the estimates of contact point and surface normal
	// FT_compensated given in FT sensor frame
	// vel is the estimated velocity of the 
	virtual TwistStamped controlSignal(const WrenchStamped &ft_compensated, const Vector3d &vel);

	// updates the estimate of the contact point (expressed in the ft sensor frame)
	// 
	// ft_compensated are gravity-compensated force-torque measurements which
	// should be expressed in the ft sensor frame
	virtual void updateContactPointEstimate(const WrenchStamped &ft_compensated);


	// updates the estimate of the surface normal expressed in the base frame
	virtual void updateSurfaceNormalEstimate(const Vector3d &vel);

	// resets the controller (integrators and trajectory generators)
	virtual void reset();


	virtual PointStamped getContactPointEstimate();

	virtual Vector3Stamped getSurfaceNormalEstimate();

	virtual double getNormalForceError();


	virtual Float64MultiArray getLr();

	virtual Float64MultiArray getQr();



protected:

	ContactPointEstimationParams *m_params;

	// expressed relative to FT sensor frame
	Vector3d m_contact_point_estimate;
	Vector3d m_r_dot;
	Matrix3d m_Lr;
	Vector3d m_Qr;


	// expressed with respect to the base frame
	Vector3d m_surface_normal_estimate;
	Matrix3d m_Ln;

	// normal force error
	double m_Fn_error;

	// expressed in ft sensor frame
	double m_Vf_integral;

	// reference velocity of FT sensor expressed in the base frame
	Vector3d m_v_ft;

	// starting time of the trajectory
	ros::Time m_t_traj_0;

	// initial position of ft sensor with respect to base frame
	Vector3d m_p_ft_0;

	// set points from the trajectory generator.
	// pos+vel of the FT sensor frame with respect to the base frame
	Vector3d m_p_ft_d;
	Vector3d m_v_ft_d;

	TransformListener *m_tf_listener;

	bool m_init;


	// contact point estimate + Lr + Qr expressed in FT sensor frame
	void updateLr(const Vector3d &force);

	void updateQr(const Vector3d &force, const Vector3d &torque);


	// velocity of ft sensor frame expressed in base frame
	virtual void updateLn(const Vector3d &v_ft);


	// PI force feedback velocity , everything expressed in FT sensor frame
	virtual bool Vf(const Vector3d &force, double &v_f);



	// Vd tracks the trajectory that ft sensor frame will follow, expressed in the base frame
	// In this case we do an arc trajectory
	// p_ft : current pos of FT sensor frame expressed in base frame
	// p_ft_d/v_ft_d: desired pos/vel of FT frame given by trajectory generator
	virtual Vector3d Vd(const Vector3d &p_ft,
			const Vector3d &p_ft_d, const Vector3d &v_ft_d);

	// desired FT sensor pos and vel
	// makes an arc trajectory
	// p_ft_d/v_ft_d: desired pos/vel of FT frame expressed in the base frame
	virtual void updateCircleTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d);



	// desired FT sensor pos and vel
	// makes an arc trajectory
	// p_ft_d/v_ft_d: desired pos/vel of FT frame expressed in the base frame
	//todo implement this function
	virtual void updateLineTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d);

	// calculates reference velocity of ft sensor frame with respect to the base frame
	virtual Vector3d Vft(const Vector3d &v_d, double v_f);

private:





};

#endif /* CONTACTPOINTESTIMATION_H_ */
