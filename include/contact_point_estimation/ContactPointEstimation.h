/*
 * ToolSurfaceCalibControl.h
 *
 *  Created on: Aug 19, 2013
 *      Author: fevb
 */

#ifndef TOOLSURFACECALIBCONTROL_H_
#define TOOLSURFACECALIBCONTROL_H_

#include <dumbo_tool_surface_calib/ToolSurfaceCalibParams.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>

using namespace Eigen;
using namespace geometry_msgs;
using namespace tf;
using namespace std_msgs;

class ToolSurfaceCalibControl
{
public:
	ToolSurfaceCalibControl(ToolSurfaceCalibParams *params);
	virtual ~ToolSurfaceCalibControl();

	// executes the controller
	// calculates the velocity screw of FT sensor frame expressed in the base frame)
	// and updates the estimates of contact point and surface normal
	// FT_compensated given in FT sensor frame
	// vel of FT sensor frame expressed in base frame
	virtual TwistStamped Run(const WrenchStamped &FT_compensated, const Vector3d& vel);

	// resets the controller (integrators and trajectory generators)
	virtual void Reset();


	virtual PointStamped GetContactPointEstimate();

	virtual Vector3Stamped GetSurfaceNormalEstimate();

	virtual double GetNormalForceError();


	virtual Float64MultiArray GetLr();

	virtual Float64MultiArray GetQr();



protected:

	ToolSurfaceCalibParams *m_params;

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
	void UpdateLr(const Vector3d &force);

	void UpdateQr(const Vector3d &force, const Vector3d &torque);

	virtual void UpdateContactPointEstimate();


	// velocity of ft sensor frame expressed in base frame
	virtual void UpdateLn(const Vector3d &v_ft);

	// surface normal expressed in the base frame
	virtual void UpdateSurfaceNormalEstimate();


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
	virtual void UpdateCircleTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d);



	// desired FT sensor pos and vel
	// makes an arc trajectory
	// p_ft_d/v_ft_d: desired pos/vel of FT frame expressed in the base frame
	//todo implement this function
	virtual void UpdateLineTrajectory(Vector3d &p_ft_d, Vector3d &v_ft_d);

	// calculate reference velocity of ft sensor frame with respect to the base frame
	virtual Vector3d Vft(const Vector3d &v_d, double v_f);

private:





};

#endif /* TOOLSURFACECALIBCONTROL_H_ */
