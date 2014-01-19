/*
 * tool_surface_calib_control_node.cpp
 *
 *  Created on: Aug 21, 2013
 *      Author: fevb
 */

#include <ros/ros.h>
#include <dumbo_tool_surface_calib/ToolSurfaceCalibControl.h>
#include <dumbo_tool_surface_calib/ToolSurfaceCalibParams.h>
#include <brics_actuator/JointVelocities.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <cob_srvs/Trigger.h>
#include <dumbo_kdl/DumboKDL.h>

#include <kdl/chainfksolvervel_recursive.hpp>



class ToolSurfaceCalibControlNode
{
public:
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_CommandVel_;
	ros::Publisher topicPub_ContactPointEstimate_;
	ros::Publisher topicPub_SurfaceNormalEstimate_;
	ros::Publisher topicPub_NormalForceError_;

	ros::Publisher topicPub_Lr_;
	ros::Publisher topicPub_Qr_;

	ros::Publisher topicPub_FT_Meas_;


	/// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointState_;
	ros::Subscriber topicSub_FT_compensated_;
	ros::Subscriber topicSub_EstimatedJointState_; // gets data from Kalman Filter of joint vel + acc

	 /// declaration of service servers
	ros::ServiceServer srvServer_Init_;

	ros::Time last_publish_time;
	ToolSurfaceCalibParams *control_params;
	ToolSurfaceCalibControl *controller;


	ToolSurfaceCalibControlNode()
	{
		n_ = ros::NodeHandle("~");
		m_InitializedKDL = false;
		m_received_ft = false;
		m_received_js = false;
		m_received_estimated_js = false;
		m_start_controller = false;

		topicPub_ContactPointEstimate_ = n_.advertise<geometry_msgs::PointStamped>("contact_point_estimate", 1);
		topicPub_SurfaceNormalEstimate_ = n_.advertise<geometry_msgs::Vector3Stamped>("surface_normal_estimate", 1);
		topicPub_NormalForceError_ = n_.advertise<std_msgs::Float64>("normal_force_error", 1);
		topicPub_FT_Meas_ = n_.advertise<geometry_msgs::WrenchStamped>("ft_meas", 1);

		topicPub_Lr_ = n_.advertise<std_msgs::Float64MultiArray>("Lr", 1);
		topicPub_Qr_ = n_.advertise<std_msgs::Float64MultiArray>("Qr", 1);

		topicSub_JointState_ = n_.subscribe("/left_arm_controller/state", 1,
				&ToolSurfaceCalibControlNode::topicCallback_JointStates, this);
		topicSub_FT_compensated_ = n_.subscribe("/left_arm_FT_sensor/FT_compensated", 1,
				&ToolSurfaceCalibControlNode::topicCallback_FT_compensated, this);

		topicSub_EstimatedJointState_ = n_.subscribe("/left_arm_joint_state_estimator/estimated_joint_state", 1,
				&ToolSurfaceCalibControlNode::topicCallback_EstimatedJointStates, this);

		srvServer_Init_ = n_.advertiseService("start", &ToolSurfaceCalibControlNode::srvCallback_Init,
				this);
	}

	~ToolSurfaceCalibControlNode()
	{
		delete control_params;
		delete controller;
		delete m_DumboKDL_;
		delete m_fk_solver_vel;
	}

	bool isKDLInitialized()
	{
		return m_InitializedKDL;
	}

	bool getROSParameters()
	{
		/// Get joint names
		XmlRpc::XmlRpcValue JointNamesXmlRpc;
		std::vector<std::string> JointNames;
		if (n_.hasParam("joint_names"))
		{
			n_.getParam("joint_names", JointNamesXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		/// Resize and assign of values to the JointNames
		JointNames.resize(JointNamesXmlRpc.size());
		for (int i = 0; i < JointNamesXmlRpc.size(); i++)
		{
			JointNames[i] = (std::string)JointNamesXmlRpc[i];
		}

		m_JointNames = JointNames;
		m_DOF = JointNames.size();
		m_JointPos.resize(m_DOF);


		// Get arm select param
		std::string ArmSelect;
		if(n_.hasParam("arm_select"))
		{
			n_.getParam("arm_select", ArmSelect);
		}

		else
		{
			ROS_ERROR("Parameter arm_select not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(ArmSelect!="left" && ArmSelect!="right")
		{
			ROS_ERROR("Incorrect arm_select param");
			n_.shutdown();
			return false;
		}
		m_ArmSelect = ArmSelect;


		// topic to publish velocity commands
		std::string vel_commands_topic;
		if (n_.hasParam("vel_commands_topic"))
		{
			n_.getParam("vel_commands_topic", vel_commands_topic);
		}

		else
		{
			ROS_ERROR("Parameter vel_commands_topic not set, shutting down node...");
			n_.shutdown();
			return false;
		}
		m_vel_commands_topic = vel_commands_topic;

		// max translational velocity
		if(n_.hasParam("v_max"))
		{
			n_.getParam("v_max", m_v_max);
		}

		else
		{
			ROS_ERROR("Parameter v_max not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(m_v_max<=0.0)
		{
			ROS_ERROR("v_max <= 0");
			n_.shutdown();
			return false;
		}

		// max rotational velocity
		if(n_.hasParam("w_max"))
		{
			n_.getParam("w_max", m_w_max);
			m_w_max = m_w_max*M_PI/180.0;
		}

		else
		{
			ROS_ERROR("Parameter w_max not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(m_w_max<=0.0)
		{
			ROS_ERROR("w_max <= 0");
			n_.shutdown();
			return false;
		}



		topicPub_CommandVel_ = n_.advertise<brics_actuator::JointVelocities>(m_vel_commands_topic, 1);
		return true;
	}

	bool getControllerParameters()
	{

		double gamma_r;
		if (n_.hasParam("gamma_r"))
		{
			n_.getParam("gamma_r", gamma_r);
		}

		else
		{
			ROS_ERROR("Parameter gamma_r not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double kappa_r;
		if (n_.hasParam("kappa_r"))
		{
			n_.getParam("kappa_r", kappa_r);
		}

		else
		{
			ROS_ERROR("Parameter kappa_r not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		double beta_r;
		if (n_.hasParam("beta_r"))
		{
			n_.getParam("beta_r", beta_r);
		}

		else
		{
			ROS_ERROR("Parameter beta_r not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		/// Get initial contact point
		XmlRpc::XmlRpcValue initial_r_XmlRpc;
		Vector3d initial_r;
		if (n_.hasParam("initial_r"))
		{
			n_.getParam("initial_r", initial_r_XmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter initial_r not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(initial_r_XmlRpc.size()!=3)
		{
			ROS_ERROR("Wrong initial_r size.");
			n_.shutdown();
			return false;
		}

		/// Resize and assign of values to the initial_r
		for (int i = 0; i < initial_r_XmlRpc.size(); i++)
		{
			initial_r(i) = (double)initial_r_XmlRpc[i];
		}


		double gamma_n;
		if (n_.hasParam("gamma_n"))
		{
			n_.getParam("gamma_n", gamma_n);
		}

		else
		{
			ROS_ERROR("Parameter gamma_n not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double beta_n;
		if (n_.hasParam("beta_n"))
		{
			n_.getParam("beta_n", beta_n);
		}

		else
		{
			ROS_ERROR("Parameter beta_n not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		/// Get initial surface normal
		XmlRpc::XmlRpcValue initial_n_XmlRpc;
		Vector3d initial_n;
		if (n_.hasParam("initial_n"))
		{
			n_.getParam("initial_n", initial_n_XmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter initial_n not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(initial_n_XmlRpc.size()!=3)
		{
			ROS_ERROR("Wrong initial_n size.");
			n_.shutdown();
			return false;
		}

		/// Resize and assign of values to the initial_n
		for (int i = 0; i < initial_n_XmlRpc.size(); i++)
		{
			initial_n(i) = (double)initial_n_XmlRpc[i];
		}

		double normal_force_ref;
		if (n_.hasParam("normal_force_ref"))
		{
			n_.getParam("normal_force_ref", normal_force_ref);
		}

		else
		{
			ROS_ERROR("Parameter normal_force_ref not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double alpha_f;
		if (n_.hasParam("alpha_f"))
		{
			n_.getParam("alpha_f", alpha_f);
		}

		else
		{
			ROS_ERROR("Parameter alpha_f not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double beta_f;
		if (n_.hasParam("beta_f"))
		{
			n_.getParam("beta_f", beta_f);
		}

		else
		{
			ROS_ERROR("Parameter beta_f not set, shutting down node...");
			n_.shutdown();
			return false;
		}



		double control_frequency;
		if (n_.hasParam("control_frequency"))
		{
			n_.getParam("control_frequency", control_frequency);
		}

		else
		{
			ROS_ERROR("Parameter control_frequency not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double alpha_v_d;
		if (n_.hasParam("alpha_v_d"))
		{
			n_.getParam("alpha_v_d", alpha_v_d);
		}

		else
		{
			ROS_ERROR("Parameter alpha_v_d not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double circle_traj_radius;
		if (n_.hasParam("circle_traj_radius"))
		{
			n_.getParam("circle_traj_radius", circle_traj_radius);
		}

		else
		{
			ROS_ERROR("Parameter circle_traj_radius not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double circle_traj_period;
		if (n_.hasParam("circle_traj_period"))
		{
			n_.getParam("circle_traj_period", circle_traj_period);
		}

		else
		{
			ROS_ERROR("Parameter circle_traj_period not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double line_traj_speed;
		if (n_.hasParam("line_traj_speed"))
		{
			n_.getParam("line_traj_speed", line_traj_speed);
		}

		else
		{
			ROS_ERROR("Parameter line_traj_speed not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double line_traj_t1;
		if (n_.hasParam("line_traj_t1"))
		{
			n_.getParam("line_traj_t1", line_traj_t1);
		}

		else
		{
			ROS_ERROR("Parameter line_traj_t1 not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		double line_traj_t2;
		if (n_.hasParam("line_traj_t2"))
		{
			n_.getParam("line_traj_t2", line_traj_t2);
		}

		else
		{
			ROS_ERROR("Parameter line_traj_t2 not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		std::string robot_base_frame_id;
		if (n_.hasParam("robot_base_frame_id"))
		{
			n_.getParam("robot_base_frame_id", robot_base_frame_id);
		}

		else
		{
			ROS_ERROR("Parameter robot_base_frame_id not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(robot_base_frame_id.size()==0)
		{
			ROS_ERROR("Empty robot_base_frame_id param");
			n_.shutdown();
			return false;
		}


		std::string robot_ft_frame_id;
		if (n_.hasParam("robot_ft_frame_id"))
		{
			n_.getParam("robot_ft_frame_id", robot_ft_frame_id);
		}

		else
		{
			ROS_ERROR("Parameter robot_ft_frame_id not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(robot_ft_frame_id.size()==0)
		{
			ROS_ERROR("Empty robot_ft_frame_id param");
			n_.shutdown();
			return false;
		}


		bool ret = true;
		control_params = new ToolSurfaceCalibParams();

		control_params->SetGammaR(gamma_r);
		control_params->setKappaR(kappa_r);
		control_params->SetBetaR(beta_r);
		control_params->SetInitialR(initial_r);

		control_params->SetGammaN(gamma_n);
		control_params->SetBetaN(beta_n);
		control_params->SetInitialN(initial_n);

		control_params->SetNormalForceRef(normal_force_ref);
		control_params->SetAlphaF(alpha_f);
		control_params->SetBetaF(beta_f);

		control_params->SetControlFrequency(control_frequency);
		control_params->SetAlphaVd(alpha_v_d);
		control_params->SetCircleTrajRadius(circle_traj_radius);
		control_params->SetCircleTrajPeriod(circle_traj_period);

		control_params->SetLineTrajSpeed(line_traj_speed);
		control_params->SetLineTrajT1(line_traj_t1);
		control_params->SetLineTrajT2(line_traj_t2);

		control_params->SetRobotBaseFrameID(robot_base_frame_id);
		control_params->SetRobotFtFrameID(robot_ft_frame_id);

		return ret;

	}

	void topicCallback_FT_compensated(const geometry_msgs::WrenchStampedPtr &msg)
	{
		m_FT_compensated = *msg;
		m_received_ft = true;
	}

	void topicCallback_JointStates(const pr2_controllers_msgs::JointTrajectoryControllerStatePtr &msg)
	{
		ROS_DEBUG("Received joint states");
		std::vector<double> JointPos(m_DOF,0.0);

		// search for joints in joint state msg
		if(msg->actual.positions.size()==m_DOF)
		{
			for(unsigned int i=0; i<m_DOF; i++)
			{
				if(msg->joint_names[i]!=m_JointNames[i])
				{
					ROS_ERROR("Error in received joint name");
					return;
				}

				else
				{
					 JointPos[i] = msg->actual.positions[i];
				}
			}

		}

		m_JointPos = JointPos;
		m_JointPos_stamp = msg->header.stamp;
		m_received_js = true;

	}

	void topicCallback_EstimatedJointStates(const pr2_controllers_msgs::JointTrajectoryControllerStatePtr &msg)
		{
			ROS_DEBUG("Received estimated joint states");
			std::vector<double> JointVel(m_DOF,0.0);

			// search for joints in joint state msg
			if(msg->actual.velocities.size()==m_DOF)
			{
				for(unsigned int i=0; i<m_DOF; i++)
				{
					if(msg->joint_names[i]!=m_JointNames[i])
					{
						ROS_ERROR("Error in received joint name");
						return;
					}

					else
					{
						 JointVel[i] = msg->actual.velocities[i];
					}
				}

			}

			m_JointVel = JointVel;
			m_received_estimated_js = true;
		}

	// calculates vel screw in arm_base_link frame
	// and the joint velocities
	bool ExecuteController(geometry_msgs::TwistStamped &twist_ft, std::vector<double> &joint_vel)
	{
		ROS_DEBUG("Calculating control signal");
		if(!m_InitializedKDL)
		{
			m_DumboKDL_ = new DumboKDL();
			if(m_DumboKDL_->Init(&n_, "left_arm_FT_sensor"))
			{
				m_fk_solver_vel = new KDL::ChainFkSolverVel_recursive(m_DumboKDL_->GetChain());
				m_InitializedKDL = true;
			}

			else
			{
				ROS_ERROR("Error initializing DumboKDL kinematics solver");
				return false;
			}
		}

		// vel screw of ft sensor frame expressed in the base frame
		std::vector<double> vel_screw_ft(6, 0.0);
		joint_vel = std::vector<double>(m_DOF, 0.0);

		// safety checks
		if(!m_received_ft || !m_received_js || !m_received_estimated_js)
		{
			ROS_ERROR("Haven't received FT measurements or joint states");
			return false;
		}

		ros::Time now = ros::Time::now();
		if(((now-m_JointPos_stamp).toSec()>0.2) || ((now - m_FT_compensated.header.stamp).toSec()>0.2))
		{
			ROS_ERROR("Joint pos or FT measurement too old");
			return false;
		}


		// calculate the velocity of the FT sensor frame from joint pos and estimated joint vel
		KDL::JntArrayVel q_in(7);
		for(unsigned int i=0; i<7; i++)
		{
			q_in.q(i) = m_JointPos[i];
			q_in.qdot(i) = m_JointVel[i];
		}

		KDL::FrameVel f_vel;
		m_fk_solver_vel->JntToCart(q_in, f_vel);

		Eigen::Vector3d vel(f_vel.p.v(0), f_vel.p.v(1), f_vel.p.v(2));

		// CALCULATE VEL SCREW of ft sensor in robot base frame
		twist_ft = controller->Run(m_FT_compensated, vel);


		//saturate velocity screws
		vel_screw_ft[0] = twist_ft.twist.linear.x;
		vel_screw_ft[1] = twist_ft.twist.linear.y;
		vel_screw_ft[2] = twist_ft.twist.linear.z;

		vel_screw_ft[3] = twist_ft.twist.angular.x;
		vel_screw_ft[4] = twist_ft.twist.angular.y;
		vel_screw_ft[5] = twist_ft.twist.angular.z;

		double v_scale = sqrt(pow(vel_screw_ft[0], 2.0) + pow(vel_screw_ft[1], 2.0) + pow(vel_screw_ft[2], 2.0))/m_v_max;

		if(v_scale>1.0)
		{
			for(int i=0; i<3; i++)
			{
				vel_screw_ft[i] /= v_scale;
			}
		}

		double w_scale = sqrt(pow(vel_screw_ft[3], 2.0) + pow(vel_screw_ft[4], 2.0) + pow(vel_screw_ft[5], 2.0))/m_w_max;

		if(w_scale>1.0)
		{
			for(int i=0; i<3; i++)
			{
				vel_screw_ft[i+3] /= w_scale;
			}
		}

		twist_ft.twist.linear.x = vel_screw_ft[0];
		twist_ft.twist.linear.y = vel_screw_ft[1];
		twist_ft.twist.linear.z = vel_screw_ft[2];

		twist_ft.twist.angular.x = vel_screw_ft[3];
		twist_ft.twist.angular.y = vel_screw_ft[4];
		twist_ft.twist.angular.z = vel_screw_ft[5];


		std::vector<double> joint_vel_;
		bool ret;
//		ret =  m_DumboKDL_->getInvVel(m_JointPos, vel_screw_ft, joint_vel_, 6.0); // for sliding door : 2.0, for others : 6.0
		ret = m_DumboKDL_->getInvVelConstantDamping(m_JointPos, vel_screw_ft, joint_vel_, 0.3);
		joint_vel = joint_vel_;
		m_FT_compensated.header.stamp = ros::Time::now();
		topicPub_FT_Meas_.publish(m_FT_compensated);
		return ret;
	}


	void publishVelCommand(std::vector<double> joint_vel)
	{
		ROS_DEBUG("Publish vel command");


		if(joint_vel.size()!=m_DOF)
		{
			ROS_ERROR("Incorrect joint_vel size");
			return;
		}


		brics_actuator::JointVelocities joint_vel_msg;
		joint_vel_msg.velocities.resize(m_DOF);

		for(unsigned int i=0; i<m_DOF; i++)
		{
			joint_vel_msg.velocities[i].unit = "rad";
			joint_vel_msg.velocities[i].joint_uri = m_JointNames[i].c_str();
			joint_vel_msg.velocities[i].value = joint_vel.at(i);
		}
		topicPub_CommandVel_.publish(joint_vel_msg);
		last_publish_time = ros::Time::now();
	}

	std::string GetArmSelect()
	{
		return m_ArmSelect;
	}


	bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
	{
		m_start_controller = true;
		return true;
	}

	bool startController()
	{
		return m_start_controller;
	}


private:
	bool m_InitializedKDL;
	unsigned int m_DOF;
	std::string m_ArmSelect;
	std::string m_vel_commands_topic;

	std::vector<std::string> m_JointNames;
	geometry_msgs::WrenchStamped m_FT_compensated;
	std::vector<double> m_JointPos;
	std::vector<double> m_JointVel;
	ros::Time m_JointPos_stamp;
	DumboKDL *m_DumboKDL_;

	KDL::ChainFkSolverVel_recursive *m_fk_solver_vel;


	double m_v_max;
	double m_w_max;

	bool m_received_js;
	bool m_received_estimated_js;
	bool m_received_ft;

	bool m_start_controller;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tool_surface_calib_control_node");

	ToolSurfaceCalibControlNode ControllerNode;

	if(!ControllerNode.getROSParameters())
	{
		ControllerNode.n_.shutdown();
		return 0;
	}

	if(!ControllerNode.getControllerParameters())
	{
		ControllerNode.n_.shutdown();
		return 0;
	}

	//initialize arm position
//	if(!ControllerNode.InitializeArmPos())
//	{
//		ROS_ERROR("Error initializing arm position");
//		ControllerNode.n_.shutdown();
//		return 0;
//	}


	// controller main loop
	double control_frequency;
	if (ControllerNode.n_.hasParam("control_frequency"))
	{
		ControllerNode.n_.getParam("control_frequency", control_frequency);
	}

	else
	{
		ROS_ERROR("Parameter control_frequency not available, shutting down node...");
		ControllerNode.n_.shutdown();
		return 0;
	}

	if(control_frequency<=0.0)
	{
		ROS_ERROR("Error in control_frequency");
		ControllerNode.n_.shutdown();
		return 0;
	}

	ros::Duration timeout;
	double sec;
	if (ControllerNode.n_.hasParam("timeout"))
	{
		ControllerNode.n_.getParam("timeout", sec);
		timeout.fromSec(sec);
	}

	else
	{
		ROS_ERROR("Parameter timeout not available, shutting down node...");
		ControllerNode.n_.shutdown();
		return 0;
	}

	if(sec<=0.0)
	{
		ROS_ERROR("Error in timeout");
		ControllerNode.n_.shutdown();
		return 0;
	}


	ros::Rate loop_rate(control_frequency);

	ControllerNode.controller = new ToolSurfaceCalibControl(ControllerNode.control_params);


	// wait for init srv to be called to start the controller
	while(!ControllerNode.startController() && ControllerNode.n_.ok())
	{
		ros::Duration(1.0).sleep();
		ros::spinOnce();
	}

	ros::Time begin = ros::Time::now();

	while(ControllerNode.n_.ok())
	{
		if((ros::Time::now()-begin)>=timeout)
		{
			ROS_INFO("Timeout... shutting down controller node");
			ControllerNode.n_.shutdown();
			return 0;
		}

		// if ((ros::Time::now() - ControllerNode.last_publish_time) >= control_period)
		// {
		geometry_msgs::TwistStamped twist_ft;
		std::vector<double> joint_vel;
		if(ControllerNode.ExecuteController(twist_ft,joint_vel))
		{
			ControllerNode.publishVelCommand(joint_vel);

			geometry_msgs::PointStamped contact_point_estimate = ControllerNode.controller->GetContactPointEstimate();
			ControllerNode.topicPub_ContactPointEstimate_.publish(contact_point_estimate);

			geometry_msgs::Vector3Stamped surface_normal_estimate = ControllerNode.controller->GetSurfaceNormalEstimate();
			ControllerNode.topicPub_SurfaceNormalEstimate_.publish(surface_normal_estimate);

			std_msgs::Float64 normal_force_error;
			normal_force_error.data = ControllerNode.controller->GetNormalForceError();
			ControllerNode.topicPub_NormalForceError_.publish(normal_force_error);

			std_msgs::Float64MultiArray Lr = ControllerNode.controller->GetLr();
			ControllerNode.topicPub_Lr_.publish(Lr);

			std_msgs::Float64MultiArray Qr = ControllerNode.controller->GetQr();
			ControllerNode.topicPub_Qr_.publish(Qr);

		}
		// }
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


