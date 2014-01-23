/*
 *  contact_point_estimation_node.cpp
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

#include <ros/ros.h>
#include <contact_point_estimation/ContactPointEstimation.h>
#include <contact_point_estimation/ContactPointEstimationParams.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <kdl/chainfksolvervel_recursive.hpp>



class ContactPointEstimationNode
{
public:
    ros::NodeHandle n_;

    /// declaration of topics to publish
    ros::Publisher topicPub_ContactPointEstimate_;
    ros::Publisher topicPub_SurfaceNormalEstimate_;

    /// declaration of topics to subscribe, callback is called for new messages arriving
    ros::Subscriber topicSub_FT_compensated_;
    ros::Subscriber topicSub_Twist_FT_Sensor_;

    /// declaration of service servers
    ros::ServiceServer srvServer_Init_;

	ros::Time last_publish_time;
	ContactPointEstimationParams *cpe_params;
	ContactPointEstimation *cpe;


	ContactPointEstimationNode()
	{
		n_ = ros::NodeHandle("~");
        m_received_ft = false;
		m_start_controller = false;

		topicPub_ContactPointEstimate_ = n_.advertise<geometry_msgs::PointStamped>("contact_point_estimate", 1);
		topicPub_SurfaceNormalEstimate_ = n_.advertise<geometry_msgs::Vector3Stamped>("surface_normal_estimate", 1);

        topicSub_FT_compensated_ = n_.subscribe("ft_compensated", 1,
				&ContactPointEstimationNode::topicCallback_FT_compensated, this);

		srvServer_Init_ = n_.advertiseService("start", &ContactPointEstimationNode::srvCallback_Init,
				this);
	}

	~ContactPointEstimationNode()
	{
		delete cpe_params;
		delete cpe;
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


        /// Get initial estimate of the contact point
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

        /// Get initial estimate of the surface normal
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



		bool ret = true;
        cpe_params = new ContactPointEstimationParams();

        cpe_params->setGammaR(gamma_r);
        cpe_params->setKappaR(kappa_r);
        cpe_params->setBetaR(beta_r);
        cpe_params->setInitialR(initial_r);

        cpe_params->setGammaN(gamma_n);
        cpe_params->setBetaN(beta_n);
        cpe_params->setInitialN(initial_n);

        cpe_params->setControlFrequency(control_frequency);

		return ret;

	}

	void topicCallback_FT_compensated(const geometry_msgs::WrenchStampedPtr &msg)
	{
		m_FT_compensated = *msg;
		m_received_ft = true;
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
	ros::init(argc, argv, "contact_point_estimation_node");

	ContactPointEstimationNode cpe_node;

	if(!cpe_node.getROSParameters())
	{
		cpe_node.n_.shutdown();
		return 0;
	}

	if(!cpe_node.getControllerParameters())
	{
		cpe_node.n_.shutdown();
		return 0;
	}

	//initialize arm position
//	if(!cpe_node.InitializeArmPos())
//	{
//		ROS_ERROR("Error initializing arm position");
//		cpe_node.n_.shutdown();
//		return 0;
//	}


	// controller main loop
	double control_frequency;
	if (cpe_node.n_.hasParam("control_frequency"))
	{
		cpe_node.n_.getParam("control_frequency", control_frequency);
	}

	else
	{
		ROS_ERROR("Parameter control_frequency not available, shutting down node...");
		cpe_node.n_.shutdown();
		return 0;
	}

	if(control_frequency<=0.0)
	{
		ROS_ERROR("Error in control_frequency");
		cpe_node.n_.shutdown();
		return 0;
	}

	ros::Duration timeout;
	double sec;
	if (cpe_node.n_.hasParam("timeout"))
	{
		cpe_node.n_.getParam("timeout", sec);
		timeout.fromSec(sec);
	}

	else
	{
		ROS_ERROR("Parameter timeout not available, shutting down node...");
		cpe_node.n_.shutdown();
		return 0;
	}

	if(sec<=0.0)
	{
		ROS_ERROR("Error in timeout");
		cpe_node.n_.shutdown();
		return 0;
	}


	ros::Rate loop_rate(control_frequency);

	cpe_node.cpe = new ContactPointEstimation(cpe_node.cpe_params);


	// wait for init srv to be called to start the controller
	while(!cpe_node.startController() && cpe_node.n_.ok())
	{
		ros::Duration(1.0).sleep();
		ros::spinOnce();
	}

	ros::Time begin = ros::Time::now();

	while(cpe_node.n_.ok())
	{
		if((ros::Time::now()-begin)>=timeout)
		{
			ROS_INFO("Timeout... shutting down controller node");
			cpe_node.n_.shutdown();
			return 0;
		}

		// if ((ros::Time::now() - cpe_node.last_publish_time) >= control_period)
		// {
		geometry_msgs::TwistStamped twist_ft;
		std::vector<double> joint_vel;
		if(cpe_node.ExecuteController(twist_ft,joint_vel))
		{
			cpe_node.publishVelCommand(joint_vel);

			geometry_msgs::PointStamped contact_point_estimate = cpe_node.controller->GetContactPointEstimate();
			cpe_node.topicPub_ContactPointEstimate_.publish(contact_point_estimate);

			geometry_msgs::Vector3Stamped surface_normal_estimate = cpe_node.controller->GetSurfaceNormalEstimate();
			cpe_node.topicPub_SurfaceNormalEstimate_.publish(surface_normal_estimate);

			std_msgs::Float64 normal_force_error;
			normal_force_error.data = cpe_node.controller->GetNormalForceError();
			cpe_node.topicPub_NormalForceError_.publish(normal_force_error);

			// std_msgs::Float64MultiArray Lr = cpe_node.controller->GetLr();
			// cpe_node.topicPub_Lr_.publish(Lr);

			// std_msgs::Float64MultiArray Qr = cpe_node.controller->GetQr();
			// cpe_node.topicPub_Qr_.publish(Qr);

		}
		// }
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


