// universalRobotsKinematics.h

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "mathLib.h"


namespace universalRobots
{
	/// URtype
	/// 
	/// Different types of URs.
	enum URtype : unsigned int
	{
		UR3, UR5, UR10 // 0, 1, 2
	};


	/////////////////////////////////////////////////////////////////////////////
	/// UR
	/// 
	/// Implements 'UR robot type' object (with particular characteristics).
	/// 
	/// Example usage:
	///		universalRobots::UR robot_one(); // Robot does not have an end-effector.
	///		universalRobots::UR robot_one(true, 0.15f); // End-effector translated 0.15 meters from the robot's tip.
	/// 
	///		robot.setTipPose(robot.forwardKinematics(targetJointValues)); // Generate a forward kinematics solution and update the robot's tip pose.
	/// 
	class UR
	{
	public:
		/// m_numDoF
		/// 
		/// Number of Degrees of Freedom is always 6 for all URs.
		/// 
		static constexpr unsigned int m_numDoF = 6;

		/// m_numTransZ
		/// 
		/// Number of translations in the z-axis is always 7 (this includes a translation for the end-effector).
		/// 
		static constexpr unsigned int m_numTransZ = 7; 

		/// m_numTransX
		/// 
		/// Number of translations in the z-axis is always 4.
		/// 
		static constexpr unsigned int m_numTransX = 4;

		/// m_numReferenceFrames
		/// 
		/// Number of frames is pre-defined (according to the MDH convention it is 9).
		/// 
		static constexpr unsigned int m_numReferenceFrames = 9;

		/// m_numIkSol
		/// 
		/// Number of inverse kinematics solutions for a UR is 8.
		/// 
		static constexpr unsigned int m_numIkSol = 8;

		/// m_endEffector
		/// 
		/// This boolean indicates whether a tool is/isnt attached to the robot.
		/// Must be specified in the constructor, if not default is false.
		/// 
		bool m_endEffector = false; 

	private:

		/// m_type
		/// 
		/// Robot type/name UR 3, 5, or 10.
		/// 
		URtype m_type = UR10;

		/// m_d
		/// 
		/// d - translation (meters) in the z-axis (the last translation d[6] is for an end-effector).
		/// d_i is a nomeclature convention.
		/// 
		float m_d[m_numTransZ] = { 0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f };

		/// m_a
		/// 
		/// a - translation (meters) in the x-axis.
		/// a_i is a nomeclature convention.
		/// 
		float m_a[m_numTransX] = { 0.6121f, 0.5722f, 0.0573f, 0.0584f };

		/// m_theta
		/// 
		/// Joint values of the robot in radians.
		/// 
		float m_theta[m_numDoF] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

		/// m_tipPose
		/// 
		/// An object which holds two 1x3 matrices.
		/// {x, y, z} {alpha, beta, gamma}
		/// Position and Euler angles of the robot's tip.
		/// 
		mathLib::tipPose m_tipPose;

	public:
		/// m_MDHmatrix
		/// 
		/// Modified Denavit-Hartenberg parameters matrix (9x4)
		/// { alpha_i-1, a_i-1, d_i, theta_i } for each frame.
		///		|_ alpha - rotation around the x axis of frame i (radians)
		///		|_ a - translation along the x axis of frame i (meters)
		///		|_ d - translation along the z axis of frame i (meters)
		///		|_ theta - rotation around the z axis of frame i (radians)
		/// 
		Eigen::Matrix<float, m_numReferenceFrames, 4> m_MDHmatrix { {0,					0,			m_d[0],			m_theta[0]},					// 0T1
																	{mathLib::rad(-90),	0,			m_d[1],			m_theta[1] + mathLib::rad(-90)},// 1T2
																	{0,					m_a[0],		m_d[2],			m_theta[2]},					// 2T3
																	{0,					m_a[1],		m_d[3],			m_theta[3]},					// 3T4
																	{0,					m_a[2],		m_d[4],			mathLib::rad(90)},				// 4T4'
																	{mathLib::rad(90),	0,			0,				m_theta[4]},					// 4'T5
																	{mathLib::rad(-90),	0,			0,				mathLib::rad(-90)},				// 5T5'
																	{0,					m_a[3],		m_d[5],			m_theta[5]},					// 5'T6
																	{0,					0,			m_d[6],			0} };							// 6T7

	public:
		UR(const bool& endEffector = false, const float& endEffectorDimension = 0.0f);

		void setTipPose(const mathLib::tipPose &tipPose);
		mathLib::tipPose forwardKinematics(const float(&targetJointVal)[]);
		void inverseKinematics(const float(&targetTipPose)[], float(*outIkSols)[m_numIkSol][m_numDoF]);
		
		friend std::ostream& operator <<(std::ostream& stream, const universalRobots::UR& robot);
	private:
		
		void setMDHmatrix();
		void setRobotType(const URtype& type);
		void setTransZ(const float (&d)[]);
		void setTransX(const float (&a)[]);
		void setTheta(const float (&jointVal)[]);

		const URtype getRobotType() const;
		const float* getTransZ() const;
		const float* getTransX() const;
		const float* getTheta() const;
		const mathLib::tipPose getTipPose() const;
		//const float** getMDHmatrix() const;
	};


	std::ostream& operator <<(std::ostream& stream, universalRobots::URtype& type);

} // namespace universalRobots 