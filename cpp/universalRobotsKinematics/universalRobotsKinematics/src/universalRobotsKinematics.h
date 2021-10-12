// universalRobotsKinematics.h

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "mathLib.h"


namespace universalRobots
{
	
	/// <summary> URtype
	/// Different types of URs.
	/// </summary>
	enum URtype : unsigned int
	{
		UR3, UR5, UR10 // 0, 1, 2
	};


	struct pose
	{
		float m_pos[3] = {}; // x y z (meters)
		float m_eulerAngles[3] = {}; // alpha beta gamma (radians)

		pose()
			: m_pos{ 0.0f, 0.0f, 0.0f }, m_eulerAngles{ 0.0f, 0.0f, 0.0f } {}

		pose(const float(&pos)[], const float(&eulerAngles)[])
			: m_pos{ pos[0],  pos[1],  pos[2] }, m_eulerAngles{ eulerAngles[0], eulerAngles[1], eulerAngles[2] } {}

		pose(const float(&pos)[], const Eigen::Matrix3f& rotationMatrix)
			: m_pos{ pos[0],  pos[1],  pos[2] }, m_eulerAngles{ rotationMatrix.eulerAngles(2, 1, 0).x(), rotationMatrix.eulerAngles(2, 1, 0).y(), rotationMatrix.eulerAngles(2, 1, 0).z() } {}
	};

	struct joint
	{
		pose jointPose;
		float jointValue = 0.0f;
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
		
		/// <summary>
		/// Number of Degrees of Freedom is always 6 for all URs.
		/// </summary>
		static constexpr unsigned int m_numDoF = 6;
		
		/// <summary>
		/// Number of translations in the z-axis is always 7 (this includes a translation for the end-effector).
		/// </summary>
		static constexpr unsigned int m_numTransZ = 7; 
		
		/// <summary>
		/// Number of translations in the z-axis is always 4.
		/// </summary>
		static constexpr unsigned int m_numTransX = 4;
		
		/// <summary>
		/// Number of frames is pre-defined (according to the MDH convention it is 9).
		/// </summary>
		static constexpr unsigned int m_numReferenceFrames = 9;
		
		/// <summary>
		/// Number of inverse kinematics solutions for a UR is 8.
		/// </summary>
		static constexpr unsigned int m_numIkSol = 8;

		/// <summary>
		/// This boolean indicates whether a tool is/isnt attached to the robot.
		/// Must be specified in the constructor, if not default is false.
		/// </summary>
		bool m_endEffector = false; 

	private:

		/// <summary>
		/// Robot type/name UR 3, 5, or 10.
		/// </summary>
		/// <returns>0, 1, 2</returns>
		URtype m_type = UR10;
		
		/// <summary>
		/// d - translation (meters) in the z-axis (the last translation d[6] is for an end-effector).
		/// d_i is a nomeclature convention. 
		/// </summary>
		float m_d[m_numTransZ] = { 0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f };
		
		/// <summary>
		/// a - translation (meters) in the x-axis.
		/// a_i is a nomeclature convention.
		/// </summary>
		float m_a[m_numTransX] = { 0.6121f, 0.5722f, 0.0573f, 0.0584f };
		
		/// <summary>
		/// Position, Euler Angles, and Value of the robot's joints. {x, y, z} {alpha, beta, gamma} {jointValue}
		/// </summary>
		joint m_jointState[m_numDoF] = {};

		// Create an array of (individual) transformation matrices iTi+1 / i-1Ti
		
		/// <summary>
		/// Array of (individual) transformation matrices iTi+1 / i-1Ti
		/// </summary>
		Eigen::Matrix4f m_individualTransformationMatrices[m_numReferenceFrames] = {};

		/// <summary>
		/// Array of (general) transformation matrices 0Ti
		/// </summary>
		Eigen::Matrix4f m_generalTransformationMatrices[m_numReferenceFrames] = {};

	public:
		
		/// <summary>
		/// Modified Denavit-Hartenberg parameters matrix (9x4)
		/// { alpha_i-1, a_i-1, d_i, theta_i } for each frame.
		///		|_ alpha - rotation around the x axis of frame i (radians)
		///		|_ a - translation along the x axis of frame i (meters)
		///		|_ d - translation along the z axis of frame i (meters)
		///		|_ theta - rotation around the z axis of frame i (radians)
		/// </summary>
		Eigen::Matrix<float, m_numReferenceFrames, 4> m_MDHmatrix { {0,					0,			m_d[0],			m_jointState[0].jointValue},					// 0T1
																	{mathLib::rad(-90),	0,			m_d[1],			m_jointState[1].jointValue + mathLib::rad(-90)},// 1T2
																	{0,					m_a[0],		m_d[2],			m_jointState[2].jointValue},					// 2T3
																	{0,					m_a[1],		m_d[3],			m_jointState[3].jointValue},					// 3T4
																	{0,					m_a[2],		m_d[4],			mathLib::rad(90)},				// 4T4'
																	{mathLib::rad(90),	0,			0,				m_jointState[4].jointValue},					// 4'T5
																	{mathLib::rad(-90),	0,			0,				mathLib::rad(-90)},				// 5T5'
																	{0,					m_a[3],		m_d[5],			m_jointState[5].jointValue},					// 5'T6
																	{0,					0,			m_d[6],			0} };							// 6T7

	public:
		UR(const bool& endEffector = false, const float& endEffectorDimension = 0.0f);
		void setTipPose(const mathLib::tipPose &tipPose);
		pose forwardKinematics(const float(&targetJointVal)[]);
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
		const float getTheta(const int& ix) const;
		const pose getTipPose() const;
		//const float** getMDHmatrix() const;
	};


	std::ostream& operator <<(std::ostream& stream, universalRobots::URtype& type);

} // namespace universalRobots 