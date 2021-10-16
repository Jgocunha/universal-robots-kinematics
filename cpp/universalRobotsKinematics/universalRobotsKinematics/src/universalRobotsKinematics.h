// universalRobotsKinematics.h

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "mathLib.h"

#define UR3_LINK_DIMENSIONS_d { 0.1089f, 0.1115f, 0.0f, 0.0f, 0.0007f, 0.0818f, 0.0f }
#define UR3_LINK_DIMENSIONS_a { 0.2437f, 0.2132f, 0.0842f, 0.0011f }

#define UR5_LINK_DIMENSIONS_d { 0.0746f, 0.0703f, 0.0f, 0.0f, 0.0397f, 0.0829f, 0.0f }
#define UR5_LINK_DIMENSIONS_a { 0.4251f, 0.3922f, 0.0456f, 0.0492f }

#define UR10_LINK_DIMENSIONS_d { 0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f }
#define UR10_LINK_DIMENSIONS_a { 0.6121f, 0.5722f, 0.0573f, 0.0584f }

namespace universalRobots
{
	
	/// <summary> URtype
	/// Different types of URs.
	/// </summary>
	enum URtype : unsigned int
	{
		UR3, UR5, UR10 // 0, 1, 2
	};

	/// <summary>
	/// Structure which holds a pose { x y z } { alpha beta gamma }
	/// </summary>
	struct pose
	{
		float m_pos[3] = {}; // x y z (meters)
		float m_eulerAngles[3] = {}; // alpha beta gamma (radians)

		pose()
			: m_pos{ 0.0f, 0.0f, 0.0f }, m_eulerAngles{ 0.0f, 0.0f, 0.0f } {}

		pose(const float& pos1, const float& pos2, const float& pos3, const float& eulerAngles1, const float& eulerAngles2, const float& eulerAngles3)
			: m_pos{ pos1, pos2, pos3 }, m_eulerAngles{ eulerAngles1, eulerAngles1, eulerAngles1 } {}

		pose(const float(&pos)[], const float(&eulerAngles)[])
			: m_pos{ pos[0],  pos[1],  pos[2] }, m_eulerAngles{ eulerAngles[0], eulerAngles[1], eulerAngles[2] } {}

		pose(const float(&pos)[], const Eigen::Matrix3f& rotationMatrix)
			: m_pos{ pos[0],  pos[1],  pos[2] }, m_eulerAngles{ rotationMatrix.eulerAngles(1, 2, 0).z(), rotationMatrix.eulerAngles(1, 2, 0).y(), rotationMatrix.eulerAngles(1, 2, 0).x() } {}

		pose divideByConst(const float& constant) const
		{
			return pose(m_pos[0]/ constant, m_pos[1] / constant, m_pos[2] / constant, m_eulerAngles[0] / constant, m_eulerAngles[1] / constant, m_eulerAngles[2] / constant );
		}

		pose subtract(const pose& other) const
		{
			return pose(m_pos[0] - other.m_pos[0], m_pos[1] - other.m_pos[1], m_pos[2] - other.m_pos[2], m_eulerAngles[0] - other.m_eulerAngles[0], m_eulerAngles[1] - other.m_eulerAngles[1], m_eulerAngles[2] - other.m_eulerAngles[2]);
		}

		pose operator/(const float& constant) const
		{
			return divideByConst(constant);
		}

		pose operator-(const pose& other) const
		{
			return subtract(other);
		}

	};

	/// <summary>
	/// Structure which holds the current value and pose of a joint.
	/// </summary>
	struct joint
	{
		pose m_jointPose = {};
		float m_jointValue = 0.0f;

		joint()
			: m_jointPose(pose()), m_jointValue(0.0f) {}
	};
	
	/// <summary>
	/// Implements 'UR robot type' object.
	/// </summary>
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

		/// <summary>
		/// Modified Denavit-Hartenberg parameters matrix (9x4)
		/// { alpha_i-1, a_i-1, d_i, theta_i } for each frame.
		/// </summary>
		Eigen::Matrix<float, m_numReferenceFrames, 4> m_MDHmatrix { {0,					0,			m_d[0],			m_jointState[0].m_jointValue},
																	{mathLib::rad(-90),	0,			m_d[1],			m_jointState[1].m_jointValue + mathLib::rad(-90)},
																	{0,					m_a[0],		m_d[2],			m_jointState[2].m_jointValue},
																	{0,					m_a[1],		m_d[3],			m_jointState[3].m_jointValue},
																	{0,					m_a[2],		m_d[4],			mathLib::rad(90)},
																	{mathLib::rad(90),	0,			0,				m_jointState[4].m_jointValue},
																	{mathLib::rad(-90),	0,			0,				mathLib::rad(-90)},
																	{0,					m_a[3],		m_d[5],			m_jointState[5].m_jointValue},
																	{0,					0,			m_d[6],			0} };
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
		float m_d[m_numTransZ] = UR10_LINK_DIMENSIONS_d;
		
		/// <summary>
		/// a - translation (meters) in the x-axis.
		/// a_i is a nomeclature convention.
		/// </summary>
		float m_a[m_numTransX] = UR10_LINK_DIMENSIONS_a;
		
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
		UR(const URtype& robotType = UR10, const bool& endEffector = false, const float& endEffectorDimension = 0.0f);
		const URtype getRobotType() const;
		pose forwardKinematics(const float (&targetJointVal)[]);
		void inverseKinematics(const pose& targetTipPose, float(*outIkSols)[m_numIkSol][m_numDoF]);
		pose generateRandomReachablePose();
		bool checkPoseReachability(const float(&targetTipPose)[]) const;
		friend std::ostream& operator <<(std::ostream& stream, const universalRobots::UR& robot);
	private:
		void setMDHmatrix();
		void setRobotType(const URtype& type);
		void setTransZ(const float (&d)[]);
		void setTransX(const float (&a)[]);
		void setTheta(const float (&jointVal)[]);
		const float* getTransZ() const;
		const float* getTransX() const;
		const float getTheta(const int& ix) const;
		const pose getTipPose() const;
	};

	std::ostream& operator <<(std::ostream& stream, universalRobots::URtype& type);

	std::ostream& operator <<(std::ostream& stream, const universalRobots::UR& robot);

} // namespace universalRobots 