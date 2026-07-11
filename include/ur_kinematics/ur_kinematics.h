// ur_kinematics.h

#pragma once

#include <iosfwd>
#include <array>
#include <Eigen/Dense>
#include "robot_parameters.h"
#include <stdexcept>

namespace universalRobots
{

	/// <summary>
	/// Converts an angle in degrees to radians.
	/// </summary>
	float rad(float degree);

	/// <summary>
	/// Converts an angle in radians to degrees.
	/// </summary>
	float deg(float rad);

	/// <summary>
	/// Structure which holds a pose { x y z } { alpha beta gamma }
	/// </summary>
	struct pose
	{
		float m_pos[3] = {};		 // x y z (meters)
		float m_eulerAngles[3] = {}; // alpha beta gamma (radians)

		pose() = default;

		pose(float pos1, float pos2, float pos3, float eulerAngles1, float eulerAngles2, float eulerAngles3)
			: m_pos{pos1, pos2, pos3}, m_eulerAngles{eulerAngles1, eulerAngles2, eulerAngles3}
		{
		}

		pose(const float (&pos)[3], const float (&eulerAngles)[3])
			: m_pos{pos[0], pos[1], pos[2]}, m_eulerAngles{eulerAngles[0], eulerAngles[1], eulerAngles[2]}
		{
		}

		pose(const float (&pos)[3], const Eigen::Matrix3f& rotationMatrix)
			: m_pos{pos[0], pos[1], pos[2]},
			  m_eulerAngles{rotationMatrix.eulerAngles(1, 2, 0).z(), rotationMatrix.eulerAngles(1, 2, 0).y(),
							rotationMatrix.eulerAngles(1, 2, 0).x()}
		{
		}

		pose divideByConst(float constant) const
		{
			return pose(m_pos[0] / constant, m_pos[1] / constant, m_pos[2] / constant, m_eulerAngles[0] / constant,
						m_eulerAngles[1] / constant, m_eulerAngles[2] / constant);
		}

		pose subtract(const pose& other) const
		{
			return pose(m_pos[0] - other.m_pos[0], m_pos[1] - other.m_pos[1], m_pos[2] - other.m_pos[2],
						m_eulerAngles[0] - other.m_eulerAngles[0], m_eulerAngles[1] - other.m_eulerAngles[1],
						m_eulerAngles[2] - other.m_eulerAngles[2]);
		}

		pose operator/(float constant) const
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
		/// Number of translations in the x-axis is always 4.
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
		/// Six joint angles (radians), in joint order 1..6.
		/// </summary>
		using JointVector = std::array<float, m_numDoF>;

		/// <summary>
		/// The eight inverse-kinematics solutions, each a full joint vector.
		/// valid[i] is true when solution i is geometrically feasible (no NaN angles).
		/// anyValid() returns true when at least one solution exists.
		/// For invalid rows, angle values are NaN — old-style consumers checking
		/// std::isnan() continue to work; valid[] is the authoritative flag.
		/// </summary>
		struct IkSolutions
		{
			std::array<std::array<float, m_numDoF>, m_numIkSol> solutions;
			/// false where the geometric solution does not exist
			std::array<bool, m_numIkSol> valid;
			bool anyValid() const
			{
				for (bool v : valid)
					if (v)
						return true;
				return false;
			}
		};

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
		std::array<float, m_numTransZ> m_d = kUR10.d;

		/// <summary>
		/// a - translation (meters) in the x-axis.
		/// a_i is a nomeclature convention.
		/// </summary>
		std::array<float, m_numTransX> m_a = kUR10.a;

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

		/// <summary>
		/// This boolean indicates whether a tool is/isnt attached to the robot.
		/// Must be specified in the constructor, if not default is false.
		/// </summary>
		bool m_endEffector = false;

		/// <summary>
		/// Modified Denavit-Hartenberg parameters matrix (9x4)
		/// { alpha_i-1, a_i-1, d_i, theta_i } for each frame.
		/// Populated by setMDHmatrix() from the constructor.
		/// </summary>
		Eigen::Matrix<float, m_numReferenceFrames, 4> m_MDHmatrix;

	  public:
		UR(URtype robotType = UR10, bool endEffector = false, float endEffectorDimension = 0.0f);
		URtype getRobotType() const;
		/// Precondition: every joint value must be finite and in [-2π, 2π].
		/// Throws std::invalid_argument otherwise.
		[[nodiscard]] pose forwardKinematics(const JointVector& targetJointVal);
		[[nodiscard]] IkSolutions inverseKinematics(const pose& targetTipPose);
		/// Returns true iff inverseKinematics(targetPose).anyValid().
		[[nodiscard]] bool isPoseReachable(const pose& targetPose);
		pose generateRandomReachablePose();
		[[nodiscard]] bool isSolutionValid(const std::array<float, m_numDoF>& ikSolution) const;
		friend std::ostream& operator<<(std::ostream& stream, const universalRobots::UR& robot);
		friend std::ostream& operator<<(std::ostream& stream, universalRobots::URtype type);

	  private:
		void setMDHmatrix();
		void setRobotType(URtype type);
		void setTheta(const JointVector& jointVal);
		const float* getTransZ() const;
		const float* getTransX() const;
		float getTheta(int ix) const;
		pose getTipPose() const;
	};

	std::ostream& operator<<(std::ostream& stream, universalRobots::URtype type);

	std::ostream& operator<<(std::ostream& stream, const universalRobots::UR& robot);

} // namespace universalRobots