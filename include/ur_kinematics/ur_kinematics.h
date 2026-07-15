// ur_kinematics.h

#pragma once

#include <iosfwd>
#include <array>
#include <algorithm>
#include <type_traits>
#include <Eigen/Dense>
#include "robot_parameters.h"
#include <stdexcept>

namespace universalRobots
{

	/** @brief Converts an angle in degrees to radians. */
	float rad(float degree);

	/** @brief Converts an angle in radians to degrees. */
	float deg(float rad);

	/** @brief Structure which holds a pose { x y z } { alpha beta gamma } */
	struct pose
	{
		std::array<float, 3> m_pos = {};		 // x y z (meters)
		std::array<float, 3> m_eulerAngles = {}; // alpha beta gamma (radians)

		pose() = default;

		// Public constructor kept as plain positional floats to match existing call
		// sites; splitting into named pos/euler types would be an API-breaking
		// redesign out of scope here.
		// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
		pose(float pos1, float pos2, float pos3, float eulerAngles1, float eulerAngles2, float eulerAngles3)
			: m_pos{pos1, pos2, pos3}, m_eulerAngles{eulerAngles1, eulerAngles2, eulerAngles3}
		{
		}

		// Same rationale as above.
		// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
		pose(const std::array<float, 3>& pos, const std::array<float, 3>& eulerAngles)
			: m_pos(pos), m_eulerAngles(eulerAngles)
		{
		}

		pose(const std::array<float, 3>& pos, const Eigen::Matrix3f& rotationMatrix) : m_pos(pos)
		{
			const auto euler = rotationMatrix.eulerAngles(1, 2, 0);
			m_eulerAngles = {euler.z(), euler.y(), euler.x()};
		}

		[[nodiscard]] pose divideByConst(float constant) const
		{
			return {m_pos[0] / constant,		 m_pos[1] / constant,		  m_pos[2] / constant,
					m_eulerAngles[0] / constant, m_eulerAngles[1] / constant, m_eulerAngles[2] / constant};
		}

		[[nodiscard]] pose subtract(const pose& other) const
		{
			return {m_pos[0] - other.m_pos[0],
					m_pos[1] - other.m_pos[1],
					m_pos[2] - other.m_pos[2],
					m_eulerAngles[0] - other.m_eulerAngles[0],
					m_eulerAngles[1] - other.m_eulerAngles[1],
					m_eulerAngles[2] - other.m_eulerAngles[2]};
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

	// #57: m_pos/m_eulerAngles moved from float[3] to std::array<float,3>; pose
	// must stay cheap to pass/return by value.
	static_assert(std::is_trivially_copyable_v<pose>, "pose must remain trivially copyable");

	/** @brief Structure which holds the current value and pose of a joint. */
	struct joint
	{
		pose m_jointPose;
		float m_jointValue = 0.0f;
	};

	/** @brief Implements 'UR robot type' object. */
	class UR
	{
	  public:
		/** @brief Number of Degrees of Freedom is always 6 for all URs. */
		static constexpr unsigned int m_numDoF = 6;

		/** @brief Number of translations in the z-axis is always 7 (this includes a translation for the end-effector).
		 */
		static constexpr unsigned int m_numTransZ = 7;

		/** @brief Number of translations in the x-axis is always 4. */
		static constexpr unsigned int m_numTransX = 4;

		/** @brief Number of frames is pre-defined (according to the MDH convention it is 9). */
		static constexpr unsigned int m_numReferenceFrames = 9;

		/** @brief Number of inverse kinematics solutions for a UR is 8. */
		static constexpr unsigned int m_numIkSol = 8;

		/** @brief Six joint angles (radians), in joint order 1..6. */
		using JointVector = std::array<float, m_numDoF>;

		/**
		 * @brief The eight inverse-kinematics solutions, each a full joint vector.
		 *
		 * valid[i] is true when solution i is geometrically feasible (no NaN angles).
		 * anyValid() returns true when at least one solution exists.
		 * For invalid rows, angle values are NaN — old-style consumers checking
		 * std::isnan() continue to work; valid[] is the authoritative flag.
		 */
		struct IkSolutions
		{
			std::array<std::array<float, m_numDoF>, m_numIkSol> solutions = {};
			/// false where the geometric solution does not exist
			std::array<bool, m_numIkSol> valid = {};
			[[nodiscard]] bool anyValid() const
			{
				return std::ranges::any_of(valid, [](bool isValid) { return isValid; });
			}
		};

	  private:
		/** @brief Robot type/name UR 3, 5, or 10. Values: 0, 1, 2. */
		URtype m_type = UR10;

		/**
		 * @brief d - translation (meters) in the z-axis (the last translation d[6] is for an end-effector).
		 *
		 * d_i is a nomeclature convention.
		 */
		std::array<float, m_numTransZ> m_d = kUR10.d;

		/**
		 * @brief a - translation (meters) in the x-axis.
		 *
		 * a_i is a nomeclature convention.
		 */
		std::array<float, m_numTransX> m_a = kUR10.a;

		// mutable: these four members are a cache of the last forwardKinematics()
		// computation, kept for operator<</pose accessors. forwardKinematics() is
		// logically const (its return value depends only on its argument and the
		// robot's fixed DH parameters); mutable lets it also refresh this cache
		// without losing const-callability. See forwardKinematics()'s doc comment.
		// Not thread-safe: the cache is updated without synchronization, so
		// concurrent const calls (e.g. forwardKinematics()) on the same UR
		// instance from multiple threads is a data race.

		/** @brief Position, Euler Angles, and Value of the robot's joints. {x, y, z} {alpha, beta, gamma} {jointValue}
		 */
		mutable std::array<joint, m_numDoF> m_jointState = {};

		// Create an array of (individual) transformation matrices iTi+1 / i-1Ti

		/** @brief Array of (individual) transformation matrices iTi+1 / i-1Ti */
		mutable std::array<Eigen::Matrix4f, m_numReferenceFrames> m_individualTransformationMatrices = {};

		/** @brief Array of (general) transformation matrices 0Ti */
		mutable std::array<Eigen::Matrix4f, m_numReferenceFrames> m_generalTransformationMatrices = {};

		/**
		 * @brief Modified Denavit-Hartenberg parameters matrix (9x4)
		 * { alpha_i-1, a_i-1, d_i, theta_i } for each frame.
		 *
		 * Populated by setMDHmatrix() from the constructor.
		 */
		mutable Eigen::Matrix<float, m_numReferenceFrames, 4> m_MDHmatrix;

	  public:
		UR(URtype robotType = UR10, bool endEffector = false, float endEffectorDimension = 0.0f);
		[[nodiscard]] URtype getRobotType() const;
		/// Precondition: every joint value must be finite and in [-2π, 2π].
		/// Throws std::invalid_argument otherwise.
		/// Note: also refreshes the joint-pose/transform cache used by operator<<
		/// and the joint-pose accessors (see the mutable members above) -- callable
		/// on a const UR, but not side-effect-free with respect to that cache.
		[[nodiscard]] pose forwardKinematics(const JointVector& targetJointVal) const;
		[[nodiscard]] IkSolutions inverseKinematics(const pose& targetTipPose) const;
		/// Returns true iff inverseKinematics(targetPose).anyValid().
		[[nodiscard]] bool isPoseReachable(const pose& targetPose) const;
		pose generateRandomReachablePose() const;
		[[nodiscard]] static bool isSolutionValid(const std::array<float, m_numDoF>& ikSolution);
		friend std::ostream& operator<<(std::ostream& stream, const universalRobots::UR& robot);
		friend std::ostream& operator<<(std::ostream& stream, universalRobots::URtype type);

	  private:
		void setMDHmatrix() const;
		void setTheta(const JointVector& jointVal) const;
		[[nodiscard]] const std::array<float, m_numTransZ>& getTransZ() const;
		[[nodiscard]] const std::array<float, m_numTransX>& getTransX() const;
		[[nodiscard]] float getTheta(int ix) const;
		// `pose` is 6 floats (24 bytes), trivially copyable; a const-reference
		// return would couple the caller to this object's lifetime for no
		// measurable performance win.
		// cppcheck-suppress returnByReference
		[[nodiscard]] pose getTipPose() const;

		// Helpers for operator<<(std::ostream&, const UR&); split out to keep each
		// print section (and its frame/loop bookkeeping) independently readable.
		static void printLinkDimensions(std::ostream& stream, const UR& robot);
		static void printJointValues(std::ostream& stream, const UR& robot);
		static void printTransforms(std::ostream& stream, const UR& robot);
		static void printJointPoses(std::ostream& stream, const UR& robot);
	};

	std::ostream& operator<<(std::ostream& stream, universalRobots::URtype type);

	std::ostream& operator<<(std::ostream& stream, const universalRobots::UR& robot);

} // namespace universalRobots