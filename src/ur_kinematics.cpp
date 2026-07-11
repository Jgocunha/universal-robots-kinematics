// ur_kinematics.cpp

#include <ur_kinematics/ur_kinematics.h>
#include "math_utils.h"
#include <ostream>
#include <random>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace
{
	// Membership test for the small frame/solution index sets below.
	template <std::size_t N> constexpr bool contains(const std::array<unsigned int, N>& set, unsigned int value)
	{
		for (const unsigned int element : set)
			if (element == value)
				return true;
		return false;
	}

	// Of the 9 Modified-DH reference frames, these are the ones whose general
	// transform yields a joint pose (frames 0..5 -> joints 1..6) or the tip pose
	// (frame 8). Frames 4 and 6 are the intermediate 4'/5' frames with no pose.
	constexpr std::array<unsigned int, 7> kPoseBearingFrames{0, 1, 2, 3, 5, 7, 8};

	// The IK solutions whose wrist is in the "up" configuration; these take
	// +acos for theta5, the others take -acos.
	constexpr std::array<unsigned int, 4> kWristUpSolutionIndices{0, 1, 4, 5};

	// Solutions alternate elbow configuration; these (odd) indices take the
	// theta3 = pi - psi branch, the even indices take theta3 = pi + psi.
	constexpr std::array<unsigned int, 4> kSecondElbowSolutionIndices{1, 3, 5, 7};
} // namespace

namespace universalRobots
{
	/// <summary>
	/// Constructor. User is only allowed to specify whether there is an end-effector and its translation to the tip.
	/// Example usage:
	///		universalRobots::UR robot_one(); Robot does not have an end-effector.
	///		universalRobots::UR robot_one(true, 0.15f); End-effector translated 0.15 meters from the robot's tip.
	/// </summary>
	/// <param name="endEffector"></param>
	/// <param name="endEffectorDimension"></param>
	UR::UR(URtype robotType, bool endEffector, float endEffectorDimension)
		: m_type(robotType), m_endEffector(endEffector)
	{
		const RobotParameters& params = parametersFor(robotType);
		m_d = params.d;
		m_a = params.a;

		m_d[m_numTransZ - 1] = endEffectorDimension;
		setMDHmatrix();
	}

	/// <summary>
	/// setRobotType
	/// </summary>
	/// <param name="type"></param>
	void UR::setRobotType(URtype type)
	{
		m_type = type;
	}

	/// <summary>
	/// setMDHmatrix
	///	</summary>
	void UR::setMDHmatrix()
	{
		m_MDHmatrix << 0.0f, 0.0f, m_d[0], m_jointState[0].m_jointValue,									   // 0T1
			universalRobots::rad(-90), 0.0f, m_d[1], m_jointState[1].m_jointValue + universalRobots::rad(-90), // 1T2
			0.0f, m_a[0], m_d[2], m_jointState[2].m_jointValue,												   // 2T3
			0.0f, m_a[1], m_d[3], m_jointState[3].m_jointValue,												   // 3T4
			0.0f, m_a[2], m_d[4], universalRobots::rad(90),													   // 4T4'
			universalRobots::rad(90), 0.0f, 0.0f, m_jointState[4].m_jointValue,								   // 4'T5
			universalRobots::rad(-90), 0.0f, 0.0f, universalRobots::rad(-90),								   // 5T5'
			0.0f, m_a[3], m_d[5], m_jointState[5].m_jointValue,												   // 5'T6
			0.0f, 0, m_d[6], 0;																				   // 6T7
	}

	/// <summary>
	/// setTheta
	/// </summary>
	/// <param name="jointVal"></param>
	void UR::setTheta(const JointVector& jointVal)
	{
		for (unsigned int i = 0; i < m_numDoF; i++)
			m_jointState[i].m_jointValue = jointVal[i];
	}

	/// <summary>
	/// Returns the enum URtype. Used for printing purposes.
	/// </summary>
	/// <returns>m_type</returns>
	URtype UR::getRobotType() const
	{
		return m_type;
	}

	/// <summary>
	/// Returns the values of the z-axis translations (d array). Used for printing purposes.
	/// </summary>
	/// <returns>m_d</returns>
	const float* UR::getTransZ() const
	{
		return m_d.data();
	}

	/// <summary>
	/// Returns the values of the x-axis translations (a array). Used for printing purposes.
	/// </summary>
	/// <returns>m_a</returns>
	const float* UR::getTransX() const
	{
		return m_a.data();
	}

	/// <summary>
	/// Returns the robot's current joint values. Used for printing purposes.
	/// </summary>
	/// <returns>m_theta</returns>
	float UR::getTheta(int ix) const
	{
		return m_jointState[ix].m_jointValue;
	}

	/// <summary>
	/// Returns the robot's current tip pose. Used for printing purposes.
	/// </summary>
	/// <returns>m_tipPose</returns>
	pose UR::getTipPose() const
	{
		return m_jointState[m_numDoF - 1].m_jointPose;
	}

	/// <summary>
	/// Receives an array of target joint values and computes the pose of the robot's tip.
	/// </summary>
	/// <param name="targetJointVal"></param>
	/// <returns>tipPose</returns>
	pose UR::forwardKinematics(const JointVector& targetJointVal)
	{
		constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
		for (unsigned int i = 0; i < m_numDoF; ++i)
		{
			const float j = targetJointVal[i];
			if (!std::isfinite(j))
				throw std::invalid_argument("joint " + std::to_string(i) + " is not finite");
			if (j < -kTwoPi || j > kTwoPi)
				throw std::invalid_argument("joint " + std::to_string(i) + " out of range [-2pi, 2pi]");
		}

		// Assign joint values to compute MDH matrix.
		setTheta(targetJointVal);
		setMDHmatrix();

		// Determine the indiviual transformation matrices.
		for (unsigned int i = 0; i < m_numReferenceFrames; i++)
			m_individualTransformationMatrices[i] = universalRobots::calcTransformationMatrix(m_MDHmatrix.row(i));

		// Determine the general transformation matrices.
		Eigen::Matrix3f rotationMatrix;
		unsigned int currentJoint = 0;
		pose tipPose = {};
		for (unsigned int i = 0; i < m_numReferenceFrames; i++)
		{
			if (!i)
				m_generalTransformationMatrices[0] = m_individualTransformationMatrices[0];
			else
				m_generalTransformationMatrices[i] =
					m_generalTransformationMatrices[i - 1] * m_individualTransformationMatrices[i];

			// Obtaining the joint pose
			// Since we have more reference frames than joints only some represent a joint pose
			// 0T1 J1 // 1T2 J2// 2T3 J3// 3T4 J4// 4T4' XX// 4'T5 J5// 5T5' XX// 5'T6 J6// 6T7 tipPose
			if (contains(kPoseBearingFrames, i))
			{
				// 0T1 1 // 1T2 2// 2T3 3// 3T4 4// 4T4' // 4'T5 5// 5T5' // 5'T6 6// 6T7
				rotationMatrix << m_generalTransformationMatrices[i](0, 0), m_generalTransformationMatrices[i](0, 1),
					m_generalTransformationMatrices[i](0, 2), m_generalTransformationMatrices[i](1, 0),
					m_generalTransformationMatrices[i](1, 1), m_generalTransformationMatrices[i](1, 2),
					m_generalTransformationMatrices[i](2, 0), m_generalTransformationMatrices[i](2, 1),
					m_generalTransformationMatrices[i](2, 2);

				float position[3] = {m_generalTransformationMatrices[i](0, 3), m_generalTransformationMatrices[i](1, 3),
									 m_generalTransformationMatrices[i](2, 3)};

				// Tip pose
				if (i == 8)
					tipPose = pose(position, rotationMatrix);
				else
				{
					m_jointState[currentJoint].m_jointPose = pose(position, rotationMatrix);
					currentJoint++;
				}
			}
		}

		// return m_jointState[m_numDoF-1].m_jointPose;
		return tipPose;
	}

	/// <summary>
	/// Computes the eight inverse kinematics solutions for a given tip pose.
	/// </summary>
	/// <param name="targetTipPose"></param>
	/// <param name="outIkSols"></param>
	UR::IkSolutions UR::inverseKinematics(const pose& targetTipPose)
	{
		IkSolutions outIkSols = {};
		outIkSols.valid.fill(true); // start optimistic; mark false at each failed acos site

		constexpr float kAcosEps = 1e-6f; // clamp when |arg| in (1, 1+eps]; invalid beyond

		Eigen::Matrix4f T_07 = Eigen::Matrix4f::Identity(); // 0T7

		// Get translation matrix.
		const Eigen::Vector3f translation = {targetTipPose.m_pos[0], targetTipPose.m_pos[1], targetTipPose.m_pos[2]};

		// Calculate rotation matrix.
		Eigen::Matrix3f rotation;
		rotation = Eigen::AngleAxisf(targetTipPose.m_eulerAngles[0], Eigen::Vector3f::UnitX()) *
				   Eigen::AngleAxisf(targetTipPose.m_eulerAngles[1], Eigen::Vector3f::UnitY()) *
				   Eigen::AngleAxisf(targetTipPose.m_eulerAngles[2], Eigen::Vector3f::UnitZ());

		T_07.block<3, 3>(0, 0) = rotation;
		T_07.block<3, 1>(0, 3) = translation;

		// Computing theta1.
		const Eigen::Matrix<float, 1, 4> P_05 =
			T_07 * Eigen::Matrix<float, 1, 4>(0.0f, 0.0f, -m_d[5] - m_d[6], 1.0f)
					   .transpose(); // 0P5 position of reference frame {5} in relation to {0}
		const float theta1_psi = atan2(P_05(0, 1), P_05(0, 0));

		// There are two possible solutions for theta1, that depend on whether the shoulder joint (joint 2) is left or
		// right. acos site 1: theta1_phi — if |arg| > 1+eps, all 8 solutions are invalid. NOTE: the domain check below
		// uses its own copy of the acos argument rather than a shared named variable. Routing the in-domain acos() call
		// through a materialized float intermediate (instead of the original inline expression) forces an extra float32
		// rounding step that, right at this near-singular domain edge (|arg|≈1, where d(acos)/dx diverges), measurably
		// shifts the result vs the golden baseline. Keeping the acos() call on the untouched inline expression
		// preserves bit-identical valid-row output.
		const bool phi_inDomain = std::abs((m_d[1] + m_d[2] + m_d[3] + m_d[4]) /
										   (sqrt(pow(P_05(0, 1), 2) + pow(P_05(0, 0), 2)))) <= 1.0 + kAcosEps;
		if (!phi_inDomain)
			outIkSols.valid.fill(false);
		const float theta1_phi =
			phi_inDomain
				? acos(std::clamp((m_d[1] + m_d[2] + m_d[3] + m_d[4]) / (sqrt(pow(P_05(0, 1), 2) + pow(P_05(0, 0), 2))),
								  -1.0, 1.0))
				: std::numeric_limits<float>::quiet_NaN();

		for (int i = -4; i < int(m_numIkSol) - 4; i++)
		{
			outIkSols.solutions[i + 4][0] =
				(std::numbers::pi_v<float> / 2 + theta1_psi + (i < 0 ? 1 : -1) * theta1_phi) -
				std::numbers::pi_v<float>; // (i < 0 ? 1 : -1) first 4 theta1 values have positive phi
		}

		Eigen::Matrix4f T_06 = T_07;
		T_06(2, 3) = T_07(2, 3) - m_d[m_numTransZ - 1]; // 0T6

		for (unsigned int i = 0; i < m_numIkSol; i++)
		{
			// Computing theta5.
			Eigen::Matrix4f T_01 = universalRobots::calcTransformationMatrix(Eigen::RowVector4f{
				0.0f, 0.0f, m_d[0], outIkSols.solutions[i][0]}); // Knowing theta1 it is possible to know 0T1
			Eigen::Matrix4f T_16 = T_01.inverse() * T_06;		 // 1T6 = 1T0 * 0T6

			// acos site 2: theta5 — if |arg| > 1+eps, solution i is invalid.
			const float t5_arg = (T_16(1, 3) - (m_d[1] + m_d[2] + m_d[3] + m_d[4])) / m_d[5];
			if (std::abs(t5_arg) <= 1.0f + kAcosEps)
			{
				const float t5 = acos(std::clamp(t5_arg, -1.0f, 1.0f));
				outIkSols.solutions[i][4] = contains(kWristUpSolutionIndices, i) ? t5 : -t5;
			}
			else
			{
				outIkSols.solutions[i][4] = std::numeric_limits<float>::quiet_NaN();
				outIkSols.valid[i] = false;
			}

			// theta5==0/pi is a wrist singularity: the joint-6 axis becomes (anti)parallel to
			// the joint-4 axis, so theta6 alone is underdetermined (the atan2 below divides by
			// sin(theta5)==0) while theta1..theta5 stay uniquely determined by the target pose.
			// Convention (task 04f): pin theta6 = 0 at the singularity; theta3/theta2/theta4 are
			// then solved consistently for that choice downstream via T_46. Away from the
			// singularity the ordinary formula below remains numerically well-conditioned (the
			// division is by a small but genuine, non-zero denominator) and is used unchanged.
			//
			// theta5 == +pi and theta5 == -pi are the *same* physical angle, so right at this
			// singularity a sub-ULP difference in acos()/sin() between platforms/compilers can
			// flip which side of pi is computed, flipping the sign of sin(theta5) and sending the
			// formula below to a different branch entirely (observed: macOS libm vs Windows/Linux
			// on this exact case). kPiSingularityEps guards the pi branch specifically — it must
			// stay well below reachable-but-non-singular theta5 values (empirically ~1e-3 rad in
			// the golden set) so it does not also swallow those.
			constexpr float kPiSingularityEps = 1e-4f;
			const float theta5 = outIkSols.solutions[i][4];
			const bool nearPiSingularity = std::abs(std::abs(theta5) - std::numbers::pi_v<float>) < kPiSingularityEps;

			// Computing theta6.
			if (theta5 == 0 || theta5 == 2 * std::numbers::pi_v<float> ||
				nearPiSingularity)				  // If theta5 is at the singularity (0 or +-pi).
				outIkSols.solutions[i][5] = 0.0f; // Wrist singularity: theta6 pinned to 0 by convention.
			else
			{
				const float sinTheta5 = sin(theta5);
				outIkSols.solutions[i][5] = std::numbers::pi_v<float> / 2 +
											atan2(-T_16.inverse()(1, 1) / sinTheta5, T_16.inverse()(0, 1) / sinTheta5);
			}

			// Computing theta3, theta2, and theta4.

			// T_45 = T_44'*T_4'5
			Eigen::Matrix4f T_44_ = universalRobots::calcTransformationMatrix(
				Eigen::RowVector4f(0.0f, m_a[2], m_d[4], std::numbers::pi_v<float> / 2));
			Eigen::Matrix4f T_4_5 = universalRobots::calcTransformationMatrix(
				Eigen::RowVector4f(std::numbers::pi_v<float> / 2, 0.0f, 0.0f, outIkSols.solutions[i][4]));
			Eigen::Matrix4f T_45 = T_44_ * T_4_5;

			// T_56 = T_55'*T_5'6
			Eigen::Matrix4f T_55_ = universalRobots::calcTransformationMatrix(
				Eigen::RowVector4f(-std::numbers::pi_v<float> / 2, 0.0f, 0.0f, -std::numbers::pi_v<float> / 2));
			Eigen::Matrix4f T_5_6 = universalRobots::calcTransformationMatrix(
				Eigen::RowVector4f(0.0f, m_a[3], m_d[5], outIkSols.solutions[i][5]));
			Eigen::Matrix4f T_56 = T_55_ * T_5_6;

			Eigen::Matrix4f T_46 = T_45 * T_56;
			Eigen::Matrix4f T_14 = T_16 * T_46.inverse();

			float P_14_xz = sqrtf(pow(T_14(0, 3), 2) + pow(T_14(2, 3), 2));

			// acos site 3: theta3_psi — if |arg| > 1+eps, solution i is invalid.
			// See the site-1 note above: acos() is called on the untouched inline expression
			// (clamped with double bounds) rather than a materialized float intermediate, to
			// avoid an extra precision-losing rounding step right at this near-singular edge.
			const bool psi_inDomain = std::abs((pow(P_14_xz, 2) - pow(m_a[1], 2) - pow(m_a[0], 2)) /
											   (-2 * m_a[0] * m_a[1])) <= 1.0 + kAcosEps;
			if (!psi_inDomain)
				outIkSols.valid[i] = false;
			const float theta3_psi =
				psi_inDomain
					? acos(std::clamp((pow(P_14_xz, 2) - pow(m_a[1], 2) - pow(m_a[0], 2)) / (-2 * m_a[0] * m_a[1]),
									  -1.0, 1.0))
					: std::numeric_limits<float>::quiet_NaN();

			// Elbow up or down
			if (contains(kSecondElbowSolutionIndices, i))
			{
				// Computing theta3.
				outIkSols.solutions[i][2] = std::numbers::pi_v<float> - theta3_psi;
				// Masking theta3 for CoppeliaSim(invert value for ang > 180).
				if (outIkSols.solutions[i][2] > std::numbers::pi_v<float>)
					outIkSols.solutions[i][2] = outIkSols.solutions[i][2] - std::numbers::pi_v<float> * 2;
				// Computing theta2.
				outIkSols.solutions[i][1] = std::numbers::pi_v<float> / 2 - atan2(T_14(2, 3), T_14(0, 3)) +
											asin((m_a[1] * sin(-theta3_psi)) / P_14_xz);
				// Computing theta4.
				Eigen::Matrix4f T_12 = universalRobots::calcTransformationMatrix(
					Eigen::RowVector4f(-std::numbers::pi_v<float> / 2, 0.0f, m_d[1],
									   outIkSols.solutions[i][1] - std::numbers::pi_v<float> / 2));
				Eigen::Matrix4f T_23 = universalRobots::calcTransformationMatrix(
					Eigen::RowVector4f(0.0f, m_a[0], m_d[2], outIkSols.solutions[i][2]));
				Eigen::Matrix4f T_03 = T_01 * T_12 * T_23;

				Eigen::Matrix4f T_36 = T_03.inverse() * T_06;
				Eigen::Matrix4f T_34 = T_36 * T_46.inverse();

				outIkSols.solutions[i][3] = atan2(T_34(1, 0), T_34(0, 0));
			}
			else
			{
				// Computing theta3.
				outIkSols.solutions[i][2] = std::numbers::pi_v<float> + theta3_psi;
				// Masking theta3 for CoppeliaSim(invert value for ang > 180).
				if (outIkSols.solutions[i][2] > std::numbers::pi_v<float>)
					outIkSols.solutions[i][2] = outIkSols.solutions[i][2] - std::numbers::pi_v<float> * 2;
				// Computing theta2.
				outIkSols.solutions[i][1] = std::numbers::pi_v<float> / 2 - atan2(T_14(2, 3), T_14(0, 3)) +
											asin(m_a[1] * sin(theta3_psi) / P_14_xz);
				// Computing theta4.
				Eigen::Matrix4f T_12 = universalRobots::calcTransformationMatrix(
					Eigen::RowVector4f(-std::numbers::pi_v<float> / 2, 0.0f, m_d[1],
									   outIkSols.solutions[i][1] - std::numbers::pi_v<float> / 2));
				Eigen::Matrix4f T_23 = universalRobots::calcTransformationMatrix(
					Eigen::RowVector4f(0.0f, m_a[0], m_d[2], outIkSols.solutions[i][2]));
				Eigen::Matrix4f T_03 = T_01 * T_12 * T_23;

				Eigen::Matrix4f T_36 = T_03.inverse() * T_06;
				Eigen::Matrix4f T_34 = T_36 * T_46.inverse();

				outIkSols.solutions[i][3] = atan2(T_34(1, 0), T_34(0, 0));
			}
		}

		return outIkSols;
	}

	bool UR::isPoseReachable(const pose& targetPose)
	{
		return inverseKinematics(targetPose).anyValid();
	}

	/// <summary>
	/// Generates a valid tip pose by running forward kinematics with random target joint values.
	/// </summary>
	/// <returns>randomValidTargetPose</returns>
	pose UR::generateRandomReachablePose()
	{
		//// https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<> distrib(-360, 360); // URs joints limits [-360; 360]

		JointVector randomTargetJointValue = {};

		for (unsigned int i = 0; i < m_numDoF; i++)
		{
			randomTargetJointValue[i] = universalRobots::rad(distrib(gen));
		}

		return forwardKinematics(randomTargetJointValue);
	}

	/// <summary>
	/// Checks whether an inverse-kinematics solution is valid (contains no NaN entries).
	/// </summary>
	/// <param name="ikSolution"></param>
	/// <returns>bool</returns>
	bool UR::isSolutionValid(const std::array<float, m_numDoF>& ikSolution) const
	{
		for (unsigned int i = 0; i < m_numDoF; i++)
		{
			if (std::isnan(ikSolution[i]))
				return false;
		}

		return true;
	}

	/// <summary>
	/// Operator overloading to be able to print a URtype enum.
	/// </summary>
	/// <param name="stream"></param>
	/// <param name="type"></param>
	/// <returns>stream</returns>
	std::ostream& operator<<(std::ostream& stream, universalRobots::URtype type)
	{
		switch (type)
		{
		case universalRobots::URtype::UR3:
			stream << "UR3";
			break;
		case universalRobots::URtype::UR5:
			stream << "UR5";
			break;
		case universalRobots::URtype::UR10:
			stream << "UR10";
			break;
		default:
			stream.setstate(std::ios_base::failbit);
		}
		return stream;
	}

	/// <summary>
	/// Operator overloading to be able to print a UR object.
	/// </summary>
	/// <param name="stream"></param>
	/// <param name="robot"></param>
	/// <returns>stream</returns>
	std::ostream& operator<<(std::ostream& stream, const universalRobots::UR& robot)
	{
		stream << "Robot type: " << robot.m_type << std::endl
			   << "Number of DoFs: " << robot.m_numDoF << std::endl
			   << "Link dimensions\n"
			   << "Translations in the z-axis (meters):\n";
		for (unsigned int i = 0; i < robot.m_numTransZ; i++)
			stream << "d" << i + 1 << ": " << robot.m_d[i] << std::endl;
		stream << "Translations in the x-axis (meters):\n";
		for (unsigned int i = 0; i < robot.m_numTransX; i++)
			stream << "a" << i + 2 << ": " << robot.m_a[i] << std::endl;
		stream << "Joint values (degrees):\n";
		for (unsigned int i = 0; i < robot.m_numDoF; i++)
			stream << "Theta" << i + 1 << ": " << universalRobots::deg(robot.getTheta(i)) << std::endl;
		stream << "Tip pose:\n";
		const pose tipPose = robot.getTipPose();
		stream << "x " << tipPose.m_pos[0] << " y " << tipPose.m_pos[1] << " z " << tipPose.m_pos[2]
			   << " (meters)\nalpha " << universalRobots::deg(tipPose.m_eulerAngles[0]) << " beta "
			   << universalRobots::deg(tipPose.m_eulerAngles[1]) << " gamma "
			   << universalRobots::deg(tipPose.m_eulerAngles[2]) << " (degrees)" << std::endl;
		stream << "Individual Transformation Matrices:\n";
		unsigned int counter = 0;
		for (unsigned int i = 0; i < robot.m_numReferenceFrames; i++)
		{
			if (i == 0 || i == 1 || i == 2 || i == 3 || i == 8)
			{
				stream << counter << "T" << counter + 1 << std::endl
					   << robot.m_individualTransformationMatrices[i] << std::endl;
				counter++;
			}
			else
			{
				if (i == 4)
					stream << counter << "T" << counter << "'" << std::endl
						   << robot.m_individualTransformationMatrices[i] << std::endl;
				if (i == 5)
				{
					stream << counter << "'T" << counter + 1 << std::endl
						   << robot.m_individualTransformationMatrices[i] << std::endl;
					counter++;
				}
				if (i == 6)
					stream << counter << "T" << counter << "'" << std::endl
						   << robot.m_individualTransformationMatrices[i] << std::endl;
				if (i == 7)
				{
					stream << counter << "'T" << counter + 1 << std::endl
						   << robot.m_individualTransformationMatrices[i] << std::endl;
					counter++;
				}
			}
		}
		stream << "General Transformation Matrices:\n";
		counter = 1;
		for (unsigned int i = 0; i < robot.m_numReferenceFrames; i++)
		{
			if (i == 0 || i == 1 || i == 2 || i == 3 || i == 5 || i == 7 || i == 8)
			{
				stream << "0T" << counter << std::endl << robot.m_generalTransformationMatrices[i] << std::endl;
				counter++;
			}

			else
			{
				if (i == 4)
					stream << "0T" << counter - 1 << "'" << std::endl
						   << robot.m_generalTransformationMatrices[i] << std::endl;
				if (i == 6)
					stream << "0T" << counter - 1 << "'" << std::endl
						   << robot.m_generalTransformationMatrices[i] << std::endl;
			}
		}
		stream << "Joint poses: {x, y, z} metres {alpha, beta, gamma} degrees\n";
		for (unsigned int i = 0; i < robot.m_numDoF; i++)
			stream << "J" << i + 1 << ": {" << robot.m_jointState[i].m_jointPose.m_pos[0] << ", "
				   << robot.m_jointState[i].m_jointPose.m_pos[1] << ", " << robot.m_jointState[i].m_jointPose.m_pos[2]
				   << "}"
				   << " {" << universalRobots::deg(robot.m_jointState[i].m_jointPose.m_eulerAngles[0]) << ", "
				   << universalRobots::deg(robot.m_jointState[i].m_jointPose.m_eulerAngles[1]) << ", "
				   << universalRobots::deg(robot.m_jointState[i].m_jointPose.m_eulerAngles[2]) << "}" << std::endl;

		return stream;
	}

} // namespace universalRobots