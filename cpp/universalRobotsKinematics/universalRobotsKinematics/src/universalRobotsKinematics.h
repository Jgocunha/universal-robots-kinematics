// universalRobotsKinematics.h
#pragma once

#include <iostream>
#include "mathLib.h"

namespace universalRobots
{

	enum URtype : int
	{
		UR3, UR5, UR10 // 0, 1, 2
	};

	std::ostream& operator <<(std::ostream& stream, universalRobots::URtype type);

	class UR
	{
	public:
		static constexpr unsigned int m_numDoF = 6; // number of DoFs is always 6
		static constexpr unsigned int m_numTransZ = 7; // number of translations in the z-axis is always 7
		static constexpr unsigned int m_numTransX = 4; // number of translations in the z-axis is always 4
		static constexpr unsigned int m_numReferenceFrames = 9; // number of frames is pre-defined
		bool m_endEffector = false; // whether a tool is/isnt attached to the robot
	private:
		URtype m_type = UR10; // UR 3, 5, 10
		float m_d[m_numTransZ] = { 0.109f, 0.10122f, 0.01945f, -0.00661f, 0.0584f, 0.09372f, 0.0f }; // d - translation (meters) in the z-axis (the last translation d[6] is for an end-effector)
		float m_a[m_numTransX] = { 0.6121f, 0.5722f, 0.0573f, 0.0584f }; // a - translation (meters) in the x-axis
		float m_theta[m_numDoF] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; // joint values in rads
		mathLib::tipPose m_tipPose; // end-effector/tip position

	public:
		float m_MDHmatrix[m_numReferenceFrames][4] = { {0,					0,			m_d[0],			m_theta[0]},					// 0T1
														{mathLib::rad(-90),	0,			m_d[1],			m_theta[1] + mathLib::rad(-90)},// 1T2
														{0,					m_a[0],		m_d[2],			m_theta[2]},					// 2T3
														{0,					m_a[1],		m_d[3],			m_theta[3]},					// 3T4
														{0,					m_a[2],		m_d[4],			mathLib::rad(90)},				// 4T4'
														{mathLib::rad(90),	0,			0,				m_theta[4]},					// 4'T5
														{mathLib::rad(-90),	0,			0,				mathLib::rad(-90)},				// 5T5'
														{0,					m_a[3],		m_d[5],			m_theta[5]},					// 5'T6
														{0,					0,			m_d[6],			0} };							// 6T7

	public:
		// constructors
		UR(const bool& endEffector = false, const float& endEffectorDimension = 0.0f);
		// set methods
		void setTipPose(const mathLib::tipPose &tipPose);
		// other methods
		mathLib::tipPose forwardKinematics(const float(&jointVal)[]);
		friend std::ostream& operator <<(std::ostream& stream, const universalRobots::UR& robot);
	private:
		// set methods
		void setTheta(const float(&jointVal)[]);
		void setMDHmatrix(void);
		// get methods
		const URtype getRobotType(void) const;
		const float* getTransZ(void) const;
		const float* getTransX(void) const;
		const float* getTheta(void) const;
		const mathLib::tipPose getTipPose(void) const;
		//const float** getMDHmatrix(void) const;
		//set methods
		void setRobotType(const URtype& type);
		void setTransZ(const float(&d)[]);
		void setTransX(const float(&a)[]);
	};
}