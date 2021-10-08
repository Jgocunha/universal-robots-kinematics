#include "universalRobotsKinematics.h"

namespace universalRobots
{
	UR::UR(const bool& endEffector, const float& endEffectorDimension)
		: m_endEffector(endEffector)
	{
		m_d[m_numTransZ - 1] = endEffectorDimension;
	}

	const URtype UR::getRobotType(void) const
	{
		return m_type;
	}
	void UR::setRobotType(const URtype& type)
	{
		m_type = type;
	}
	const float* UR::getTransZ(void) const
	{
		return m_d;
	}
	void UR::setTransZ(const float(&d)[])
	{
		memcpy(m_d, d, sizeof(m_d));
	}
	const float* UR::getTransX(void) const
	{
		return m_a;
	}
	void UR::setTransX(const float(&a)[])
	{
		memcpy(m_a, a, sizeof(m_a));
	}
	//const float** UR::getMDHmatrix(void) const
	//{
	//	return reinterpret_cast<const float**>(MDHmatrix);
	//}
	const float* UR::getTheta(void) const
	{
		return m_theta;
	}

	void UR::setTheta(const float(&jointVal)[])
	{
		std::memcpy(m_theta, jointVal, sizeof(m_theta));
	}
	void UR::setMDHmatrix(void)
	{
		using namespace mathLib;
		static float tempMat[m_numReferenceFrames][4] = { {0,		0,			m_d[0],			m_theta[0]},	// 0T1
												{ -rad(90),	0,			m_d[1],			m_theta[1] - rad(90) },	// 1T2
												{ 0,		m_a[0],		m_d[2],			m_theta[2] },			// 2T3
												{ 0,		m_a[1],		m_d[3],			m_theta[3] },			// 3T4
												{ 0,		m_a[2],		m_d[4],			rad(90) },				// 4T4'
												{ rad(90),	0,			0,				m_theta[4] },			// 4'T5
												{ -rad(90),	0,			0,				-rad(90) },				// 5T5'
												{ 0,		m_a[3],		m_d[5],			m_theta[5] },			// 5'T6
												{ 0,		0,			m_d[6],			0 } };					// 6T7

		std::memcpy(m_MDHmatrix, tempMat, sizeof(m_MDHmatrix));

	}
	const mathLib::tipPose UR::getTipPose(void) const
	{
		return m_tipPose;
	}


	mathLib::tipPose UR::forwardKinematics(const float(&targetJointVal)[])
	{
		// assign joint values to compute MDH matrix;
		setTheta(targetJointVal);
		setMDHmatrix();

		// create an array of transformation matrices
		mathLib::TransformationMatrix individualTransformationMatrices[m_numReferenceFrames] = {};
		// determine the indiviual transformation matrices
		for (unsigned int i = 0; i < m_numReferenceFrames; i++)
		{
			std::memcpy(individualTransformationMatrices[i].m_matrix, mathLib::calcTransformationMatrix(m_MDHmatrix[i]), sizeof(float[4][4]));
			//memcpy(individualTransformationMatrices[i].matrix, MDHMatrix(getMDHmatrix()[i]), sizeof(float[4][4]));
		}
		// create another array of transformation matrices
		mathLib::TransformationMatrix generalTransformationMatrices[m_numReferenceFrames] = {};
		// determine the general transformation matrix
		memcpy(generalTransformationMatrices[0].m_matrix, individualTransformationMatrices[0].m_matrix, sizeof(float[4][4]));
		for (unsigned int i = 1; i < m_numReferenceFrames; i++)
		{
			std::memcpy(generalTransformationMatrices[i].m_matrix, (generalTransformationMatrices[i - 1] * individualTransformationMatrices[i]), sizeof(float[4][4]));
		}

		//static 
		float rotationMatrix[3][3] = {  { generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[0][0], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[0][1], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[0][2] },
										{ generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[1][0], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[1][1], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[1][2] },
										{ generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[2][0], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[2][1], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[2][2] } };


		return mathLib::tipPose(mathLib::position::position(generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[0][3], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[1][3], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[2][3]), mathLib::RPY::RPY(rotationMatrix));
	}

	void UR::setTipPose(const mathLib::tipPose& newTipPose)
	{
		m_tipPose.m_pos_e = newTipPose.m_pos_e;
		m_tipPose.m_rpy_e = newTipPose.m_rpy_e;
	}
	
	// << overloading to be able to print a URtype enum
	std::ostream& operator <<(std::ostream& stream, universalRobots::URtype type)
	{
		switch (type)
		{
		case universalRobots::UR3:
			stream << "UR3";
			break;
		case universalRobots::UR5:
			stream << "UR5";
			break;
		case universalRobots::UR10:
			stream << "UR10";
			break;
		default:
			stream.setstate(std::ios_base::failbit);
		}
		return stream;
	}
	// << overloading to be able to print a UR
	std::ostream& operator <<(std::ostream& stream, const universalRobots::UR& robot)
	{
		stream << "Robot type: " << robot.getRobotType() << std::endl
			<< "Number of DoFs: " << robot.m_numDoF << std::endl
			<< "Link dimensions\n" << "Translations in the z-axis (meters)\n";
		for (unsigned int i = 0; i < robot.m_numTransZ; i++)
			stream << "d" << i + 1 << ": " << robot.getTransZ()[i] << std::endl;
		stream << "Translations in the x-axis (meters)\n";
		for (unsigned int i = 0; i < robot.m_numTransX; i++)
			stream << "a" << i + 2 << ": " << robot.getTransX()[i] << std::endl;
		//stream << "Modified Denavit-Hartengerg Matrix\n";
		//for (int x = 0; x < robot.m_numReferenceFrames; x++)  // loop lines
		//{
		//	for (int y = 0; y < 4; y++)  // loop columns
		//	{
		//		//stream << robot.getMDHmatrix()[x][y];
		//		stream << (robot.m_MDHmatrix[x][y]) << "				";
		//	}
		//	stream << endl;  
		//}
		stream << "Joint values (degress)\n";
		for (unsigned int i = 0; i < robot.m_numDoF; i++)
			stream << "Theta" << i + 1 << ": " << mathLib::deg(robot.getTheta()[i]) << std::endl;
		stream << "Tip pose:\n";
		stream << "x " << robot.getTipPose().m_pos_e.m_x << " y " << robot.getTipPose().m_pos_e.m_y << " z " << robot.getTipPose().m_pos_e.m_z << " (meters)\nalpha "
			<< mathLib::deg(robot.getTipPose().m_rpy_e.m_alpha) << " beta " << mathLib::deg(robot.getTipPose().m_rpy_e.m_beta) << " gamma " << mathLib::deg(robot.getTipPose().m_rpy_e.m_gamma) << " (degrees)" << std::endl;
		return stream;
	}
}




