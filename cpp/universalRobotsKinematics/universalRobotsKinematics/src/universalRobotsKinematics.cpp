// universalRobotsKinematics.cpp

#include "universalRobotsKinematics.h"


Eigen::Matrix4f calcTransformationMatrix(const Eigen::Matrix<float,1 , 4> &DHparams)
{
	Eigen::Matrix4f individualTransformationMatrix;
	individualTransformationMatrix  <<	cos(DHparams[3]),								-sin(DHparams[3]),							0,							DHparams[1],
										(sin(DHparams[3]) * cos(DHparams[0])),		(cos(DHparams[3]) * cos(DHparams[0])),	    -sin(DHparams[0]),	    (-sin(DHparams[0]) * DHparams[2]),
										(sin(DHparams[3]) * sin(DHparams[0])),		(cos(DHparams[3]) * sin(DHparams[0])),	    cos(DHparams[0]),		(cos(DHparams[0]) * DHparams[2]),
										0,														0,									0,							1;
	return individualTransformationMatrix;
}


namespace universalRobots
{
	UR::UR(const bool& endEffector, const float& endEffectorDimension)
		: m_endEffector(endEffector)
	{
		m_d[m_numTransZ - 1] = endEffectorDimension;
		m_MDHmatrix <<  0.0f,				0.0f,		m_d[0],			m_theta[0] ,					// 0T1
						mathLib::rad(-90),	0.0f,		m_d[1],			m_theta[1] + mathLib::rad(-90) ,// 1T2
						0.0f,				m_a[0],		m_d[2],			m_theta[2] ,					// 2T3
						0.0f,				m_a[1],		m_d[3],			m_theta[3] ,					// 3T4
						0.0f,				m_a[2],		m_d[4],			mathLib::rad(90) ,				// 4T4'
						mathLib::rad(90),	0.0f,		0.0f,			m_theta[4] ,					// 4'T5
						mathLib::rad(-90),	0.0f,		0.0f,			mathLib::rad(-90) ,				// 5T5'
						0.0f,				m_a[3],		m_d[5],			m_theta[5] ,					// 5'T6
						0.0f,				0,			m_d[6],			0 ;								// 6T7
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
		m_MDHmatrix << 0.0f,					0.0f,		m_d[0],			m_theta[0] ,					// 0T1
						mathLib::rad(-90),		0.0f,		m_d[1],			m_theta[1] + mathLib::rad(-90) ,// 1T2
						0.0f,					m_a[0],		m_d[2],			m_theta[2] ,					// 2T3
						0.0f,					m_a[1],		m_d[3],			m_theta[3] ,					// 3T4
						0.0f,					m_a[2],		m_d[4],			mathLib::rad(90) ,				// 4T4'
						mathLib::rad(90),		0.0f,		0.0f,			m_theta[4] ,					// 4'T5
						mathLib::rad(-90),		0.0f,		0.0f,			mathLib::rad(-90) ,				// 5T5'
						0.0f,					m_a[3],		m_d[5],			m_theta[5] ,					// 5'T6
						0.0f,					0,			m_d[6],			0 ;								// 6T7
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
		Eigen::Matrix4f individualTransformationMatrices[m_numReferenceFrames] = {};
		// determine the indiviual transformation matrices
		for (unsigned int i = 0; i < m_numReferenceFrames; i++)
		{
			individualTransformationMatrices[i] = calcTransformationMatrix(m_MDHmatrix.row(i));
		}
		// create another array of transformation matrices
		Eigen::Matrix4f generalTransformationMatrices[m_numReferenceFrames] = {};
		// determine the general transformation matrix
		generalTransformationMatrices[0] = individualTransformationMatrices[0];
		for (unsigned int i = 1; i < m_numReferenceFrames; i++)
		{
			generalTransformationMatrices[i] = generalTransformationMatrices[i - 1] * individualTransformationMatrices[i];
		}

		Eigen::Matrix3f rotationMatrix;
		rotationMatrix << generalTransformationMatrices[m_numReferenceFrames - 1](0, 0),	generalTransformationMatrices[m_numReferenceFrames - 1](0, 1),	generalTransformationMatrices[m_numReferenceFrames - 1](0, 2),
						  generalTransformationMatrices[m_numReferenceFrames - 1](1, 0),	generalTransformationMatrices[m_numReferenceFrames - 1](1, 1),	generalTransformationMatrices[m_numReferenceFrames - 1](1, 2),
						  generalTransformationMatrices[m_numReferenceFrames - 1](2, 0),	generalTransformationMatrices[m_numReferenceFrames - 1](2, 1),	generalTransformationMatrices[m_numReferenceFrames - 1](2, 2);
		
		return mathLib::tipPose( Eigen::Matrix<float, 1, 3>(generalTransformationMatrices[m_numReferenceFrames - 1](0, 3), 
															generalTransformationMatrices[m_numReferenceFrames - 1](1, 3), 
															generalTransformationMatrices[m_numReferenceFrames - 1](2, 3)), 
								rotationMatrix);
	}

	void UR::setTipPose(const mathLib::tipPose& newTipPose)
	{
		m_tipPose = newTipPose;
	}
	
	// << overloading to be able to print a URtype enum
	std::ostream& operator <<(std::ostream& stream, universalRobots::URtype& type)
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
		stream << "x " << robot.getTipPose().m_pos.x() << " y " << robot.getTipPose().m_pos.y() << " z " << robot.getTipPose().m_pos.z() << " (meters)\nalpha "
			<< mathLib::deg(robot.getTipPose().m_rpy.x()) << " beta " << mathLib::deg(robot.getTipPose().m_rpy.y()) << " gamma " << mathLib::deg(robot.getTipPose().m_rpy.z()) << " (degrees)" << std::endl;
		return stream;
	}
}




