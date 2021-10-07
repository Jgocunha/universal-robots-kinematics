#include <iostream>
#include <string>
#include <numbers>
#include <math.h>


// TO DO:
// 1. getMDHmatrix()
// 2. setMDHmatrix() without temp. matrix
// 3. calcTransformationMatrix without temp. matrix

using namespace std;

//const double PI = std::atan(1.0) * 4;

enum URtype: int
{
	UR3, UR5, UR10 // 0, 1, 2
};

float rad(const float& degree)
{
	 return (degree * std::numbers::pi_v<float> / 180);// C++20
	//return (degree * PI / 180); //non-compiler dependant
}

float deg(const float& rad)
{
	return (rad * 180 / std::numbers::pi_v<float>);// C++20
   //return (rad * 180 / PI); //non-compiler dependant
}

struct RPY
{
	float m_alpha = 0, m_beta = 0, m_gamma = 0;

	RPY(const float& alpha, const float& beta, const float& gamma)
		:m_alpha(alpha), m_beta(beta), m_gamma(gamma) {}

	RPY calcRPYangles(const float (&rotationMatrix)[3][3])
	{

	}
};

struct position
{
	float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct tipPose
{
	position m_pos_e;
	RPY m_rpy_e;

	tipPose() 
		: m_pos_e({0.0f, 0.0f, 0.0f}), m_rpy_e({0.0f, 0.0f, 0.0f}) {}

	tipPose(const position& pos, const RPY& rpy)
		:m_pos_e(pos), m_rpy_e(rpy) {}
};

struct TransformationMatrix
{
	float m_matrix[4][4] = {{0, 0, 0, 0},
							{0, 0, 0, 0}, 
							{0, 0, 0, 0}, 
							{0, 0, 0, 0}};

	TransformationMatrix()
		:m_matrix{	{0, 0, 0, 0},
					{0, 0, 0, 0},
					{0, 0, 0, 0},
					{0, 0, 0, 0}} {}

	TransformationMatrix(const float (&matrix)[4][4])
		:m_matrix{**matrix } {}

	friend TransformationMatrix** operator*(const TransformationMatrix& mat1, const TransformationMatrix& mat2);

};


TransformationMatrix** operator*(const TransformationMatrix& mat1, const TransformationMatrix& mat2)
{
	float result[4][4] = {};

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				result[i][j] += mat1.m_matrix[i][k] * mat2.m_matrix[k][j];
			}
		}
	}

	return reinterpret_cast<TransformationMatrix**>(result);
}

// Computes the transformation matrix from a system of coordinates to another
float** calcTransformationMatrix(const float(&DHparams)[])
{
	float individualTransformationMatrix[4][4] = { {cos(DHparams[3]),								-sin(DHparams[3]),							0,							DHparams[1]},
													{(sin(DHparams[3]) * cos(DHparams[0])),		(cos(DHparams[3]) * cos(DHparams[0])),	    -sin(DHparams[0]),	    (-sin(DHparams[0]) * DHparams[2])},
													{(sin(DHparams[3]) * sin(DHparams[0])),		(cos(DHparams[3]) * sin(DHparams[0])),	    cos(DHparams[0]),		(cos(DHparams[0]) * DHparams[2])},
													{0,														0,										0,							1} };
	return reinterpret_cast<float**>(individualTransformationMatrix);
}


class UR
{

public:
	static constexpr unsigned int m_numDoF = 6; // number of DoFs is always 6
	static constexpr unsigned int m_numTransZ = 7; // number of translations in the z-axis is always 7
	static constexpr unsigned int m_numTransX = 4; // number of translations in the z-axis is always 4
	static constexpr unsigned int m_numReferenceFrames = 9; // number of frames is pre-defined

private:
	URtype m_type = UR10; // UR 3, 5, 10
	float m_d[m_numTransZ] = {0.109f, 0.10122f, 0.12067f - 0.10122f, 0.11406f - 0.12067f, 0.17246f - 0.11406f , 0.26612f - 0.1724f, 0.36474f - 0.26612f }; // d - translation (meters) in the z-axis
	float m_a[m_numTransX] = { 0.7211f - 0.109f, 1.2933f - 0.7211f, 1.3506f - 1.2933f, 1.409f - 1.3506f }; // a - translation (meters) in the x-axis
	float m_theta[m_numDoF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // joint values in rads

public:
		float m_MDHmatrix[m_numReferenceFrames][4] = {  {0,			0,			m_d[0],			m_theta[0]},				// 0T1
														{rad(-90),	0,			m_d[1],			m_theta[1] + rad(-90)},	// 1T2
														{0,			m_a[0],		m_d[2],			m_theta[2]},				// 2T3
														{0,			m_a[1],		m_d[3],			m_theta[3]},				// 3T4
														{0,			m_a[2],		m_d[4],			rad(90)},				// 4T4'
														{rad(90),	0,			0,				m_theta[4]},				// 4'T5
														{rad(-90),	0,			0,				rad(-90)},				// 5T5'
														{0,			m_a[3],		m_d[5],			m_theta[5]},				// 5'T6
														{0,			0,			m_d[6],			0} };					// 6T7
public:
	const URtype getRobotType(void) const
	{
		return m_type;
	}
	void setRobotType(const URtype& type)
	{
		m_type = type;
	}
	const float* getTransZ(void) const
	{
		return m_d;
	}
	void setTransZ(const float (&d)[])
	{
		memcpy(m_d, d, sizeof(m_d));
	}
	const float* getTransX(void) const
	{
		return m_a;
	}
	void setTransX(const float (&a)[])
	{
		memcpy(m_a, a, sizeof(m_a));
	}
	//const float** getMDHmatrix(void) const
	//{
	//	return reinterpret_cast<const float**>(MDHmatrix);
	//}

	const float* getTheta(void) const
	{
		return m_theta;
	}
	tipPose forwardKinematics(const float(&jointVal)[]);
private:
	void setTheta(const float(&jointVal)[])
	{
		memcpy(m_theta, jointVal, sizeof(m_theta));
	}
	void setMDHmatrix(void)
	{
		float tempMat[m_numReferenceFrames][4]= {{0,		0,			m_d[0],			m_theta[0]},				// 0T1
												{ -rad(90),	0,			m_d[1],			m_theta[1] - rad(90) },	// 1T2
												{ 0,		m_a[0],		m_d[2],			m_theta[2] },				// 2T3
												{ 0,		m_a[1],		m_d[3],			m_theta[3] },				// 3T4
												{ 0,		m_a[2],		m_d[4],			rad(90) },				// 4T4'
												{ rad(90),	0,			0,				m_theta[4] },				// 4'T5
												{ -rad(90),	0,			0,				-rad(90) },				// 5T5'
												{ 0,		m_a[3],		m_d[5],			m_theta[5] },				// 5'T6
												{ 0,		0,			m_d[6],			0 } };					// 6T7

		memcpy(m_MDHmatrix, tempMat, sizeof(m_MDHmatrix));

	}
};

tipPose UR::forwardKinematics(const float(&targetJointVal)[])
{
	// assign joint values to compute MDH matrix;
	setTheta(targetJointVal);
	setMDHmatrix();

	// create an array of transformation matrices
	TransformationMatrix individualTransformationMatrices[m_numReferenceFrames] = {};
	// determine the indiviual transformation matrices
	for (unsigned int i = 0; i < m_numReferenceFrames; i++)
	{
		memcpy(individualTransformationMatrices[i].m_matrix, calcTransformationMatrix(m_MDHmatrix[i]), sizeof(float[4][4]));
		//memcpy(individualTransformationMatrices[i].matrix, MDHMatrix(getMDHmatrix()[i]), sizeof(float[4][4]));
	}
	// create another array of transformation matrices
	TransformationMatrix generalTransformationMatrices[m_numReferenceFrames] = {};
	// determine the general transformation matrix
	memcpy(generalTransformationMatrices[0].m_matrix, individualTransformationMatrices[0].m_matrix, sizeof(float[4][4]));
	for (unsigned int i = 1; i < m_numReferenceFrames; i++)
	{
		memcpy(generalTransformationMatrices[i].m_matrix, (generalTransformationMatrices[i-1]*individualTransformationMatrices[i]), sizeof(float[4][4]) );
	}

	float rotationMatrix[3][3] = {  { generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[0][0], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[0][1], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[0][2] },
									{ generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[1][0], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[1][1], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[1][2] },
									{ generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[2][0], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[2][1], generalTransformationMatrices[m_numReferenceFrames - 1].m_matrix[2][2] } };
		
		
		
		//generalTransformationMatrices[m_numReferenceFrames].m_matrix

	//tipPose tipPose( ,RPY::calcRPYangles(rotationMatrix));
	tipPose tipPose;
	return tipPose;
}

// << overloading to be able to print a URtype enum
ostream& operator <<(std::ostream& stream, URtype type)
{
	switch (type)
	{
	case UR3: 
		stream << "UR3";
		break;
	case UR5: 
		stream << "UR5";
		break;
	case UR10: 
		stream << "UR10";
		break;
	default: 
		stream.setstate(std::ios_base::failbit);
	}
	return stream;
}

// << overloading to be able to print a UR
ostream& operator <<(ostream& stream, const UR& robot)
{
	stream << "Robot type: " << robot.getRobotType() << endl
		<< "Number of DoFs: " << robot.m_numDoF << endl
		<< "Link dimensions\n" << "Translations in the z-axis (meters)\n";
	for (unsigned int i = 0; i < robot.m_numTransZ; i++)
		stream << "d" << i + 1 << ": " << robot.getTransZ()[i] << endl;
	stream << "Translations in the x-axis (meters)\n";
	for (unsigned int i = 0; i < robot.m_numTransX; i++)
		stream << "a" << i + 2 << ": " << robot.getTransX()[i] << endl;
	stream << "Modified Denavit-Hartengerg Matrix\n";
	for (int x = 0; x < robot.m_numReferenceFrames; x++)  // loop lines
	{
		for (int y = 0; y < 4; y++)  // loop columns
		{
			//stream << robot.getMDHmatrix()[x][y];
			stream << (robot.m_MDHmatrix[x][y]) << "				";
		}
		stream << endl;  
	}
	stream << "Joint values (deg)\n";
	for (unsigned int i = 0; i < robot.m_numDoF; i++)
		stream << "Theta" << i + 1 << ": " << deg(robot.getTheta()[i]) << endl;
	return stream;
}

int main()
{
	UR robot;

	cout << robot;


	//float jointValues[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	float jointValues[6] = { rad(90), rad(45), rad(90), rad(45), rad(45), rad(90) };


	// give joint states, receive end-effector pose
	tipPose tipPose = robot.forwardKinematics(jointValues);
	


	cin.get();
}