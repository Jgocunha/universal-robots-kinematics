#include <iostream>
#include <string>
#include <numbers>
#include <math.h>

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
	float alpha = 0, beta = 0, gamma = 0;
};

struct position
{
	float x = 0, y = 0, z = 0;
};

struct tipPose
{
	position pos_e;
	RPY rpy_e;
};

struct TransformationMatrix
{
	float matrix[4][4] = {  {0, 0, 0, 0},
							{0, 0, 0, 0}, 
							{0, 0, 0, 0}, 
							{0, 0, 0, 0} };

	TransformationMatrix()
		:matrix{	{0, 0, 0, 0},
					{0, 0, 0, 0},
					{0, 0, 0, 0},
					{0, 0, 0, 0}} {}

	TransformationMatrix(const float (&in)[4][4])
		:matrix{**in} {}


	//TransformationMatrix** operator*(const TransformationMatrix& mat1, const TransformationMatrix& mat2)
	//{
	//	float result[sizeof(mat1.matrix) / sizeof(mat1.matrix[0])][sizeof(mat2.matrix[0]) / sizeof(mat2.matrix)] = {};

	//	for (int i = 0; i < sizeof(mat1.matrix) / sizeof(mat1.matrix[0]); i++) {
	//		for (int j = 0; j < sizeof(mat2.matrix[0]) / sizeof(mat2.matrix); j++) {
	//			result[i][j] = 0;
	//			for (int k = 0; k < sizeof(mat2.matrix) / sizeof(mat2.matrix[0]); k++) {
	//				result[i][j] += mat1.matrix[i][k] * mat2.matrix[k][j];
	//			}
	//		}
	//	}

	//	return reinterpret_cast<float**>(result);
	//}

};


float** MDHMatrix(const float(&DHmatrixLine)[])
{
	float individualTransformationMatrix[4][4] = { {cos(DHmatrixLine[3]),								-sin(DHmatrixLine[3]),							0,							DHmatrixLine[1]},
													{(sin(DHmatrixLine[3]) * cos(DHmatrixLine[0])),		(cos(DHmatrixLine[3]) * cos(DHmatrixLine[0])),	    -sin(DHmatrixLine[0]),	    (-sin(DHmatrixLine[0]) * DHmatrixLine[2])},
													{(sin(DHmatrixLine[3]) * sin(DHmatrixLine[0])),		(cos(DHmatrixLine[3]) * sin(DHmatrixLine[0])),	    cos(DHmatrixLine[0]),		(cos(DHmatrixLine[0]) * DHmatrixLine[2])},
													{0,														0,												    0,							1} };
	return reinterpret_cast<float**>(individualTransformationMatrix);
}


class UR
{

public:
	static constexpr unsigned int numDoF = 6; // number of DoFs is always 6
	static constexpr unsigned int numTransZ = 7; // number of translations in the z-axis is always 7
	static constexpr unsigned int numTransX = 4; // number of translations in the z-axis is always 4
	static constexpr unsigned int numReferenceFrames = 9; // number of frames is pre-defined

private:
	URtype type = UR10; // UR 3, 5, 10
	float d[numTransZ] = {0.109f, 0.10122f, 0.12067f - 0.10122f, 0.11406f - 0.12067f, 0.17246f - 0.11406f , 0.26612f - 0.1724f, 0.36474f - 0.26612f }; // d - translation (meters) in the z-axis
	float a[numTransX] = { 0.7211f - 0.109f, 1.2933f - 0.7211f, 1.3506f - 1.2933f, 1.409f - 1.3506f }; // a - translation (meters) in the x-axis
	float theta[numDoF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // joint values in rads
public:
		float MDHmatrix[numReferenceFrames][4] = {  {0,			0,			d[0],			theta[0]},				// 0T1
												{rad(-90),	0,			d[1],			theta[1] + rad(-90)},	// 1T2
												{0,			a[0],		d[2],			theta[2]},				// 2T3
												{0,			a[1],		d[3],			theta[3]},				// 3T4
												{0,			a[2],		d[4],			rad(90)},				// 4T4'
												{rad(90),	0,			0,				theta[4]},				// 4'T5
												{rad(-90),	0,			0,				rad(-90)},				// 5T5'
												{0,			a[3],		d[5],			theta[5]},				// 5'T6
												{0,			0,			d[6],			0} };					// 6T7
public:
	const URtype getRobotType(void) const
	{
		return type;
	}
	void setRobotType(const URtype& type)
	{
		this->type = type;
	}
	const float* getTransZ(void) const
	{
		return d;
	}
	void setTransZ(const float (&d)[])
	{
		//this->d = d;
		memcpy(this->d, d, sizeof(this->d));
	}
	const float* getTransX(void) const
	{
		return a;
	}
	void setTransX(const float (&a)[])
	{
		//this->a = (*a);
		memcpy(this->a, a, sizeof(this->a));
	}
	//const float** getMDHmatrix(void) const
	//{
	//	return reinterpret_cast<const float**>(MDHmatrix);
	//}
	const float* getTheta(void) const
	{
		return theta;
	}
	tipPose forwardKinematics(const float(&jointVal)[]);
};

tipPose UR::forwardKinematics(const float(&jointVal)[])
{
	tipPose tipPose;
	// create an array of transformation matrices
	TransformationMatrix individualTransformationMatrices[numReferenceFrames] = {};
	// determine the indiviual transformation matrices
	for (unsigned int i = 0; i < numReferenceFrames; i++)
	{
		memcpy(individualTransformationMatrices[i].matrix, MDHMatrix(MDHmatrix[i]), sizeof(float[4][4]));
		//memcpy(individualTransformationMatrices[i].matrix, MDHMatrix(getMDHmatrix()[i]), sizeof(float[4][4]));
	}
	// create another array of transformation matrices
	TransformationMatrix generalTransformationMatrices[numReferenceFrames] = {};
	// determine the general transformation matrix
	memcpy(generalTransformationMatrices[0].matrix, individualTransformationMatrices[0].matrix, sizeof(float[4][4]));
	for (unsigned int i = 1; i < numReferenceFrames; i++)
	{

		//memcpy(generalTransformationMatrices[i].matrix, (generalTransformationMatrices[i-1].matrix*individualTransformationMatrices[i].matrix), sizeof(float[4][4]) );
	}

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
		<< "Number of DoFs: " << robot.numDoF << endl
		<< "Link dimensions\n" << "Translations in the z-axis (meters)\n";
	for (unsigned int i = 0; i < robot.numTransZ; i++)
		stream << "d" << i + 1 << ": " << robot.getTransZ()[i] << endl;
	stream << "Translations in the x-axis (meters)\n";
	for (unsigned int i = 0; i < robot.numTransX; i++)
		stream << "a" << i + 2 << ": " << robot.getTransX()[i] << endl;
	stream << "Modified Denavit-Hartengerg Matrix\n";
	for (int x = 0; x < robot.numReferenceFrames; x++)  // loop lines
	{
		for (int y = 0; y < 4; y++)  // loop columns
		{
			//stream << robot.getMDHmatrix()[x][y];
			stream << (robot.MDHmatrix[x][y]) << "				";
		}
		stream << endl;  
	}
	stream << "Joint values (deg)\n";
	for (unsigned int i = 0; i < robot.numDoF; i++)
		stream << "Theta" << i + 1 << ": " << deg(robot.getTheta()[i]) << endl;
	return stream;
}



int main()
{
	UR robot;

	cout << robot;


	float jointValues[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	// give joint states, receive end-effector pose
	tipPose tipPose = robot.forwardKinematics(jointValues);
	


	cin.get();
}