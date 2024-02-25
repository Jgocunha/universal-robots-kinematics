#include "ur_parameters.h"


namespace universal_robots_kinematics
{
	RobotParameters::RobotParameters(RobotType robotType, bool endEffector)
		: type(robotType), endEffector(endEffector)
	{
		switch (type)
		{
			case UR3:
				linkDimensions.d = ur3LinkDimensions_d;
				linkDimensions.a = ur3LinkDimensions_a;
			break;
			case UR5:
				linkDimensions.d = ur5LinkDimensions_d;
				linkDimensions.a = ur5LinkDimensions_a;
			break;
			case UR10:
				linkDimensions.d = ur10LinkDimensions_d;
				linkDimensions.a = ur10LinkDimensions_a;
			break;
		}
	}
}
