// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include <iostream>
#include <vector>

#include "my_math.h"
#include "ur.h"

int main(int argc, char* argv[])
{
	using namespace universal_robots_kinematics;
    using namespace my_math;


    // Create a robot object
    UR robot(RobotParameters(RobotType::UR5, false));

    // Test different target joint values
    const std::vector<double> targetJointValues = { degToRad(23.0), degToRad(345.0), degToRad(78.0), degToRad(66.0), degToRad(77.0), degToRad(12.0) };

    // Compute forward kinematics
    const Pose tipPose = robot.forwardKinematics(targetJointValues);

    std::cout << "Tip Pose: " << tipPose.position << std::endl;

    return 0;	
}
