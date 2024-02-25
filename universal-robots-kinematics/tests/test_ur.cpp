#include <catch2/catch_test_macros.hpp>

#include "ur.h"

TEST_CASE("UR Class Constructor and getParameters Method", "[UR]")
{
    using namespace universal_robots_kinematics;

    SECTION("Constructing UR with default RobotParameters")
	{
        RobotParameters params;
        UR urRobot(params);
        auto returnedParams = urRobot.getParameters();

        REQUIRE(returnedParams.type == UR10);
        REQUIRE(returnedParams.endEffector == false);
        REQUIRE(RobotParameters::numDoF == 6);
        REQUIRE(RobotParameters::numTransZ == 7);
        REQUIRE(RobotParameters::numTransX == 4);
        REQUIRE(RobotParameters::numReferenceFrames == 9);
        REQUIRE(RobotParameters::numIkSol == 8);
    }

    SECTION("Constructing UR with UR3 and endEffector true")
	{
        RobotParameters customParams(UR3, true);
        UR urRobot(customParams);
        auto returnedParams = urRobot.getParameters();

        REQUIRE(returnedParams.type == UR3);
        REQUIRE(returnedParams.endEffector == true);
    }

    SECTION("Constructing UR with UR5 and endEffector false")
    {
        RobotParameters customParams(UR5, false);
        UR urRobot(customParams);
        auto returnedParams = urRobot.getParameters();

        REQUIRE(returnedParams.type == UR5);
        REQUIRE(returnedParams.endEffector == false);
    }

    SECTION("Updating UR parameters with setParameters and verifying with getParameters")
	{
        RobotParameters initialParams(UR3, true); // Start with one set of parameters
        UR urRobot(initialParams);

        // Verify initial setup
        auto returnedParams = urRobot.getParameters();
        REQUIRE(returnedParams.type == UR3);
        REQUIRE(returnedParams.endEffector == true);

        // Update the parameters
        RobotParameters newParams(UR10, false); // Change to another set of parameters
        urRobot.setParameters(newParams);

        // Verify the update
        returnedParams = urRobot.getParameters();
        REQUIRE(returnedParams.type == UR10);
        REQUIRE(returnedParams.endEffector == false);
    }

    SECTION("Set and Get parameters for each UR type")
	{
        UR urRobot{ RobotParameters() }; // Initialize with default parameters

        // Test updating to UR3
        urRobot.setParameters(RobotParameters(UR3, false));
        auto params = urRobot.getParameters();
        REQUIRE(params.type == UR3);
        REQUIRE(params.endEffector == false);

        // Test updating to UR5
        urRobot.setParameters(RobotParameters(UR5, true)); 
        params = urRobot.getParameters();
        REQUIRE(params.type == UR5);
        REQUIRE(params.endEffector == true);

        // Test updating to UR10
        urRobot.setParameters(RobotParameters(UR10, false));
        params = urRobot.getParameters();
        REQUIRE(params.type == UR10);
        REQUIRE(params.endEffector == false);
    }
}