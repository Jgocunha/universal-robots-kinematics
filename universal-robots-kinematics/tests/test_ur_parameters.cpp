#include <catch2/catch_test_macros.hpp>

#include "ur.h"

TEST_CASE("UR RobotParameters Initialization", "[RobotParameters]")
{
    using namespace universal_robots_kinematics;

    SECTION("Default Constructor with UR10 and no endEffector")
	{
        RobotParameters params;
        REQUIRE(params.type == UR10);
        REQUIRE(params.endEffector == false);
        REQUIRE(RobotParameters::numDoF == 6);
        REQUIRE(RobotParameters::numTransZ == 7);
        REQUIRE(RobotParameters::numTransX == 4);
        REQUIRE(RobotParameters::numReferenceFrames == 9);
        REQUIRE(RobotParameters::numIkSol == 8);
    }

    SECTION("Constructor with UR3 and endEffector true")
	{
       RobotParameters params(UR3, true);
        REQUIRE(params.type == UR3);
        REQUIRE(params.endEffector == true);
    }

    SECTION("Constructor with UR5 and endEffector false")
	{
        RobotParameters params(UR5, false);
        REQUIRE(params.type == UR5);
		REQUIRE(params.endEffector == false);
    }
}
