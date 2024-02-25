#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <limits>

#include "my_math.h"


TEST_CASE("Degrees to Radians Conversion", "[degToRad]")
{
    using namespace my_math;

    SECTION("Normal conversion")
	{
        REQUIRE(degToRad(180.0) == Catch::Approx(std::numbers::pi));
        REQUIRE(degToRad(90.0) == Catch::Approx(std::numbers::pi / 2));
    }

    SECTION("NaN input throws exception")
	{
        REQUIRE_THROWS_AS(degToRad(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    }
}

TEST_CASE("Radians to Degrees Conversion", "[radToDeg]")
{
    using namespace my_math;

    SECTION("Normal conversion")
	{
        REQUIRE(radToDeg(std::numbers::pi) == Catch::Approx(180.0));
        REQUIRE(radToDeg(std::numbers::pi / 2) == Catch::Approx(90.0));
    }

    SECTION("NaN input throws exception")
	{
        REQUIRE_THROWS_AS(radToDeg(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    }
}

TEST_CASE("Pose Construction", "[Pose]")
{
    using namespace my_math;

    SECTION("Default Constructor")
	{
        Pose pose;
        REQUIRE(pose.position == Eigen::Vector3f::Zero());
        REQUIRE(pose.eulerAngles == Eigen::Vector3f::Zero());
    }

    SECTION("Constructor with individual components")
	{
        Pose pose(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
        REQUIRE(pose.position == Eigen::Vector3f(1.0f, 2.0f, 3.0f));
        REQUIRE(pose.eulerAngles == Eigen::Vector3f(0.1f, 0.2f, 0.3f));
    }

    SECTION("Constructor with Eigen::Vector3f for position and orientation")
	{
        Eigen::Vector3f position(4.0f, 5.0f, 6.0f);
        Eigen::Vector3f angles(0.4f, 0.5f, 0.6f);
        Pose pose(position, angles);
        REQUIRE(pose.position == position);
        REQUIRE(pose.eulerAngles == angles);
    }

    SECTION("Constructor from position vector and rotation matrix")
	{
        Eigen::Vector3f position(7.0f, 8.0f, 9.0f);
        Eigen::Matrix3f rotationMatrix;
        rotationMatrix = Eigen::AngleAxisf(0.7f, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(0.9f, Eigen::Vector3f::UnitZ());
        Pose pose(position, rotationMatrix);
        REQUIRE(pose.position == position);
        // Allow for a small error margin due to floating-point arithmetic
        REQUIRE(pose.eulerAngles.isApprox(rotationMatrix.eulerAngles(0, 1, 2), 1e-6f));
    }
}

TEST_CASE("Pose Operations", "[Pose]")
{
    using namespace my_math;

    SECTION("divideByConst")
	{
        Pose pose(2.0f, 4.0f, 6.0f, 0.1f, 0.2f, 0.3f);
        float divisor = 100.0f;
        Pose result = pose / divisor;

        // Checking position
        REQUIRE(result.position.x() == Catch::Approx(2.0f / divisor));
        REQUIRE(result.position.y() == Catch::Approx(4.0f / divisor));
        REQUIRE(result.position.z() == Catch::Approx(6.0f / divisor));

        // Checking eulerAngles
        REQUIRE(result.eulerAngles.x() == Catch::Approx(0.1f / divisor).epsilon(0.001));
        REQUIRE(result.eulerAngles.y() == Catch::Approx(0.2f / divisor).epsilon(0.001));
        REQUIRE(result.eulerAngles.z() == Catch::Approx(0.3f / divisor).epsilon(0.001));

        SECTION("Division by zero throws runtime_error")
    	{
            REQUIRE_THROWS_AS(pose.divideByConst(0.0f), std::runtime_error);
        }
    }

    SECTION("subtract")
	{
        Pose pose1(10.0f, 20.0f, 30.0f, 0.1f, 0.2f, 0.3f);
        Pose pose2(1.0f, 2.0f, 3.0f, 0.01f, 0.02f, 0.03f);
        Pose result = pose1 - pose2;
        REQUIRE(result.position == Eigen::Vector3f(9.0f, 18.0f, 27.0f));
        REQUIRE(result.eulerAngles == Eigen::Vector3f(0.09f, 0.18f, 0.27f));
    }
}