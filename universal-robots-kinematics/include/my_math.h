#pragma once

#include <cmath>
#include <limits>
#include <type_traits>
#include <stdexcept>
#include <numbers>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace my_math
{
    template<typename T>
    T degToRad(T degrees)
    {
        static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating point type.");

        if (std::isnan(degrees))
            throw std::invalid_argument("Input is not a number (NaN).");

        return degrees * static_cast<T>(std::numbers::pi / 180.0);
    }

    template<typename T>
    T radToDeg(T radians)
    {
        static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating point type.");

        if (std::isnan(radians))
            throw std::invalid_argument("Input is not a number (NaN).");

        return radians * static_cast<T>(180.0 / std::numbers::pi);
    }

    struct Pose
	{
        Eigen::Vector3f position = Eigen::Vector3f::Zero(); // x, y, z in meters
        Eigen::Vector3f eulerAngles = Eigen::Vector3f::Zero(); // alpha, beta, gamma in radians

        Pose()
	        : position(0, 0, 0), eulerAngles(0, 0, 0) {}
    	Pose(float pos_x, float pos_y, float pos_z, float angle_alpha, float angle_beta, float angle_gamma)
            : position(pos_x, pos_y, pos_z), eulerAngles(angle_alpha, angle_beta, angle_gamma) {}
    	Pose(Eigen::Vector3f pos, Eigen::Vector3f angles)
            : position(std::move(pos)), eulerAngles(std::move(angles)) {}
    	Pose(Eigen::Vector3f pos, const Eigen::Matrix3f& rotationMatrix)
            : position(std::move(pos)), eulerAngles(rotationMatrix.eulerAngles(0, 1, 2)) {}

    	Pose divideByConst(float constant) const;
        Pose subtract(const Pose& other) const;
        Pose operator/(float constant) const;
        Pose operator-(const Pose& other) const;
    };

    struct Joint
    {
        Pose pose;
        double value;

        Joint(Pose jointPose, const double& jointValue = 0)
            : pose(std::move(jointPose)), value(jointValue) {}
    };

    struct ModifiedDenavitHartenbergParameters
    {
    	double alpha;
		double a;
		double d;
		double theta;

		ModifiedDenavitHartenbergParameters(double alpha, double a, double d, double theta)
			: alpha(alpha), a(a), d(d), theta(theta) {}
    };

    Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& dh_params);
}
