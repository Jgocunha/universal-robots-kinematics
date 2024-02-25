#include "my_math.h"

namespace my_math
{
	Pose Pose::divideByConst(float constant) const
    {
        if (constant == 0) throw std::runtime_error("Division by zero.");
        return Pose{ Eigen::Vector3f{position / constant}, Eigen::Vector3f{eulerAngles / constant} };
    }

    Pose Pose::subtract(const Pose& other) const
    {
        return Pose{ Eigen::Vector3f{position - other.position}, Eigen::Vector3f{eulerAngles - other.eulerAngles } };
    }

    Pose Pose::operator/(float constant) const
    {
        return divideByConst(constant);
    }

    Pose Pose::operator-(const Pose& other) const
    {
        return subtract(other);
    }

    Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& dh_params)
    {
        Eigen::Matrix4f individualTransformationMatrix;
        individualTransformationMatrix << cos(dh_params[3]), -sin(dh_params[3]), 0, dh_params[1],
            (sin(dh_params[3]) * cos(dh_params[0])), (cos(dh_params[3]) * cos(dh_params[0])), -sin(dh_params[0]), (-sin(dh_params[0]) * dh_params[2]),
            (sin(dh_params[3]) * sin(dh_params[0])), (cos(dh_params[3]) * sin(dh_params[0])), cos(dh_params[0]), (cos(dh_params[0]) * dh_params[2]),
            0, 0, 0, 1;

        return individualTransformationMatrix;
    }
}
