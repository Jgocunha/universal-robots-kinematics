#include "urkC.h"
#include "universalRobotsKinematics.h"
#include "mathLib.h"
#include <new> // For std::nothrow
#include <cstring> // For memcpy

// --- mathLib.h implementations ---

float mathLib_rad(const float degree) {
    return mathLib::rad(degree);
}

float mathLib_deg(const float rad) {
    return mathLib::deg(rad);
}

void mathLib_calcTransformationMatrix(const float* DHparams, float* out_matrix4x4) {
    Eigen::RowVector4f dh_params_eigen;
    for (int i = 0; i < 4; ++i) {
        dh_params_eigen[i] = DHparams[i];
    }
    Eigen::Matrix4f result_matrix = mathLib::calcTransformationMatrix(dh_params_eigen);
    // Copy the Eigen matrix to the C-style float array
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out_matrix4x4[i * 4 + j] = result_matrix(i, j);
        }
    }
}

// --- universalRobotsKinematics.h implementations ---

UR_C* UR_create(URtype_C robotType, bool endEffector, float endEffectorDimension) {
    universalRobots::URtype cpp_robotType;
    switch (robotType) {
        case UR3: cpp_robotType = universalRobots::UR3; break;
        case UR5: cpp_robotType = universalRobots::UR5; break;
        case UR10: cpp_robotType = universalRobots::UR10; break;
        default: cpp_robotType = universalRobots::UR10; break; // Default to UR10 or handle error
    }
    universalRobots::UR* ur_obj = new (std::nothrow) universalRobots::UR(cpp_robotType, endEffector, endEffectorDimension);
    return static_cast<UR_C*>(ur_obj);
}

void UR_destroy(UR_C* ur_obj) {
    universalRobots::UR* cpp_ur_obj = static_cast<universalRobots::UR*>(ur_obj);
    delete cpp_ur_obj;
}

URtype_C UR_getRobotType(UR_C* ur_obj) {
    universalRobots::UR* cpp_ur_obj = static_cast<universalRobots::UR*>(ur_obj);
    universalRobots::URtype cpp_type = cpp_ur_obj->getRobotType();
    switch (cpp_type) {
        case universalRobots::UR3: return UR3;
        case universalRobots::UR5: return UR5;
        case universalRobots::UR10: return UR10;
        default: return UR10; // Should not happen
    }
}

void UR_forwardKinematics(UR_C* ur_obj, const float* targetJointVal, pose_C* out_pose) {
    universalRobots::UR* cpp_ur_obj = static_cast<universalRobots::UR*>(ur_obj);

    if (targetJointVal == nullptr) {
        if (out_pose != nullptr) {
            for (int i = 0; i < 3; ++i) {
                out_pose->m_pos[i] = 0.0f;
                out_pose->m_eulerAngles[i] = 0.0f;
            }
        }
        return;
    }

    // Create a temporary local array to satisfy the C++ function signature
    float local_targetJointVal[universalRobots::UR::m_numDoF];
    std::memcpy(local_targetJointVal, targetJointVal, universalRobots::UR::m_numDoF * sizeof(float));

    universalRobots::pose cpp_pose = cpp_ur_obj->forwardKinematics(local_targetJointVal);
    
    if (out_pose != nullptr) {
        for (int i = 0; i < 3; ++i) {
            out_pose->m_pos[i] = cpp_pose.m_pos[i];
            out_pose->m_eulerAngles[i] = cpp_pose.m_eulerAngles[i];
        }
    }
}

void UR_inverseKinematics(UR_C* ur_obj, const pose_C* targetTipPose, float* outIkSols) {
    universalRobots::UR* cpp_ur_obj = static_cast<universalRobots::UR*>(ur_obj);
    
    universalRobots::pose cpp_targetTipPose;
    if (targetTipPose != nullptr) {
        for (int i = 0; i < 3; ++i) {
            cpp_targetTipPose.m_pos[i] = targetTipPose->m_pos[i];
            cpp_targetTipPose.m_eulerAngles[i] = targetTipPose->m_eulerAngles[i];
        }
    }

    float cpp_outIkSols[UR_NUM_IK_SOLUTIONS][UR_NUM_DOF];
    cpp_ur_obj->inverseKinematics(cpp_targetTipPose, &cpp_outIkSols);

    // Copy the 2D array to the flat C-style array
    if (outIkSols != nullptr) {
        for (int i = 0; i < UR_NUM_IK_SOLUTIONS; ++i) {
            for (int j = 0; j < UR_NUM_DOF; ++j) {
                outIkSols[i * UR_NUM_DOF + j] = cpp_outIkSols[i][j];
            }
        }
    }
}

pose_C UR_generateRandomReachablePose(UR_C* ur_obj) {
    universalRobots::UR* cpp_ur_obj = static_cast<universalRobots::UR*>(ur_obj);
    universalRobots::pose cpp_pose = cpp_ur_obj->generateRandomReachablePose();
    
    pose_C c_pose;
    for (int i = 0; i < 3; ++i) {
        c_pose.m_pos[i] = cpp_pose.m_pos[i];
        c_pose.m_eulerAngles[i] = cpp_pose.m_eulerAngles[i];
    }
    return c_pose;
}

bool UR_checkPoseReachability(UR_C* ur_obj, const float* ikSol) {
    universalRobots::UR* cpp_ur_obj = static_cast<universalRobots::UR*>(ur_obj);

    if (ikSol == nullptr) {
        return false;
    }

    // Create a temporary local array to satisfy the C++ function signature
    float local_ikSol[universalRobots::UR::m_numDoF];
    std::memcpy(local_ikSol, ikSol, universalRobots::UR::m_numDoF * sizeof(float));
    
    return cpp_ur_obj->checkPoseReachability(local_ikSol);
}
