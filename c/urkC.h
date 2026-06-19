#ifndef UNIVERSAL_ROBOTS_KINEMATICS_C_H
#define UNIVERSAL_ROBOTS_KINEMATICS_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h> // For bool type in C

// --- Constants from universalRobotsKinematics.h ---
#define UR_NUM_DOF 6
#define UR_NUM_TRANS_Z 7
#define UR_NUM_TRANS_X 4
#define UR_NUM_REFERENCE_FRAMES 9
#define UR_NUM_IK_SOLUTIONS 8

// --- From mathLib.h ---
#define MATHLIB_PI 3.14159265358979323846f

float mathLib_rad(const float degree);
float mathLib_deg(const float rad);

// For calcTransformationMatrix, we'll need to pass raw float arrays
// DHparams is a 4-element array, result is a 4x4 matrix (16 elements)
void mathLib_calcTransformationMatrix(const float* DHparams, float* out_matrix4x4);

// --- From universalRobotsKinematics.h ---

typedef enum {
    UR3,
    UR5,
    UR10
} URtype_C;

typedef struct {
    float m_pos[3];         // x y z (meters)
    float m_eulerAngles[3]; // alpha beta gamma (radians)
} pose_C;

typedef struct {
    pose_C m_jointPose;
    float m_jointValue;
} joint_C;

// Opaque pointer for the C++ UR class
typedef void UR_C;

// Constructor
UR_C* UR_create(URtype_C robotType, bool endEffector, float endEffectorDimension);

// Destructor
void UR_destroy(UR_C* ur_obj);

// Methods
URtype_C UR_getRobotType(UR_C* ur_obj);
void UR_forwardKinematics(UR_C* ur_obj, const float* targetJointVal, pose_C* out_pose);
void UR_inverseKinematics(UR_C* ur_obj, const pose_C* targetTipPose, float* outIkSols); // outIkSols is a float[UR_NUM_IK_SOLUTIONS * UR_NUM_DOF]
pose_C UR_generateRandomReachablePose(UR_C* ur_obj);
bool UR_checkPoseReachability(UR_C* ur_obj, const float* ikSol);

#ifdef __cplusplus
}
#endif

#endif // UNIVERSAL_ROBOTS_KINEMATICS_C_H
