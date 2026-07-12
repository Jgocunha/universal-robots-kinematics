# CoppeliaSim Integration

This repository (`universal-robots-kinematics`) is the **C++ kinematics library only**. As of the v2.0.0 restructuring, CoppeliaSim integration is no longer part of this repo — it was removed along with the MATLAB reference implementation.

## Where it moved

The original MATLAB kinematics implementation, the CoppeliaSim scenes/models for UR3/UR5/UR10, and the CoppeliaSim C++ remote-API integration all now live in the archive repository:

**[Jgocunha/universal-robots-kinematics-matlab](https://github.com/Jgocunha/universal-robots-kinematics-matlab)**

That repository includes the CoppeliaSim launch instructions (previously `LAUNCH.md` here) and the scene files for all three UR models.

## Using this library alongside a simulator

If you want to drive a CoppeliaSim (or any other) simulation using this C++ library, the intended pattern is:

1. Use `UR::forwardKinematics()` / `UR::inverseKinematics()` from this library to compute joint values or tip poses.
2. Send those joint values to your simulator through whatever integration layer it exposes (e.g. CoppeliaSim's remote API, ROS, etc.) — that glue code is outside the scope of this library.

This library has no simulator dependency and no CoppeliaSim-specific code; it only performs the kinematics math.
