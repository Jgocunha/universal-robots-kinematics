# universal-robots-kinematics Wiki

A C++20 library for forward and inverse kinematics of Universal Robots (UR3/UR5/UR10) manipulators, based on the Modified Denavit-Hartenberg (MDH) convention.

## Pages

- **[Getting Started](Getting-Started)** — dependencies and build instructions.
- **[UR Class API](UR-Class-API)** — constructor, `forwardKinematics()`, `inverseKinematics()`, and the other public members.
- **[Modified Denavit-Hartenberg Convention](Modified-Denavit-Hartenberg-Convention)** — the 9-frame MDH parameterisation this library uses.
- **[Kinematics Theory](Kinematics-Theory)** — how forward and inverse kinematics are derived and solved.
- **[Jacobian Theory](Jacobian-Theory)** — the geometric (spatial) Jacobian, manipulability, and the three UR singularity families.
- **[Benchmarking](Benchmarking)** — how to reproduce the timing and accuracy results.
- **[CoppeliaSim Integration](CoppeliaSim-Integration)** — where the CoppeliaSim scenes and remote-API integration now live.

For citations to the papers and textbook behind the kinematics analysis, see [`docs/REFERENCES.md`](https://github.com/Jgocunha/universal-robots-kinematics/blob/main/docs/REFERENCES.md) in the repository.
