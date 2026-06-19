const std = @import("std");
const c = @import("lib_zurk").c; // Import our C bindings

pub fn main() !void {
    const stdout = std.fs.File.stdout().deprecatedWriter();

    // Create a UR robot object (e.g., UR5)
    // Explicitly cast the integer value for UR5 to URtype_C
    const ur_robot = c.UR_create(@as(c.URtype_C, 1), false, 0.0); // UR5, no end effector, 0.0 dimension
    if (ur_robot == null) {
        try stdout.print("Failed to create UR robot object.\n", .{});
        return error.FailedToCreateURRobot;
    }

    defer c.UR_destroy(ur_robot); // Ensure the robot object is destroyed when we exit

    // Get robot type
    const robot_type = c.UR_getRobotType(ur_robot);
    switch (robot_type) {
        @as(c.URtype_C, 0) => try stdout.print("Robot Type: UR3\n", .{}),
        @as(c.URtype_C, 1) => try stdout.print("Robot Type: UR5\n", .{}),
        @as(c.URtype_C, 2) => try stdout.print("Robot Type: UR10\n", .{}),
        else => try stdout.print("Robot Type: Unknown\n", .{}),
    }

    // Example: Forward Kinematics
    // Joint values (radians) for a 6-DOF robot
    var joint_values: [c.UR_NUM_DOF]f32 = .{ 0.0, -std.math.pi / 2.0, 0.0, -std.math.pi / 2.0, 0.0, 0.0 };
    var tip_pose: c.pose_C = undefined; // Output pose

    c.UR_forwardKinematics(ur_robot, &joint_values[0], &tip_pose);

    try stdout.print("\nForward Kinematics Result:\n", .{});
    try stdout.print("  Position (x, y, z): ({d:.4}, {d:.4}, {d:.4})\n", .{ tip_pose.m_pos[0], tip_pose.m_pos[1], tip_pose.m_pos[2] });
    try stdout.print("  Euler Angles (alpha, beta, gamma): ({d:.4}, {d:.4}, {d:.4})\n", .{ tip_pose.m_eulerAngles[0], tip_pose.m_eulerAngles[1], tip_pose.m_eulerAngles[2] });

    // Example: mathLib functions
    const degree_val: f32 = 90.0;
    const rad_val = c.mathLib_rad(degree_val);
    const deg_val = c.mathLib_deg(rad_val);

    try stdout.print("\nmathLib Examples:\n", .{});
    try stdout.print("  {d} degrees is {d:.4} radians\n", .{ degree_val, rad_val });
    try stdout.print("  {d:.4} radians is {d:.4} degrees\n", .{ rad_val, deg_val });

    // Example: Inverse Kinematics (simplified, just showing the call)
    // In a real scenario, you'd use the tip_pose from FK or a desired pose
    var ik_solutions: [c.UR_NUM_IK_SOLUTIONS * c.UR_NUM_DOF]f32 = undefined; // Flat array for solutions
    c.UR_inverseKinematics(ur_robot, &tip_pose, &ik_solutions[0]);

    try stdout.print("\nInverse Kinematics called (results not printed for brevity).\n", .{});

    // Example: Generate Random Reachable Pose
    const random_pose = c.UR_generateRandomReachablePose(ur_robot);
    try stdout.print("\nRandom Reachable Pose:\n", .{});
    try stdout.print("  Position (x, y, z): ({d:.4}, {d:.4}, {d:.4})\n", .{ random_pose.m_pos[0], random_pose.m_pos[1], random_pose.m_pos[2] });
    try stdout.print("  Euler Angles (alpha, beta, gamma): ({d:.4}, {d:.4}, {d:.4})\n", .{ random_pose.m_eulerAngles[0], random_pose.m_eulerAngles[1], random_pose.m_eulerAngles[2] });

    // Example: Check Pose Reachability (using the first IK solution as an example)
    if (c.UR_checkPoseReachability(ur_robot, &ik_solutions[0])) {
        try stdout.print("\nFirst IK solution is reachable.\n", .{});
    } else {
        try stdout.print("\nFirst IK solution is NOT reachable.\n", .{});
    }
}
