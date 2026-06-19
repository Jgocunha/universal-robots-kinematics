const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});
    
    // 1. Create the C++ static library for 'universalRobotsKinematics'
    const lib_urk = b.addLibrary(.{
        .name = "universalRobotsKinematics",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libcpp = true,
        }),
    });
    
    // 2. Add C++ source files
    lib_urk.root_module.addCSourceFiles(.{
        .files = &.{
            "cpp/universalRobotsKinematics/universalRobotsKinematics/src/urkC.cpp", // Our C wrapper implementation
            "cpp/universalRobotsKinematics/universalRobotsKinematics/src/universalRobotsKinematics.cpp",
            "cpp/universalRobotsKinematics/mathLib/src/mathLib.cpp",
        },
        .flags = &.{
            "-std=c++20",
            "-Wno-unused-parameter",
            "-Wno-shadow",
        },
    });
    
    // 3. Add C++ include paths
    lib_urk.addIncludePath(b.path("c/")); // For urkC.h
    lib_urk.addIncludePath(b.path("cpp/universalRobotsKinematics/universalRobotsKinematics/src"));
    lib_urk.addIncludePath(b.path("cpp/universalRobotsKinematics/mathLib/src"));
    lib_urk.addIncludePath(b.path("eigen"));
    
    // 4. Link C standard library
    lib_urk.linkLibC();
    
    // 5. Install the C++ library artifact
    b.installArtifact(lib_urk);
    
    // 6. Create and export the Zig module
    const lib_zurk_module = b.createModule(.{
        .root_source_file = b.path("zig/c.zig"),
        .target = target,
        .optimize = optimize,
    });
    
    lib_zurk_module.addIncludePath(b.path(".")); // For universalRobotsKinematics_C.h
    lib_zurk_module.addIncludePath(b.path("cpp/universalRobotsKinematics/universalRobotsKinematics/src"));
    lib_zurk_module.addIncludePath(b.path("cpp/universalRobotsKinematics/mathLib/src"));
    lib_zurk_module.addIncludePath(b.path("eigen"));
    lib_zurk_module.linkLibrary(lib_urk);
    
    b.modules.put("lib_zurk", lib_zurk_module) catch @panic("failed to register urk module");

    // --- Build an example executable --- 
    const example_exe = b.addExecutable(.{
        .name = "urk_example",
        .root_module = b.createModule(.{
            .root_source_file = b.path("zig/example.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{ 
                .{ .name = "lib_zurk", .module = lib_zurk_module },
            },
        }),
    });

    b.installArtifact(example_exe);

    const run_example_step = b.addRunArtifact(example_exe);
    const run_step = b.step("run", "Run the universalRobotsKinematics example");
    run_step.dependOn(&run_example_step.step);
}
