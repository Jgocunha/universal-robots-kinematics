const std = @import("std");
const fs = std.fs;
const process = std.process;

const SUBMODULE_URL = "https://gitlab.com/libeigen/eigen.git";
const SUBMODULE_TAG = "3.4.0";  // Changed from BRANCH to TAG
const SUBMODULE_PATH = "cpp/eigen";
const VENDOR_PATH = "eigen";

// Whitelist of directories/files to copy
const ALLOWED_DIRS = [_][]const u8{
    "Eigen",
    "COPYING.MPL2",
    "COPYING.README",
};

fn runCommand(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var child = process.Child.init(args, allocator);
    child.stdout_behavior = .Inherit;
    child.stderr_behavior = .Inherit;
    const term = try child.spawnAndWait();

    switch (term) {
        .Exited => |code| if (code != 0) return error.CommandFailed,
        else => return error.CommandFailed,
    }
}

fn isAllowed(name: []const u8) bool {
    for (ALLOWED_DIRS) |allowed| {
        if (std.mem.eql(u8, name, allowed)) return true;
    }
    return false;
}

fn copyDirRecursive(allocator: std.mem.Allocator, src_path: []const u8, dest_path: []const u8) !void {
    try fs.cwd().makePath(dest_path);

    var dir = try fs.cwd().openDir(src_path, .{ .iterate = true });
    defer dir.close();

    var it = dir.iterate();
    while (try it.next()) |entry| {
        if (std.mem.eql(u8, entry.name, ".git")) continue;

        const entry_src_path = try fs.path.join(allocator, &[_][]const u8{ src_path, entry.name });
        defer allocator.free(entry_src_path);

        const entry_dest_path = try fs.path.join(allocator, &[_][]const u8{ dest_path, entry.name });
        defer allocator.free(entry_dest_path);

        switch (entry.kind) {
            .directory => try copyDirRecursive(allocator, entry_src_path, entry_dest_path),
            .file => {
                if (fs.path.dirname(entry_dest_path)) |dir_path| {
                    try fs.cwd().makePath(dir_path);
                }
                try fs.cwd().copyFile(entry_src_path, fs.cwd(), entry_dest_path, .{});
            },
            else => {},
        }
    }
}

fn copySelectedDirs(allocator: std.mem.Allocator, src_path: []const u8, dest_path: []const u8) !void {
    try fs.cwd().makePath(dest_path);

    var dir = try fs.cwd().openDir(src_path, .{ .iterate = true });
    defer dir.close();

    var it = dir.iterate();
    while (try it.next()) |entry| {
        if (!isAllowed(entry.name)) continue;

        const entry_src_path = try fs.path.join(allocator, &[_][]const u8{ src_path, entry.name });
        defer allocator.free(entry_src_path);

        const entry_dest_path = try fs.path.join(allocator, &[_][]const u8{ dest_path, entry.name });
        defer allocator.free(entry_dest_path);

        switch (entry.kind) {
            .directory => try copyDirRecursive(allocator, entry_src_path, entry_dest_path),
            .file => {
                if (fs.path.dirname(entry_dest_path)) |dir_path| {
                    try fs.cwd().makePath(dir_path);
                }
                try fs.cwd().copyFile(entry_src_path, fs.cwd(), entry_dest_path, .{});
            },
            else => {},
        }
    }
}

fn isSubmoduleTracked(allocator: std.mem.Allocator, path: []const u8) !bool {
    const status_args = &[_][]const u8{ "git", "submodule", "status", path };
    var status_child = process.Child.init(status_args, allocator);
    status_child.stdout_behavior = .Ignore;
    status_child.stderr_behavior = .Ignore;
    const status_term = try status_child.spawnAndWait();

    return switch (status_term) {
        .Exited => |code| code == 0,
        else => false,
    };
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    std.debug.print("Checking submodule...\n", .{});
    if (!try isSubmoduleTracked(allocator, SUBMODULE_PATH)) {
        std.debug.print("Adding submodule: {s}\n", .{SUBMODULE_PATH});
        // Add submodule without specifying branch
        try runCommand(allocator, &[_][]const u8{
            "git", "submodule", "add", SUBMODULE_URL, SUBMODULE_PATH,
        });
    } else {
        std.debug.print("Submodule already tracked.\n", .{});
    }

    std.debug.print("Initializing and updating submodule...\n", .{});
    try fs.cwd().makePath(SUBMODULE_PATH);
    try runCommand(allocator, &[_][]const u8{ "git", "submodule", "update", "--init", "--force" });

    // Checkout the specific tag
    std.debug.print("Checking out tag {s}...\n", .{SUBMODULE_TAG});
    const git_args = &[_][]const u8{ "git", "-C", SUBMODULE_PATH, "checkout", SUBMODULE_TAG };
    try runCommand(allocator, git_args);

    std.debug.print("Submodule status:\n", .{});
    try runCommand(allocator, &[_][]const u8{ "git", "submodule", "status" });

    std.debug.print("Vendoring selected files from {s} to {s}...\n", .{ SUBMODULE_PATH, VENDOR_PATH });
    try copySelectedDirs(allocator, SUBMODULE_PATH, VENDOR_PATH);

    std.debug.print("Setup complete.\n", .{});
}
