# Controller Aim Result and Direct Trajectory Implementation Plan

> **For agentic workers:** Execute this plan task-by-task with review checkpoints. Preserve unrelated working-tree changes.

**Goal:** Publish the controller's six kinematic aim values through `/ly/aim/result` while making direct `/ly/control/trajectory` publication an explicit opt-in configuration.

**Architecture:** Extend `sentry_msgs/AimResult` without removing existing fields. Centralize assignment of the six kinematic values in a small controller header helper, then call it from MPC, legacy, and fallback publication paths. Reuse the existing `publish_control_trajectory` parameter and set its code and YAML defaults to `false`.

**Tech Stack:** ROS 2, C++17, `ament_cmake`, `ament_cmake_gtest`, `sentry_msgs`, `aim_armor_controller`.

---

### Task 1: Add the failing mapping test

**Files:**
- Create: `src/aim_armor_controller/test/test_aim_result_kinematics.cpp`
- Modify: `src/aim_armor_controller/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create a gtest that includes `aim_armor_controller/aim_result_kinematics.hpp`, constructs a `sentry_msgs::msg::AimResult`, calls `setAimResultKinematics(result, 10.0, 20.0, 1.5, 2.5, 3.5, 4.5)`, and asserts that `yaw`, `pitch`, `yaw_omega`, `pitch_omega`, `yaw_alpha`, and `pitch_alpha` equal those values.

Register the test with `ament_add_gtest`, add the controller include directory, and link the `sentry_msgs` dependency.

- [ ] **Step 2: Run the test to verify RED**

Run from the remote repository:

```bash
cd /home/hustlyrm/sentry.aim
source /opt/ros/$ROS_DISTRO/setup.bash
colcon test --packages-select aim_armor_controller --ctest-args -R test_aim_result_kinematics
```

Expected result: the test cannot compile because `aim_result_kinematics.hpp` does not exist.

### Task 2: Implement the mapping helper and message fields

**Files:**
- Create: `src/aim_armor_controller/include/aim_armor_controller/aim_result_kinematics.hpp`
- Modify: `src/sentry_msgs/msg/AimResult.msg`

- [ ] **Step 1: Extend the ROS message**

Append these fields after the existing `yaw` field, preserving all existing fields:

```text
float32 yaw_omega
float32 pitch_omega
float32 yaw_alpha
float32 pitch_alpha
```

- [ ] **Step 2: Implement the minimal helper**

Define `aim_armor_controller::setAimResultKinematics` as an inline function taking a `sentry_msgs::msg::AimResult &` and six `double` values. Assign all six values using `static_cast<float>`.

- [ ] **Step 3: Run the focused test to verify GREEN**

Run the same `colcon test` command from Task 1. Expected result: `test_aim_result_kinematics` passes.

### Task 3: Route controller output through AimResult and gate direct trajectory output

**Files:**
- Modify: `src/aim_armor_controller/src/armor_controller_node.cpp`
- Modify: `src/aim_armor_controller/config/armor_controller.yaml`

- [ ] **Step 1: Use the helper in fallback**

Include the new helper and call it after the existing fallback yaw/pitch assignments with zero angular velocity and acceleration.

- [ ] **Step 2: Use the helper in MPC**

After assigning the existing MPC `aim_msg.yaw` and `aim_msg.pitch`, call the helper with `output_yaw_deg`, `output_pitch_deg`, `plan.yaw_velocity * 180.0 / kPi`, `plan.pitch_velocity * 180.0 / kPi`, `plan.yaw_acceleration * 180.0 / kPi`, and `plan.pitch_acceleration * 180.0 / kPi`.

- [ ] **Step 3: Use the helper in the legacy path**

After assigning the existing legacy `aim_msg.yaw` and `aim_msg.pitch`, call the helper with `command.yaw_deg`, `command.pitch_deg`, and four zero derivative values.

- [ ] **Step 4: Make direct trajectory opt-in**

Change the `declare_parameter<bool>("publish_control_trajectory", true)` default and the member initializer to `false`. Change the YAML parameter value to `false` and describe it as the direct-control switch. Leave the existing conditional publisher creation and `publishControlTrajectory` guard intact.

- [ ] **Step 5: Inspect the diff**

Confirm that existing `AimResult` assignments for header, follow, fire, yaw, and pitch remain present, and that only the main `src/` package files are modified.

### Task 4: Build and verify

**Files:**
- Modify only files listed above if corrections are required by verification.

- [ ] **Step 1: Build the message and controller packages**

```bash
cd /home/hustlyrm/sentry.aim
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select sentry_msgs aim_armor_controller --symlink-install
```

Expected result: exit code 0.

- [ ] **Step 2: Run all controller tests**

```bash
cd /home/hustlyrm/sentry.aim
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
colcon test --packages-select aim_armor_controller
colcon test-result --verbose
```

Expected result: all selected controller tests pass, including `test_aim_result_kinematics`.

- [ ] **Step 3: Verify the final working tree**

```bash
cd /home/hustlyrm/sentry.aim
git diff --check
git status --short
```

Report any pre-existing dirty files separately from the files changed for this task.
