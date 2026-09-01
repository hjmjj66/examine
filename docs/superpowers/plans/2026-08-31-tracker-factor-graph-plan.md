# Tracker Factor-Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace ordinary-vehicle EKF prediction with a ROS2 `tracker` package using GTSAM, preserving `TargetStateArray` and downstream topic contracts.

**Architecture:** Extend solver output with one observation object per armor containing world pose, camera pose, the timestamped camera-to-world transform, image corners, and class. Maintain one shared GTSAM target graph per ordinary ID with a fixed 30-frame window; keep outpost tracking unchanged.

**Tech Stack:** ROS2 `ament_cmake`, C++17, Eigen3, GTSAM/ISAM2, geometry_msgs, aim_msgs, GoogleTest, Python launch.

---

## File Map

- Modify `src/aim_msgs/msg/ArmorPoseSet.msg` and `src/aim_msgs/CMakeLists.txt`; create `src/aim_msgs/msg/ArmorPoseObservation.msg`.
- Modify `src/aim_solver/src/aim_solver_node.cpp`; create `src/aim_solver/test/test_pose_observation_forwarding.cpp`.
- Create `src/tracker/CMakeLists.txt`, `package.xml`, `config/tracker.yaml`, and `launch/tracker.launch.py`.
- Create tracker headers `config.hpp`, `measurement.hpp`, `factors.hpp`, `target_tracker.hpp`, and `tracker_node.hpp`.
- Create tracker sources `factors.cpp`, `target_tracker.cpp`, `tracker_node.cpp`, and `tracker_main.cpp`.
- Create tests under `src/tracker/test`.
- Modify `src/sentry_tf/launch/sentry_bringup.launch.py`, `start.bash`, and only the predictor launch references in `BUILD_AND_RUN.md` after standalone verification.
- Keep `src/aim_predictor` as the rollback baseline until NUC verification succeeds.

### Task 1: Extend the solver observation contract

**Files:** `src/aim_msgs/msg/ArmorPoseObservation.msg`, `src/aim_msgs/msg/ArmorPoseSet.msg`, `src/aim_msgs/CMakeLists.txt`, `src/aim_solver/src/aim_solver_node.cpp`, `src/aim_solver/test/test_pose_observation_forwarding.cpp`.

- [ ] **Step 1: Write a failing forwarding test.** Add a test for `aim_solver::makeArmorPoseObservation(source, camera_pose, world_pose)` that sets one corner, one class ID, and one coordinate in both poses, then asserts exact equality in all returned fields.
- [ ] **Step 2: Run `colcon test --packages-select aim_solver --ctest-args -R PoseObservationForwarding`.** Expected: missing message/helper or compilation failure.
- [ ] **Step 3: Add exact message definitions.** `ArmorPoseObservation.msg` contains `geometry_msgs/Pose pose`, `geometry_msgs/Pose camera_pose`, `geometry_msgs/Transform camera_to_world`, `geometry_msgs/Point[4] corners`, and `aim_msgs/ArmorClass armor_class`. `ArmorPoseSet.msg` contains `uint8 id`, `std_msgs/Header header`, and `aim_msgs/ArmorPoseObservation[] observations`. Register the new message and dependencies in `rosidl_generate_interfaces`.
- [ ] **Step 4: Implement forwarding.** Preserve the camera pose before TF conversion, copy detector corners/class, apply existing world transform and yaw optimization to `pose`, and append one observation per solved armor. Remove the old parallel pose-array write.
- [ ] **Step 5: Run `colcon build --packages-select aim_msgs aim_solver --symlink-install` and the forwarding test.** Expected: build and test pass.
- [ ] **Step 6: Commit with `git add src/aim_msgs src/aim_solver; git commit -m "feat: forward armor observations from solver"`.**

### Task 2: Add GTSAM factor primitives

**Files:** `src/tracker/CMakeLists.txt`, `src/tracker/package.xml`, `src/tracker/include/tracker/{config.hpp,measurement.hpp,factors.hpp}`, `src/tracker/src/factors.cpp`, `src/tracker/test/test_factors.cpp`.

- [ ] **Step 1: Write failing factor tests.** Cover zero constant-velocity translation, zero velocity change, yaw wrapping across `+pi/-pi`, geometry residuals for armor indices 0 and 1 including radius/height offsets, and zero reprojection residual for a known pinhole projection.
- [ ] **Step 2: Run `colcon test --packages-select tracker --ctest-args -R tracker_factors`.** Expected: missing package or failing tests.
- [ ] **Step 3: Implement factor classes.** Provide `TranslationFactor(Xpre,Vpre,Xcur,dt)`, `VelocityFactor(Vpre,Vcur)`, `YawFactor(Rpre,Wpre,Rcur,dt)`, `VyawFactor(Wpre,Wcur)`, `ArmorGeometryFactor(Pcamera,radius,radius_offset,height_offset,center_yaw,center_position,Tcamera_to_world,index)`, and `ArmorReprojFactor(Pcamera,calibration,armor_points,corners)`. Their residuals must match the meanings in `jlu_vision_26`: translation/velocity continuity, wrapped yaw continuity, tangential/radial/height/armor-yaw geometry, and eight pixel residual components.
- [ ] **Step 4: Implement configuration and measurement types.** Define `CameraSource`, timestamped observations, ROS conversion, sigma vectors, geometry limits, lifecycle values, speed-dependent process noise, and per-camera scales. `camera_pose` is an initial value only; the corner factor is the only pixel measurement factor.
- [ ] **Step 5: Add `find_package(GTSAM REQUIRED)` and ROS/Eigen dependencies.** Run `colcon build --packages-select aim_msgs tracker --symlink-install` and the factor tests on the NUC. Expected: build and tests pass.
- [ ] **Step 6: Commit with `git add src/tracker; git commit -m "feat: add tracker gtsam factors"`.**

### Task 3: Implement one target and its fixed window

**Files:** `src/tracker/include/tracker/target_tracker.hpp`, `src/tracker/src/target_tracker.cpp`, `src/tracker/test/test_target_tracker.cpp`, `src/tracker/test/test_window.cpp`.

- [ ] **Step 1: Write failing tests.** Assert state order `[cx,vx,cy,vy,cz,vz,yaw,vyaw,radius,radius_offset,height_offset]`, initialization, prediction-only behavior, yaw wrapping, old-timestamp rejection, and exactly 30 retained frame records after adding 31 frames.
- [ ] **Step 2: Run `colcon test --packages-select tracker --ctest-args -R "target|window"`.** Expected: missing implementation or failures.
- [ ] **Step 3: Implement `TargetTracker`.** Expose `initialize`, `acceptTimestamp`, `predict`, `addMeasurement`, `optimize`, `active`, `converged`, `diverged`, `jumped`, and `toMessage`. Use per-frame `X/V/R/W` and camera-frame `Pose3` variables plus shared radius, radius offset, and height offset. Add priors, motion, geometry, and reprojection factors.
- [ ] **Step 4: Implement rollover.** Store timestamped frame records in a deque. Below 30 frames update `ISAM2` incrementally. On frame 31 rebuild a local graph and `ISAM2` from the newest 30 frames, seed it with the previous estimate, and ensure every factor key has a value.
- [ ] **Step 5: Implement narrow failures.** Failed optimization returns false and preserves the last valid estimate; non-finite state or invalid geometry marks the target diverged; no hidden retries or recovery state machine.
- [ ] **Step 6: Run target/window tests and commit with `git add src/tracker; git commit -m "feat: add fixed-window vehicle tracker"`.** Expected: all pass.

### Task 4: Integrate the shared multi-camera node

**Files:** `src/tracker/include/tracker/tracker_node.hpp`, `src/tracker/src/{tracker_node.cpp,tracker_main.cpp}`, `src/tracker/config/tracker.yaml`, `src/tracker/launch/tracker.launch.py`, `src/tracker/test/test_tracker_node.cpp`.

- [ ] **Step 1: Write failing node tests.** Verify three camera messages with one ID update one tracker, source-specific sigma selection, ID 6 filtering, old-timestamp dropping, prediction-only empty input, and exact `TargetState` field mapping.
- [ ] **Step 2: Implement subscriptions and map.** Subscribe to the unchanged three solver topics and maintain one `std::map<uint8_t, TargetTracker>` for all sources. Preserve front-target hold, confirmation interval, timeout, visualization, delay statistics, and fused output topic.
- [ ] **Step 3: Implement callback order.** Drop stamps older than the pipeline stamp; predict active trackers; group observations by ID excluding ID 6; initialize or update graphs; optimize touched targets; remove timed-out targets; publish `/aim_predictor/fused/target_states`. Accept equal timestamps, with no wait queue or OOSM replay.
- [ ] **Step 4: Add configuration and launch.** Put `window_size: 30`, existing lifecycle values, geometry/motion/reprojection sigmas, speed bands, and front_0/front_1/back scales under `tracker_node.ros__parameters`. The launch file loads `tracker.yaml` and starts `tracker_node`.
- [ ] **Step 5: Run `colcon test --packages-select tracker --ctest-args -R tracker_node`.** Expected: all node tests pass.
- [ ] **Step 6: Commit with `git add src/tracker; git commit -m "feat: integrate shared multi-camera tracker node"`.**

### Task 5: Migrate bringup while preserving downstream contracts

**Files:** `src/sentry_tf/launch/sentry_bringup.launch.py`, `start.bash`, predictor references in `BUILD_AND_RUN.md`, `src/tracker/test/test_launch_contract.py`.

- [ ] **Step 1: Write a launch contract test.** Assert that bringup includes package `tracker` and `tracker.launch.py`, retains `/aim_predictor/fused/target_states`, and still includes `aim_outpost_predictor`, `aim_armor_decider`, and `aim_armor_controller`.
- [ ] **Step 2: Replace only package launch references.** Change `aim_predictor` package references to `tracker`; leave all topic names and downstream message types unchanged.
- [ ] **Step 3: Run `python -m pytest src/tracker/test/test_launch_contract.py -q` and build `aim_msgs aim_solver tracker aim_outpost_predictor aim_armor_decider aim_armor_controller`.** Expected: test passes and selected packages build.
- [ ] **Step 4: Commit with `git add src/sentry_tf/launch/sentry_bringup.launch.py start.bash BUILD_AND_RUN.md src/tracker/test/test_launch_contract.py; git commit -m "feat: launch tracker in sentry pipeline"`.**

### Task 6: Verify on the NUC and decide cleanup

**Files:** all tracker tests; leave `src/aim_predictor` intact during this task.

- [ ] **Step 1: Run `colcon test --packages-select tracker aim_solver --ctest-args -R "tracker|PoseObservation"` and `colcon test-result --verbose`.** Expected: no focused failures.
- [ ] **Step 2: Build the relevant chain with `colcon build --packages-select aim_msgs aim_solver tracker aim_outpost_predictor aim_armor_decider aim_armor_controller --symlink-install`.** Expected: installed NUC GTSAM resolves.
- [ ] **Step 3: Run `git diff --check` and search `src/sentry_tf`, `start.bash`, and `src/tracker` for active old predictor launch references.** Expected: no active launch starts the old package.
- [ ] **Step 4: Runtime-check `ros2 node list | grep tracker_node`, `ros2 topic echo --once /aim_predictor/fused/target_states`, and `ros2 topic info /aim_outpost_predictor/outpost_state`.** Expected: tracker publishes the unchanged state message and outpost remains active.
- [ ] **Step 5: Replay identical input through old and new nodes where available; compare IDs, fields, timestamps, predicted armor count, and decider/controller subscription success.** Do not remove `src/aim_predictor` or claim numerical equivalence until this comparison passes.
- [ ] **Step 6: Remove the old package only in a separate cleanup commit after explicit deployment confirmation.**

## Plan Self-Review

The plan covers the approved message extension, solver forwarding, GTSAM factors, eleven-state mapping, 30-frame window, shared camera tracker, source-specific noise, narrow failure behavior, outpost isolation, unchanged downstream topics, NUC GTSAM prerequisite, launch migration, and focused/full verification. It contains no unresolved `TODO` or `TBD` steps.
