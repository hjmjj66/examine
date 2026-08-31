# Tracker Factor-Graph Design

## Goal

Replace the ordinary-vehicle `aim_predictor` package with a new `tracker`
package. The new package must produce the same `aim_msgs/TargetStateArray`
semantics for downstream decision and control nodes, while replacing the EKF
with a GTSAM factor graph that estimates the same 11 target quantities.

The outpost path remains in `aim_outpost_predictor` and is outside this
change.

## Package and ROS boundaries

The package, node, and executable are named `tracker`, `tracker_node`, and
`tracker_node`. Input and output topics remain unchanged:

- `/aim_solver/front_0/armor_pose_sets`
- `/aim_solver/front_1/armor_pose_sets`
- `/aim_solver/back/armor_pose_sets`
- `/aim_predictor/fused/target_states`

The decider, controller, and outpost predictor keep their existing ROS
interfaces. Launch files and build metadata will refer to the new package.

One tracker is maintained per ordinary target ID. The three camera sources
feed observations into that target's single shared graph. Process noise is
shared; camera-specific measurement noise is selected by observation source.

## Observation message

The existing pose-only output is insufficient for pixel reprojection factors.
`aim_msgs` will add `ArmorPoseObservation.msg`:

```text
geometry_msgs/Pose pose
geometry_msgs/Pose camera_pose
geometry_msgs/Point[4] corners
aim_msgs/ArmorClass armor_class
```

`ArmorPoseSet.msg` will contain:

```text
uint8 id
std_msgs/Header header
aim_msgs/ArmorPoseObservation[] observations
```

`pose` is the transformed, yaw-optimized pose in the common world frame.
`camera_pose` is the corresponding camera-frame pose used by reprojection.
`corners` are the detector's original image points. `armor_class` identifies
the armor geometry. The four values are carried in one object so pose, type,
and corners cannot become mismatched through parallel arrays.

`aim_solver` already has all four values while processing `ArmorSetArray`; it
will forward them in the output message. No new upstream data source is
required. The tracker uses `camera_pose` as the reprojection variable's initial
value, not as a second independent PnP measurement factor, preventing double
counting of the same corner measurement.

## Factor-graph state

The tracker preserves the current output state ordering:

```text
[center_x, velocity_x, center_y, velocity_y, center_z, velocity_z,
 yaw, angular_velocity, radius, radius_offset, height_offset]
```

For each frame `k`, the graph contains:

```text
X(k): center position
V(k): center velocity
R(k): center yaw
W(k): center angular velocity
P_i(k): camera-frame Pose3 for each observed armor i
```

The vehicle geometry parameters are shared across the active window:

```text
radius
radius_offset
height_offset
```

The ordinary four-armor model remains:

```text
armor 0, 2: radius = radius; radius z = center_z
armor 1, 3: radius = radius + radius_offset;
             armor z = center_z + height_offset
```

The graph uses the following factors:

- `TranslationFactor`: `X(k) - X(k-1) - V(k-1) * dt`
- `VelocityFactor`: `V(k) - V(k-1)`
- `YawFactor`: `R(k) - R(k-1) - W(k-1) * dt`, with angle wrapping
- `VyawFactor`: `W(k) - W(k-1)`
- geometry factor: tangential, radial, height, and armor-yaw residuals linking
  `P_i(k)`, `X(k)`, `R(k)`, and the shared geometry parameters
- reprojection factor: projects the known armor corners from `P_i(k)` using
  the camera calibration and compares them with `corners`

The geometry factor uses the camera-to-world transform associated with the
measurement timestamp. The optimized vehicle state, rather than an arbitrary
single armor pose, is used to generate the four `predicted_armors` in the
published message.

## Window and lifecycle

Each target owns a fixed 30-frame window, configured as:

```yaml
window_size: 30
```

New observations are added in timestamp order. Existing active targets are
predicted to the incoming timestamp before measurement factors are added.
When the window exceeds 30 frames, the oldest frame is marginalized into a
prior. If the installed GTSAM version does not provide a suitable fixed-lag
API, the implementation rebuilds a local graph and `ISAM2` instance from the
remaining window using the previous result as the initial value.

Initialization, confirmation count, geometry initialization, convergence,
and loss timeout preserve the current predictor semantics. A target without a
measurement is predicted and published until `target_lost_timeout_sec`; then
its tracker is removed. Target ID 6 is filtered out and remains owned by the
outpost predictor.

Messages older than the last processed timestamp are dropped. Equal
timestamps are accepted as additional observations. No waiting queue or OOSM
replay is added.

## Parameters and noise

All tracker parameters move to `tracker/config/tracker.yaml`. Existing
predictor and `jlu_vision_26` parameter meanings and initial values are reused
before any new tuning.

GTSAM noise models use standard deviations (`sigma`). The original EKF process
noise covariance is converted to the corresponding motion-factor sigmas. The
low, middle, and high angular-velocity process-noise regimes remain.

The configuration includes priors, motion-factor noise, geometry-factor noise,
pixel reprojection noise, initialization values, window size, target
lifecycle thresholds, and per-camera measurement-noise scales. NIS is used
only for diagnostics and invalid-track decisions; it does not adapt factor
weights online.

## Error handling

The implementation deliberately keeps failure handling narrow:

- invalid or empty input: predict existing targets and skip measurement factors;
- out-of-order input: drop the message;
- GTSAM optimization failure: retain the last valid state for that update;
- non-finite state or invalid geometry: discard that target tracker;
- timeout: remove the target tracker and wait for normal reinitialization.

No automatic noise retuning, repeated hidden retries, or complex recovery
state machine is introduced.

## Build and compatibility

The NUC is assumed to provide GTSAM. The package will declare and use
`find_package(GTSAM REQUIRED)` and link the installed GTSAM libraries. The
local development computer is not required to build this package.

The original `aim_predictor` implementation is retained as a Git-recoverable
baseline until the new package builds and passes regression checks. The first
implementation pass does not modify `aim_outpost_predictor` or downstream
business logic.

## Verification

Tests will cover:

- message field correspondence and solver forwarding;
- state ordering and `TargetStateArray` compatibility;
- motion, geometry, yaw-wrap, and reprojection residuals;
- armor type and corner correspondence;
- 30-frame window size and sliding behavior;
- shared multi-camera tracker behavior and source-specific noise;
- timestamp monotonicity and equal-timestamp updates;
- initialization, prediction-only loss handling, timeout removal, and ID 6
  exclusion;
- building the tracker package and running downstream interface regression
  tests.

