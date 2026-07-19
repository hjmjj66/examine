# Controller Aim Result and Direct Trajectory Design

Date: 2026-07-19

## Goal

Allow the controller to choose between direct gimbal control through
`/ly/control/trajectory` and behavior-tree control through `/ly/aim/result`,
while preserving the existing aim result fields and exposing the MPC
trajectory derivatives through the aim result.

## Scope

Only the main package under `src/` is in scope. The untracked
`ly_aim_sentry/src/aim_armor_controller` copy is not modified.

## Current State

- `publish_control_trajectory` already controls creation and publication of
  `/ly/control/trajectory`, but the checked-in YAML value is `true`.
- `aim_msgs/ControlAngles.msg` contains yaw, pitch, yaw/pitch angular velocity,
  and yaw/pitch angular acceleration, expressed in degrees, degrees per second,
  and degrees per second squared respectively.
- `sentry_msgs/AimResult.msg` currently contains header, follow, fire, pitch,
  and yaw. The controller publishes it from MPC, legacy, and fallback paths.

## Design

### Direct trajectory switch

Reuse `publish_control_trajectory` as the direct-control switch. Its YAML value
and the C++ declaration/member default become `false`, so the normal behavior
is to avoid creating or publishing `/ly/control/trajectory`. Setting it to
`true` retains the existing direct trajectory publisher and does not suppress
`/ly/aim/result`.

### Aim result payload

Extend `sentry_msgs/AimResult.msg` with:

```text
float32 yaw_omega
float32 pitch_omega
float32 yaw_alpha
float32 pitch_alpha
```

The existing `header`, `follow`, `fire`, `pitch`, and `yaw` fields remain
unchanged. The new fields use the same names and units as `ControlAngles.msg`.

On a successful MPC solve, all six kinematic values are copied from the
trajectory plan into `AimResult`. On fallback and the legacy non-MPC path,
angles remain as currently published and the four derivative values are zero.
The direct trajectory publisher receives the same values as before and remains
independently gated by `publish_control_trajectory`.

### Implementation boundary

Use one small controller-side mapping helper for the six kinematic fields so
the MPC, legacy, and fallback publication paths cannot drift apart. The helper
will not change the existing follow/fire semantics or any existing fields.

## Testing

- Add a focused unit test for the mapping helper that verifies all four new
  derivative fields and the existing yaw/pitch values.
- Build the `sentry_msgs` and `aim_armor_controller` packages so the generated
  message interface and controller compile together.
- Run the controller package tests and inspect the final diff to verify that
  only the intended main-package files changed in addition to this design
  document.

## Compatibility

Adding fields preserves all existing `AimResult` fields and does not rename or
remove the existing topic. Consumers must rebuild against the updated ROS
interface to access the new fields.

