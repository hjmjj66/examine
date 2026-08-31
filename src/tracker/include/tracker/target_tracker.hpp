#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

#include "aim_msgs/msg/target_state.hpp"
#include "tracker/config.hpp"
#include "tracker/measurement.hpp"

namespace tracker
{

struct CameraCalibration
{
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
  gtsam::Pose3 camera_to_world;
};

class TargetTracker
{
public:
  TargetTracker();
  explicit TargetTracker(const TrackerConfig & config);
  TargetTracker(const TrackerConfig & config, const std::array<CameraCalibration, 3> & calibrations);

  void setCameraCalibration(CameraSource source, const CameraCalibration & calibration);

  bool initialize(const ArmorObservation & observation);
  [[nodiscard]] bool acceptTimestamp(const builtin_interfaces::msg::Time & stamp) const;
  bool predict(const builtin_interfaces::msg::Time & stamp);
  bool addMeasurement(const ArmorObservation & observation);
  bool optimize();

  [[nodiscard]] bool active() const;
  [[nodiscard]] bool converged() const;
  [[nodiscard]] bool diverged() const;
  [[nodiscard]] bool jumped() const;
  [[nodiscard]] std::uint8_t target_id() const;
  [[nodiscard]] const Eigen::VectorXd & state() const;
  [[nodiscard]] aim_msgs::msg::TargetState toMessage() const;
  [[nodiscard]] aim_msgs::msg::TargetState toMessage(const std_msgs::msg::Header & header) const;

  [[nodiscard]] std::size_t frameCount() const;
  [[nodiscard]] std::size_t windowSize() const;
  [[nodiscard]] std::optional<builtin_interfaces::msg::Time> oldestFrameStamp() const;
  [[nodiscard]] std::optional<builtin_interfaces::msg::Time> latestFrameStamp() const;
  [[nodiscard]] const gtsam::Values & values() const;
  [[nodiscard]] bool allFactorKeysHaveValues() const;

private:
  struct MeasurementRecord
  {
    ArmorObservation observation;
    std::size_t armor_index{0};
    gtsam::Key pose_key{0};
  };

  struct FrameRecord
  {
    builtin_interfaces::msg::Time stamp;
    std::size_t frame_id{0};
    std::vector<MeasurementRecord> measurements;
  };

  static std::size_t cameraIndex(CameraSource source);
  static bool isOlder(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs);
  static double secondsBetween(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs);

  static double wrapAngle(double angle);
  static double yawFromPose(const geometry_msgs::msg::Pose & pose);
  static gtsam::Pose3 toPose3(const geometry_msgs::msg::Pose & pose);
  static gtsam::Point3 toPoint3(const geometry_msgs::msg::Point & point);
  static geometry_msgs::msg::Quaternion yawQuaternion(double yaw);
  static bool finiteState(const Eigen::VectorXd & state);

  double cameraScale(CameraSource source) const;
  std::size_t inferArmorIndex(const ArmorObservation & observation) const;
  bool validGeometry(const Eigen::VectorXd & state) const;
  void initializeState(const ArmorObservation & observation);
  void initializeGraph();

  gtsam::Key frameKey(char prefix, std::size_t frame_id) const;
  gtsam::Key poseKey(std::size_t pose_id) const;
  void insertIfMissing(gtsam::Values & values, gtsam::Key key, const gtsam::Point3 & value) const;
  void insertIfMissing(gtsam::Values & values, gtsam::Key key, const gtsam::Vector3 & value) const;
  void insertIfMissing(gtsam::Values & values, gtsam::Key key, const gtsam::Rot2 & value) const;
  void insertIfMissing(gtsam::Values & values, gtsam::Key key, const gtsam::Pose3 & value) const;
  void insertIfMissing(gtsam::Values & values, gtsam::Key key, double value) const;
  void addFactor(gtsam::NonlinearFactorGraph & graph, const gtsam::NonlinearFactor::shared_ptr & factor);
  void addSharedValuesAndPriors(gtsam::NonlinearFactorGraph & graph, gtsam::Values & values);
  void addFrame(
    gtsam::NonlinearFactorGraph & graph,
    gtsam::Values & values,
    const FrameRecord & frame,
    const FrameRecord * previous,
    bool add_priors);
  void addMeasurementFactors(
    gtsam::NonlinearFactorGraph & graph,
    gtsam::Values & values,
    const FrameRecord & frame,
    const MeasurementRecord & measurement);
  void queueFrame(const FrameRecord & frame, const FrameRecord * previous);
  void queueMeasurement(const FrameRecord & frame, const MeasurementRecord & measurement);
  bool rebuildGraph();
  bool setStateFromValues(const gtsam::Values & estimate, std::size_t frame_id);
  bool calibrationIsUsable(CameraSource source) const;
  std::array<gtsam::Point3, 4> armorPoints(const ArmorObservation & observation) const;

  TrackerConfig config_{};
  std::array<CameraCalibration, 3> calibrations_{};
  std::deque<FrameRecord> frames_;
  Eigen::VectorXd state_{Eigen::VectorXd::Zero(11)};
  std::uint8_t target_id_{0};
  builtin_interfaces::msg::Time last_stamp_{};
  std::size_t next_frame_id_{0};
  std::size_t next_pose_id_{0};
  std::size_t update_count_{0};
  std::size_t last_frame_id_{0};
  bool initialized_{false};
  bool diverged_{false};
  bool jumped_{false};
  std::size_t last_armor_index_{0};

  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values last_values_;
  gtsam::NonlinearFactorGraph pending_graph_;
  gtsam::Values pending_values_;
  std::set<gtsam::Key> factor_keys_;
  bool needs_rebuild_{false};
  bool optimization_failed_{false};
};

}  // namespace tracker
