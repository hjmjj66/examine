#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "aim_msgs/msg/outpost_state.hpp"
#include "aim_msgs/msg/selected_target_id.hpp"
#include "aim_msgs/msg/target_state.hpp"
#include "aim_msgs/msg/target_state_array.hpp"
#include "sentry_msgs/msg/aim_target.hpp"
#include "sentry_msgs/msg/aim_target_array.hpp"

namespace
{

double distanceSquared(const geometry_msgs::msg::Point & point)
{
  return point.x * point.x + point.y * point.y + point.z * point.z;
}

struct TargetCandidate
{
  std_msgs::msg::Header header;
  uint8_t id{0};
  bool tracking{false};
  bool converged{false};
  geometry_msgs::msg::Point center;
};

TargetCandidate fromTargetState(const aim_msgs::msg::TargetState & target)
{
  TargetCandidate candidate;
  candidate.header = target.header;
  candidate.id = target.id;
  candidate.tracking = target.tracking;
  candidate.converged = target.converged;
  candidate.center = target.center;
  return candidate;
}

TargetCandidate fromOutpostState(const aim_msgs::msg::OutpostState & target)
{
  TargetCandidate candidate;
  candidate.header = target.header;
  candidate.id = target.id;
  candidate.tracking = target.tracking;
  candidate.converged = target.converged;
  candidate.center = target.center;
  return candidate;
}

bool isBetterCandidate(const TargetCandidate & candidate, const TargetCandidate & existing)
{
  if (candidate.tracking != existing.tracking) {
    return candidate.tracking && !existing.tracking;
  }
  if (candidate.converged != existing.converged) {
    return candidate.converged && !existing.converged;
  }

  const auto candidate_stamp = rclcpp::Time(candidate.header.stamp);
  const auto existing_stamp = rclcpp::Time(existing.header.stamp);
  if (candidate_stamp != existing_stamp) {
    return candidate_stamp > existing_stamp;
  }

  return distanceSquared(candidate.center) < distanceSquared(existing.center);
}

const TargetCandidate * nearestCandidate(const std::vector<TargetCandidate> & candidates)
{
  const TargetCandidate * selected = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & candidate : candidates) {
    const double dist = distanceSquared(candidate.center);
    if (dist < min_dist) {
      min_dist = dist;
      selected = &candidate;
    }
  }

  return selected;
}

}  // namespace

class AimArmorDeciderNode : public rclcpp::Node
{
public:
  AimArmorDeciderNode()
  : Node("aim_armor_decider_node")
  {
    declare_parameter<std::string>(
      "front_0_target_states_topic", "/aim_predictor/front_0/target_states");
    declare_parameter<std::string>(
      "front_1_target_states_topic", "/aim_predictor/front_1/target_states");
    declare_parameter<std::string>(
      "back_target_states_topic", "/aim_predictor/back/target_states");
    declare_parameter<std::string>(
      "outpost_state_topic", "/aim_outpost_predictor/outpost_state");
    declare_parameter<std::string>("armor_targets_topic", "/ly/aim/armor_targets");
    declare_parameter<std::string>("select_target_topic", "/ly/aim/select_target");
    declare_parameter<std::string>(
      "selected_target_id_topic", "/decider/selected_target_id");
    declare_parameter<double>("select_target_timeout_sec", 0.5);
    declare_parameter<int>("outpost_target_id", 6);
    declare_parameter<bool>("default_select_outpost_when_no_external_target", false);

    const auto front_0_topic = get_parameter("front_0_target_states_topic").as_string();
    const auto front_1_topic = get_parameter("front_1_target_states_topic").as_string();
    const auto back_topic = get_parameter("back_target_states_topic").as_string();
    outpost_state_topic_ = get_parameter("outpost_state_topic").as_string();
    const auto armor_targets_topic = get_parameter("armor_targets_topic").as_string();
    const auto select_target_topic = get_parameter("select_target_topic").as_string();
    const auto selected_target_id_topic =
      get_parameter("selected_target_id_topic").as_string();
    select_target_timeout_sec_ = get_parameter("select_target_timeout_sec").as_double();
    outpost_target_id_ = static_cast<uint8_t>(get_parameter("outpost_target_id").as_int());
    default_select_outpost_when_no_external_target_ =
      get_parameter("default_select_outpost_when_no_external_target").as_bool();

    targets_pub_ = create_publisher<sentry_msgs::msg::AimTargetArray>(
      armor_targets_topic, rclcpp::SensorDataQoS());
    selected_target_id_pub_ = create_publisher<aim_msgs::msg::SelectedTargetId>(
      selected_target_id_topic, rclcpp::SensorDataQoS());

    front_0_sub_ = create_subscription<aim_msgs::msg::TargetStateArray>(
      front_0_topic, rclcpp::SensorDataQoS(),
      std::bind(&AimArmorDeciderNode::onFront0TargetStates, this, std::placeholders::_1));
    front_1_sub_ = create_subscription<aim_msgs::msg::TargetStateArray>(
      front_1_topic, rclcpp::SensorDataQoS(),
      std::bind(&AimArmorDeciderNode::onFront1TargetStates, this, std::placeholders::_1));
    back_sub_ = create_subscription<aim_msgs::msg::TargetStateArray>(
      back_topic, rclcpp::SensorDataQoS(),
      std::bind(&AimArmorDeciderNode::onBackTargetStates, this, std::placeholders::_1));
    outpost_sub_ = create_subscription<aim_msgs::msg::OutpostState>(
      outpost_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AimArmorDeciderNode::onOutpostState, this, std::placeholders::_1));
    select_sub_ = create_subscription<sentry_msgs::msg::AimTarget>(
      select_target_topic, rclcpp::SensorDataQoS(),
      std::bind(&AimArmorDeciderNode::onSelectTarget, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "aim_armor_decider_node started. front_0=%s front_1=%s back=%s outpost=%s targets=%s "
      "select=%s selected_target_id=%s outpost_id=%d auto_outpost=%s",
      front_0_topic.c_str(), front_1_topic.c_str(), back_topic.c_str(),
      outpost_state_topic_.c_str(), armor_targets_topic.c_str(), select_target_topic.c_str(),
      selected_target_id_topic.c_str(), outpost_target_id_,
      default_select_outpost_when_no_external_target_ ? "true" : "false");
  }

private:
  void onSelectTarget(const sentry_msgs::msg::AimTarget::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    publishSelectedTargetId(msg->id);
    processLatestLocked();
  }

  bool selectTargetFresh() const
  {
    if (!has_selected_target_) {
      return false;
    }
    if (select_target_timeout_sec_ <= 0.0) {
      return true;
    }
    return (now() - selected_target_updated_at_).seconds() <= select_target_timeout_sec_;
  }

  void onFront0TargetStates(const aim_msgs::msg::TargetStateArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_front_0_ = msg;
    processLatestLocked();
  }

  void onFront1TargetStates(const aim_msgs::msg::TargetStateArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_front_1_ = msg;
    processLatestLocked();
  }

  void onBackTargetStates(const aim_msgs::msg::TargetStateArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_back_ = msg;
    if (!hasFrontCandidates()) {
      processLatestLocked();
    }
  }

  void onOutpostState(const aim_msgs::msg::OutpostState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_outpost_ = msg;
    if ((selectTargetFresh() && selected_target_id_ == outpost_target_id_) ||
        (!selectTargetFresh() && default_select_outpost_when_no_external_target_)) {
      processLatestLocked();
    }
  }

  bool hasFrontCandidates() const
  {
    return (latest_front_0_ && !latest_front_0_->targets.empty()) ||
           (latest_front_1_ && !latest_front_1_->targets.empty());
  }

  std::vector<TargetCandidate> collectFrontCandidates() const
  {
    std::map<uint8_t, TargetCandidate> best_per_id;

    auto merge_targets = [&best_per_id](const aim_msgs::msg::TargetStateArray::ConstSharedPtr & msg) {
        if (!msg) {
          return;
        }
        for (const auto & target : msg->targets) {
          const auto candidate = fromTargetState(target);
          auto it = best_per_id.find(candidate.id);
          if (it == best_per_id.end() || isBetterCandidate(candidate, it->second)) {
            best_per_id[candidate.id] = candidate;
          }
        }
      };

    merge_targets(latest_front_0_);
    merge_targets(latest_front_1_);

    std::vector<TargetCandidate> candidates;
    candidates.reserve(best_per_id.size());
    for (const auto & [id, candidate] : best_per_id) {
      (void)id;
      candidates.push_back(candidate);
    }
    return candidates;
  }

  std::vector<TargetCandidate> collectBackCandidates() const
  {
    std::vector<TargetCandidate> candidates;
    if (!latest_back_) {
      return candidates;
    }

    candidates.reserve(latest_back_->targets.size());
    for (const auto & target : latest_back_->targets) {
      candidates.push_back(fromTargetState(target));
    }
    return candidates;
  }

  void publishArmorTargets(
    const std_msgs::msg::Header & header,
    const std::vector<TargetCandidate> & candidates) const
  {
    sentry_msgs::msg::AimTargetArray targets;
    targets.header = header;

    targets.aim_targets.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      sentry_msgs::msg::AimTarget target;
      target.header = candidate.header;
      if (target.header.frame_id.empty()) {
        target.header = header;
      }
      target.position = candidate.center;
      target.id = candidate.id;
      targets.aim_targets.push_back(target);
    }

    targets_pub_->publish(targets);
  }

  void publishSelectedTargetId(uint8_t id)
  {
    aim_msgs::msg::SelectedTargetId selected_target_msg;
    selected_target_msg.header.stamp = now();
    selected_target_msg.id = id;
    selected_target_msg.valid = true;
    selected_target_id_pub_->publish(selected_target_msg);

    selected_target_id_ = id;
    selected_target_updated_at_ = now();
    has_selected_target_ = true;
  }

  std::optional<TargetCandidate> defaultOutpostCandidate() const
  {
    if (!default_select_outpost_when_no_external_target_ || !latest_outpost_) {
      return std::nullopt;
    }

    const auto candidate = fromOutpostState(*latest_outpost_);
    if (!candidate.tracking || !candidate.converged) {
      return std::nullopt;
    }
    return candidate;
  }

  const TargetCandidate * selectCandidate(const std::vector<TargetCandidate> & candidates) const
  {
    if (candidates.empty()) {
      return nullptr;
    }

    if (selectTargetFresh()) {
      auto it = std::find_if(
        candidates.begin(), candidates.end(),
        [this](const auto & candidate) { return candidate.id == selected_target_id_; });
      if (it != candidates.end()) {
        return &(*it);
      }
    }

    return nearestCandidate(candidates);
  }

  void processLatestLocked()
  {
    if (selectTargetFresh() && selected_target_id_ == outpost_target_id_) {
      if (latest_outpost_) {
        const auto outpost_candidate = fromOutpostState(*latest_outpost_);
        const std::vector<TargetCandidate> outpost_candidates{outpost_candidate};
        publishArmorTargets(latest_outpost_->header, outpost_candidates);
        return;
      }
    }

    const auto default_outpost_candidate =
      !selectTargetFresh() ? defaultOutpostCandidate() : std::nullopt;

    auto front_candidates = collectFrontCandidates();
    if (default_outpost_candidate.has_value()) {
      front_candidates.push_back(*default_outpost_candidate);
    }

    if (!front_candidates.empty()) {
      const auto header =
        latest_front_0_ ? latest_front_0_->header :
        (latest_front_1_ ? latest_front_1_->header : latest_outpost_->header);
      publishArmorTargets(header, front_candidates);
      if (!selectTargetFresh()) {
        if (const auto * candidate = selectCandidate(front_candidates)) {
          publishSelectedTargetId(candidate->id);
        }
      }
      return;
    }

    auto back_candidates = collectBackCandidates();
    if (!back_candidates.empty() && latest_back_) {
      publishArmorTargets(latest_back_->header, back_candidates);
      if (!selectTargetFresh()) {
        if (const auto * candidate = selectCandidate(back_candidates)) {
          publishSelectedTargetId(candidate->id);
        }
      }
    }
  }

  bool has_selected_target_{false};
  uint8_t selected_target_id_{0};
  rclcpp::Time selected_target_updated_at_{0, 0, RCL_ROS_TIME};
  double select_target_timeout_sec_{0.5};
  bool default_select_outpost_when_no_external_target_{false};

  std::mutex data_mutex_;
  aim_msgs::msg::TargetStateArray::ConstSharedPtr latest_front_0_;
  aim_msgs::msg::TargetStateArray::ConstSharedPtr latest_front_1_;
  aim_msgs::msg::TargetStateArray::ConstSharedPtr latest_back_;
  aim_msgs::msg::OutpostState::ConstSharedPtr latest_outpost_;

  std::string outpost_state_topic_;
  uint8_t outpost_target_id_{6};

  rclcpp::Publisher<sentry_msgs::msg::AimTargetArray>::SharedPtr targets_pub_;
  rclcpp::Publisher<aim_msgs::msg::SelectedTargetId>::SharedPtr selected_target_id_pub_;
  rclcpp::Subscription<aim_msgs::msg::TargetStateArray>::SharedPtr front_0_sub_;
  rclcpp::Subscription<aim_msgs::msg::TargetStateArray>::SharedPtr front_1_sub_;
  rclcpp::Subscription<aim_msgs::msg::TargetStateArray>::SharedPtr back_sub_;
  rclcpp::Subscription<aim_msgs::msg::OutpostState>::SharedPtr outpost_sub_;
  rclcpp::Subscription<sentry_msgs::msg::AimTarget>::SharedPtr select_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AimArmorDeciderNode>());
  rclcpp::shutdown();
  return 0;
}
