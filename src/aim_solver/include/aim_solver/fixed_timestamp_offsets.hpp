#pragma once

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace aim_solver
{

struct FixedTimestampOffsets
{
  double front_sec{0.0};
  double front_1_sec{0.0};
  double back_sec{0.0};
};

template<typename ParameterGetter>
FixedTimestampOffsets loadFixedTimestampOffsets(ParameterGetter && get_parameter)
{
  const auto require = [&get_parameter](const char * name) {
      const auto value = get_parameter(name);
      if (!value.has_value()) {
        throw std::invalid_argument(
                std::string("missing required parameter: ") + name);
      }
      if (!std::isfinite(*value)) {
        throw std::invalid_argument(
                std::string("parameter must be finite: ") + name);
      }
      return *value;
    };

  return {
    require("front_tf_timestamp_offset_sec"),
    require("front_1_tf_timestamp_offset_sec"),
    require("back_tf_timestamp_offset_sec")
  };
}

}  // namespace aim_solver
