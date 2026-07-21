#pragma once

#include <algorithm>

namespace aim_armor_controller
{

struct LegacyTimingInput
{
  double measurement_age_sec{0.0};
  double fly_time_sec{0.0};
  bool include_processing_delay{true};
  double system_response_time_sec{0.0};
};

struct LegacyTimingOutput
{
  double processing_delay_sec{0.0};
  double extra_delay_sec{0.0};
  double predict_time_sec{0.0};
};

inline LegacyTimingOutput computeLegacyPredictTime(const LegacyTimingInput & input)
{
  LegacyTimingOutput output;
  output.processing_delay_sec =
    input.include_processing_delay ? std::max(0.0, input.measurement_age_sec) : 0.0;
  output.extra_delay_sec = output.processing_delay_sec + input.system_response_time_sec;
  output.predict_time_sec = input.fly_time_sec + output.extra_delay_sec;
  return output;
}

}  // namespace aim_armor_controller
