#include "aim_armor_controller/mpc_math.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

void testShortestAngleDeltaAcrossWrap()
{
  const double delta = aim_armor_controller::shortestAngleDelta(
    -179.0 * kPi / 180.0, 179.0 * kPi / 180.0);
  assert(std::abs(delta - 2.0 * kPi / 180.0) < 1e-12);
}

void testUnwrapAngleNearReference()
{
  const double unwrapped = aim_armor_controller::unwrapAngleNear(
    -179.0 * kPi / 180.0, 179.0 * kPi / 180.0);
  assert(std::abs(unwrapped - 181.0 * kPi / 180.0) < 1e-12);
}

void testDelaySamplesAreCenteredOnBaseDelay()
{
  const std::vector<double> samples = aim_armor_controller::buildCenteredDelaySamples(
    0.20, 0.01, 4);
  assert(samples.size() == 5U);
  assert(std::abs(samples[0] - 0.18) < 1e-12);
  assert(std::abs(samples[2] - 0.20) < 1e-12);
  assert(std::abs(samples[4] - 0.22) < 1e-12);
}

void testAbsoluteYawUsesFixedWorldFrame()
{
  const double yaw = aim_armor_controller::absoluteYawFromWorldPoint(
    1.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  assert(std::abs(yaw - kPi / 4.0) < 1e-12);
}

}  // namespace

int main()
{
  testShortestAngleDeltaAcrossWrap();
  testUnwrapAngleNearReference();
  testDelaySamplesAreCenteredOnBaseDelay();
  testAbsoluteYawUsesFixedWorldFrame();
  return 0;
}
