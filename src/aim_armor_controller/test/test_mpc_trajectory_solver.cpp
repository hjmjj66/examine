#include "aim_armor_controller/mpc_trajectory_solver.hpp"

#include <cassert>
#include <cmath>

namespace
{

void testNoAirTrajectory()
{
  const auto result = aim_armor_controller::solveMpcTrajectory(
    23.0, 10.0, 1.0, 0.5, false);
  assert(!result.unsolvable);
  assert(std::isfinite(result.yaw));
  assert(std::isfinite(result.pitch));
  assert(std::isfinite(result.fly_time));
}

void testAirTrajectory()
{
  const auto result = aim_armor_controller::solveMpcTrajectory(
    23.0, 10.0, 1.0, 0.5, true);
  assert(!result.unsolvable);
  assert(std::isfinite(result.yaw));
  assert(std::isfinite(result.pitch));
  assert(std::isfinite(result.fly_time));
}

void testInvalidSpeed()
{
  const auto result = aim_armor_controller::solveMpcTrajectory(
    0.0, 10.0, 1.0, 0.5, true);
  assert(result.unsolvable);
}

}  // namespace

int main()
{
  testNoAirTrajectory();
  testAirTrajectory();
  testInvalidSpeed();
  return 0;
}
