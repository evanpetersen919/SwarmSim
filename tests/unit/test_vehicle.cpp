#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "swarm/dynamics.hpp"
#include "swarm/vehicle.hpp"

using swarm::DroneCommand;
using swarm::DroneParams;
using swarm::DroneState;
using swarm::IVehicle;
using swarm::QuadVehicle;
using swarm::Vec3;

TEST(Vehicle, QuadVehicleMatchesTheDynamicsStepExactly) {
  const DroneParams params;
  const std::unique_ptr<const IVehicle> vehicle = std::make_unique<QuadVehicle>(params);
  DroneCommand command;
  command.accel = Vec3{3.0, -1.0, 0.5};
  command.yaw_rate = 0.7;
  DroneState direct;
  DroneState through;
  for (int i = 0; i < 500; ++i) {
    direct = swarm::step(direct, command, params, 0.01);
    through = vehicle->step(through, command, 0.01);
    ASSERT_EQ(direct.pos.x, through.pos.x);
    ASSERT_EQ(direct.pos.y, through.pos.y);
    ASSERT_EQ(direct.pos.z, through.pos.z);
    ASSERT_EQ(direct.vel.x, through.vel.x);
    ASSERT_EQ(direct.vel.y, through.vel.y);
    ASSERT_EQ(direct.vel.z, through.vel.z);
    ASSERT_EQ(direct.yaw, through.yaw);
  }
}

TEST(Vehicle, QuadVehicleStillRejectsInvalidLimitsWhenStepping) {
  DroneParams bad;
  bad.max_speed = 0.0;
  const QuadVehicle vehicle(bad);
  EXPECT_THROW(vehicle.step(DroneState{}, DroneCommand{}, 0.01), std::invalid_argument);
}
