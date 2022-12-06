#pragma once

#include <vector>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace pma_control
{
/**
 * Define the precision for a pressure value.
 */
using Pressure = double;
/**
 * Define a type for a list of pressures.
 */
using PressureList = std::vector<Pressure>;

/**
 * A point in a joint-space trajectory that also has a corresponding pressure.
 */
struct PneumaticMuscleActuatorTrajectoryPoint
{
public:
    /**
     * The trajectory point in joint-space.
     */
    trajectory_msgs::msg::JointTrajectoryPoint joint_space;

    /**
     * The trajectory point in pressure-space.
     */
    PressureList pressures;

    /**
     * Constructor, fully initializes the trajectory point.
     */
    PneumaticMuscleActuatorTrajectoryPoint(
        const trajectory_msgs::msg::JointTrajectoryPoint& js,
        const PressureList& p
    ) :
        joint_space(js),
        pressures(p)
    {
    }

    /**
     * Constructor, for an empty trajectory point.
     */
    PneumaticMuscleActuatorTrajectoryPoint() :
        PneumaticMuscleActuatorTrajectoryPoint(
            trajectory_msgs::msg::JointTrajectoryPoint(),
            {}
        )
    {
    }
};
}
