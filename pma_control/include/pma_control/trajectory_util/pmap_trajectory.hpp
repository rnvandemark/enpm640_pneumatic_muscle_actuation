// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "joint_trajectory_controller/trajectory.hpp"

#include "pma_control/trajectory_util/joint_pressure_trajectory.hpp"

namespace pma_control
{
using JointPressureTrajectoryPointIter = std::vector<JointPressureTrajectoryPoint>::iterator;
using JointPressureTrajectoryPointConstIter = std::vector<JointPressureTrajectoryPoint>::const_iterator;

class PmapTrajectory
{
public:
    PmapTrajectory();

    explicit PmapTrajectory(
        const rclcpp::Time& current_time,
        const trajectory_msgs::msg::JointTrajectoryPoint& current_point,
        std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory
    );

    /// Set the point before the trajectory message is replaced/appended
    /// Example: if we receive a new trajectory message and it's first point is 0.5 seconds
    /// from the current one, we call this function to log the current state, then
    /// append/replace the current trajectory
    void set_point_before_trajectory_msg(
        const rclcpp::Time& current_time,
        const trajectory_msgs::msg::JointTrajectoryPoint& current_point,
        const double current_pressure
    );

    void update(
        std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory,
        const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
        const std::vector<pma_hardware::PneumaticMuscleLinkageConfiguration>& pm_linkage_configurations
    );

//    /// Find the segment (made up of 2 points) and its expected state from the
//    /// containing trajectory.
//    /**
//     * Sampling trajectory at given \p sample_time.
//     * If position in the \p end_segment_itr is missing it will be deduced from provided velocity, or acceleration respectively.
//     * Deduction assumes that the provided velocity or acceleration have to be reached at the time defined in the segment.
//     *
//     * Specific case returns for start_segment_itr and end_segment_itr:
//     * - Sampling before the trajectory start:
//     *     start_segment_itr = begin(), end_segment_itr = begin()
//     * - Sampling exactly on a point of the trajectory:
//     *        start_segment_itr = iterator where point is, end_segment_itr = iterator after start_segment_itr
//     * - Sampling between points:
//     *        start_segment_itr = iterator before the sampled point, end_segment_itr = iterator after start_segment_itr
//     * - Sampling after entire trajectory:
//     *        start_segment_itr = --end(), end_segment_itr = end()
//     * - Sampling empty msg or before the time given in set_point_before_trajectory_msg()
//     *        return false
//     *
//     * \param[in] sample_time Time at which trajectory will be sampled.
//     * \param[in] interpolation_method Specify whether splines, another method, or no interpolation at all.
//     * \param[out] expected_state Calculated new at \p sample_time.
//     * \param[out] start_segment_itr Iterator to the start segment for given \p sample_time. See description above.
//     * \param[out] end_segment_itr Iterator to the end segment for given \p sample_time. See description above.
//     */
//    bool sample(
//        const rclcpp::Time& sample_time,
//        const joint_trajectory_controller::interpolation_methods::InterpolationMethod interpolation_method,
//        trajectory_msgs::msg::JointTrajectoryPoint& output_state,
//        JointPressureTrajectoryPointConstIter& start_segment_itr,
//        JointPressureTrajectoryPointConstIter& end_segment_itr
//    );
//
//    /**
//     * Do interpolation between 2 states given a time in between their respective timestamps
//     *
//     * The start and end states need not necessarily be specified all the way to the acceleration level:
//     * - If only \b positions are specified, linear interpolation will be used.
//     * - If \b positions and \b velocities are specified, a cubic spline will be used.
//     * - If \b positions, \b velocities and \b accelerations are specified, a quintic spline will be used.
//     *
//     * If start and end states have different specifications
//     * (eg. start is position-only, end is position-velocity), the lowest common specification will be used
//     * (position-only in the example).
//     *
//     * \param[in] time_a Time at which the segment state equals \p state_a.
//     * \param[in] state_a State at \p time_a.
//     * \param[in] time_b Time at which the segment state equals \p state_b.
//     * \param[in] state_b State at time \p time_b.
//     * \param[in] sample_time The time to sample, between time_a and time_b.
//     * \param[out] output The state at \p sample_time.
//     */
//    void interpolate_between_points(
//        const rclcpp::Time& time_a,
//        const trajectory_msgs::msg::JointTrajectoryPoint& state_a,
//        const rclcpp::Time& time_b,
//        const trajectory_msgs::msg::JointTrajectoryPoint& state_b,
//        const rclcpp::Time& sample_time,
//        trajectory_msgs::msg::JointTrajectoryPoint& output
//    );
//
//    JointPressureTrajectoryPointConstIter begin() const;
//
//    JointPressureTrajectoryPointConstIter end() const;

    rclcpp::Time time_from_start() const;

//    bool has_trajectory_msg() const;
//
//    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> get_trajectory_msg() const;

    rclcpp::Time get_trajectory_start_time() const;

    bool is_sampled_already() const;

protected:
    joint_trajectory_controller::Trajectory joint_trajectory_controller_trajectory;

    std::shared_ptr<JointPressureTrajectory> pressure_trajectory_msg_;

    double pressure_before_traj_msg_;

//    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
//    rclcpp::Time trajectory_start_time_;
//
//    rclcpp::Time time_before_traj_msg_;
//    trajectory_msgs::msg::JointTrajectoryPoint state_before_traj_msg_;
//
//    bool sampled_already_ = false;

    /// Set the point before the trajectory message is replaced/appended
    /// Example: if we receive a new trajectory message and it's first point is 0.5 seconds
    /// from the current one, we call this function to log the current state, then
    /// append/replace the current trajectory
    void set_pressure_before_trajectory_msg(const double current_pressure);

    /// Set the point before the trajectory message is replaced/appended
    /// Example: if we receive a new trajectory message and it's first point is 0.5 seconds
    /// from the current one, we call this function to log the current state, then
    /// append/replace the current trajectory
    void update_pressure(
        const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
        const std::vector<pma_hardware::PneumaticMuscleLinkageConfiguration>& pm_linkage_configurations
    );
};
}
