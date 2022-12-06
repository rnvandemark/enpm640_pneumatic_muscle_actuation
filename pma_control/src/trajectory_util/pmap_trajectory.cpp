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

#include "pma_control/trajectory_util/joint_pressure_trajectory.hpp"

#include <cmath>
#include <Eigen/Core>

#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"

#include "pma_hardware/description/pneumatic_muscle_linkage_telemetry.hpp"
#include "pma_hardware/math_util/compute_linkage_torques.hpp"

#define PMA_PI (M_PI)
#define PMA_HPI (PMA_PI/2.0)

namespace
{
    JointPressureTrajectory2DofPressure compute_desired_input_pressures_for(
        const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
        const std::vector<pma_hardware::PneumaticMuscleLinkageConfiguration>& pm_linkage_configurations,
        const std::vector<pma_hardware::PneumaticMuscleLinkageTelemetry>& pm_linkage_telemetries,
        const std::vector<double>& desired_joint_positions,
        const std::vector<double>& desired_joint_velocities,
        const std::vector<double>& actual_joint_positions,
        const std::vector<double>& actual_joint_velocities
    )
    {
        const pma_hardware::PneumaticMuscleLinkageConfiguration& shoulder_linkage_configuration = pm_linkage_configurations[0];
        const pma_hardware::PneumaticMuscleLinkageConfiguration& elbow_linkage_configuration = pm_linkage_configurations[1];
        const pma_hardware::PneumaticMuscleLinkageConfiguration& shoulder_linkage_telemetry = pm_linkage_telemetries[0];
        const pma_hardware::PneumaticMuscleLinkageConfiguration& elbow_linkage_telemetry = pm_linkage_telemetries[1];
        double shoulder_torque_nominal_pressure;
        double shoulder_torque_factor_input_pressure;
        double elbow_torque_nominal_pressure;
        double elbow_torque_factor_input_pressure;

        pma_hardware::compute_shoulder_linkage_torque_components(
            shoulder_linkage_configuration,
            shoulder_linkage_telemetry,
            shoulder_torque_nominal_pressure,
            shoulder_torque_factor_input_pressure
        );
        pma_hardware::compute_elbow_linkage_torque_components(
            elbow_linkage_configuration,
            elbow_linkage_telemetry,
            elbow_torque_nominal_pressure,
            elbow_torque_factor_input_pressure
        );

#define DECLARE_LINKAGE_VARS(config, num) \
        const double m_##num = config.mass; \
        const double l_#num = config.link_length; \
        const double l_com_##num = config.link_com_length; \
        const double I_##num = m_##num * l_com_##num * l_com_##num; \
        const double th_##num = desired_joint_positions[num-1]; \
        const double th_dot_##num = desired_joint_velocities[num-1]; \

        // Declare variables for both linkages/joints
        DECLARE_LINKAGE_VARS(shoulder_linkage_configuration, 1);
        DECLARE_LINKAGE_VARS(elbow_linkage_configuration, 2);

#undef DECLARE_LINKAGE_VARS

        // Declare any others
        const double a_g = pm_control_configuration.a_g;
        const Eigen::Vector2f th {{th_1, th_2}};
        const Eigen::Vector2f th_dot {{th_dot_1, th_dot_2}};
        const double c_2 = std::cos(th_2);

        // Calculate the inverse of the D matrix
        const double d11 = (2 * I_1) + (2 * I_2) + (m_2 * ((l_1 * l_1) + (2 * l_1 * l_com_2 * c_2)));
        const double d12 = I_2 + (m_2 * l_1 * l_com_2 * c_2);
        const double d22 = 2 * I_2;
        const Eigen::Matrix2f D_inv = Eigen::Matrix2f({
            {d11, d12},
            {d12, d22}
        }).inverse();

        // Calculate the inverse of the G matrix
        const Eigen::Matrix2f G_inv = (D_inv * Eigen::Matrix2f({
            {shoulder_torque_factor_input_pressure, 0.0},
            {0.0,                                   elbow_torque_factor_input_pressure}
        })).inverse();

        // Calculate the a vector
        const double h = -m_2 * l_1 * l_com_2 * std::sin(th_2);
        const Eigen::Matrix2f C
        {
            {h * th_dot_2,  (h * th_dot_1) + (h * th_dot_2)},
            {-h * th_dot_1, 0.0}
        };
        const double f_2 = m_2 * l_com_2 * a_g * std::cos(th_1 + th_2);
        const double f_1 = (((m_1 * l_com_1) + (m_2 * l_1)) * a_g * std::cos(th_1)) + f_2;
        const Eigen::Vector2f f {{f_1, f_2}};
        const Eigen::Vector2f tau_0 {{shoulder_torque_nominal_pressure, elbow_torque_nominal_pressure}};
        const Eigen::Vector2f a = D_inv * ((C * th_dot * -1) - f + tau_0);

        // Calculate the ideal input pressures
        const Eigen::Vector2f delta_p = G_inv * (a * -1);

        // Finished, return the input pressures as a pair
        return std::make_pair<double, double>(delta_p(0), delta_p(1));
    }
}

namespace pma_control
{
PmapTrajectory::PmapTrajectory()
{
}

PmapTrajectory::PmapTrajectory(
    const rclcpp::Time& current_time,
    const trajectory_msgs::msg::JointTrajectoryPoint& current_point,
    const double current_pressure,
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory,
    const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
    const std::vector<pma_hardware::PneumaticMuscleLinkageConfiguration>& pm_linkage_configurations
) :
    joint_trajectory_controller_trajectory(current_time, current_point, joint_trajectory)
{
    set_pressure_before_trajectory_msg(current_pressure);
    update_pressure(pm_control_configuration, pm_linkage_configurations);
}

void PmapTrajectory::set_point_before_trajectory_msg(
    const rclcpp::Time& current_time,
    const trajectory_msgs::msg::JointTrajectoryPoint& current_point,
    const double current_pressure
)
{
    joint_trajectory_controller_trajectory.set_point_before_trajectory_msg(current_time, current_point);
    set_pressure_before_trajectory_msg(current_pressure);
}

void PmapTrajectory::update(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory,
    const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
    const std::vector<pma_hardware::PneumaticMuscleLinkageConfiguration>& pm_linkage_configurations
)
{
    joint_trajectory_controller_trajectory.update(joint_trajectory);
    update_pressure(pm_control_configuration, pm_linkage_configurations);
}

//bool Trajectory::sample(
//    const rclcpp::Time& sample_time,
//    const joint_trajectory_controller::interpolation_methods::InterpolationMethod interpolation_method,
//    trajectory_msgs::msg::JointTrajectoryPoint& output_state,
//    JointPressureTrajectoryPointConstIter& start_segment_itr,
//    JointPressureTrajectoryPointConstIter& end_segment_itr)
//{
//    THROW_ON_NULLPTR(trajectory_msg_)
//    output_state = trajectory_msgs::msg::JointTrajectoryPoint();
//
//    if (trajectory_msg_->points.empty())
//    {
//        start_segment_itr = end();
//        end_segment_itr = end();
//        return false;
//    }
//
//    // first sampling of this trajectory
//    if (!sampled_already_)
//    {
//        if (trajectory_start_time_.seconds() == 0.0)
//        {
//            trajectory_start_time_ = sample_time;
//        }
//
//        sampled_already_ = true;
//    }
//
//    // sampling before the current point
//    if (sample_time < time_before_traj_msg_)
//    {
//        return false;
//    }
//
//    auto& first_point_in_msg = trajectory_msg_->points[0];
//    const rclcpp::Time first_point_timestamp =
//        trajectory_start_time_ + first_point_in_msg.time_from_start;
//
//    // current time hasn't reached traj time of the first point in the msg yet
//    if (sample_time < first_point_timestamp)
//    {
//        // If interpolation is disabled, just forward the next waypoint
//        if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
//        {
//            output_state = state_before_traj_msg_;
//        }
//        else
//        {
//            // it changes points only if position and velocity do not exist, but their derivatives
//            deduce_from_derivatives(
//                state_before_traj_msg_, first_point_in_msg, state_before_traj_msg_.positions.size(),
//                (first_point_timestamp - time_before_traj_msg_).seconds());
//
//            interpolate_between_points(
//                time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
//                sample_time, output_state);
//        }
//        start_segment_itr = begin();    // no segments before the first
//        end_segment_itr = begin();
//        return true;
//    }
//
//    // time_from_start + trajectory time is the expected arrival time of trajectory
//    const auto last_idx = trajectory_msg_->points.size() - 1;
//    for (size_t i = 0; i < last_idx; ++i)
//    {
//        auto& point = trajectory_msg_->points[i];
//        auto& next_point = trajectory_msg_->points[i + 1];
//
//        const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
//        const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;
//
//        if (sample_time >= t0 && sample_time < t1)
//        {
//            // If interpolation is disabled, just forward the next waypoint
//            if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
//            {
//                output_state = next_point;
//            }
//            // Do interpolation
//            else
//            {
//                // it changes points only if position and velocity do not exist, but their derivatives
//                deduce_from_derivatives(
//                    point, next_point, state_before_traj_msg_.positions.size(), (t1 - t0).seconds());
//
//                interpolate_between_points(t0, point, t1, next_point, sample_time, output_state);
//            }
//            start_segment_itr = begin() + i;
//            end_segment_itr = begin() + (i + 1);
//            return true;
//        }
//    }
//
//    // whole animation has played out
//    start_segment_itr = --end();
//    end_segment_itr = end();
//    output_state = (*start_segment_itr);
//    // the trajectories in msg may have empty velocities/accel, so resize them
//    if (output_state.velocities.empty())
//    {
//        output_state.velocities.resize(output_state.positions.size(), 0.0);
//    }
//    if (output_state.accelerations.empty())
//    {
//        output_state.accelerations.resize(output_state.positions.size(), 0.0);
//    }
//    return true;
//}
//
//void Trajectory::interpolate_between_points(
//    const rclcpp::Time& time_a, const trajectory_msgs::msg::JointTrajectoryPoint& state_a,
//    const rclcpp::Time& time_b, const trajectory_msgs::msg::JointTrajectoryPoint& state_b,
//    const rclcpp::Time& sample_time, trajectory_msgs::msg::JointTrajectoryPoint& output)
//{
//    rclcpp::Duration duration_so_far = sample_time - time_a;
//    rclcpp::Duration duration_btwn_points = time_b - time_a;
//
//    const size_t dim = state_a.positions.size();
//    output.positions.resize(dim, 0.0);
//    output.velocities.resize(dim, 0.0);
//    output.accelerations.resize(dim, 0.0);
//
//    auto generate_powers = [](int n, double x, double * powers)
//    {
//        powers[0] = 1.0;
//        for (int i = 1; i <= n; ++i)
//        {
//            powers[i] = powers[i - 1] * x;
//        }
//    };
//
//    bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
//    bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();
//    if (duration_so_far.seconds() < 0.0)
//    {
//        duration_so_far = rclcpp::Duration::from_seconds(0.0);
//        has_velocity = has_accel = false;
//    }
//    if (duration_so_far.seconds() > duration_btwn_points.seconds())
//    {
//        duration_so_far = duration_btwn_points;
//        has_velocity = has_accel = false;
//    }
//
//    double t[6];
//    generate_powers(5, duration_so_far.seconds(), t);
//
//    if (!has_velocity && !has_accel)
//    {
//        // do linear interpolation
//        for (size_t i = 0; i < dim; ++i)
//        {
//            double start_pos = state_a.positions[i];
//            double end_pos = state_b.positions[i];
//
//            double coefficients[2] = {0.0, 0.0};
//            coefficients[0] = start_pos;
//            if (duration_btwn_points.seconds() != 0.0)
//            {
//                coefficients[1] = (end_pos - start_pos) / duration_btwn_points.seconds();
//            }
//
//            output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
//            output.velocities[i] = t[0] * coefficients[1];
//        }
//    }
//    else if (has_velocity && !has_accel)
//    {
//        // do cubic interpolation
//        double T[4];
//        generate_powers(3, duration_btwn_points.seconds(), T);
//
//        for (size_t i = 0; i < dim; ++i)
//        {
//            double start_pos = state_a.positions[i];
//            double start_vel = state_a.velocities[i];
//            double end_pos = state_b.positions[i];
//            double end_vel = state_b.velocities[i];
//
//            double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
//            coefficients[0] = start_pos;
//            coefficients[1] = start_vel;
//            if (duration_btwn_points.seconds() != 0.0)
//            {
//                coefficients[2] =
//                    (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
//                coefficients[3] =
//                    (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
//            }
//
//            output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
//                                                        t[2] * coefficients[2] + t[3] * coefficients[3];
//            output.velocities[i] =
//                t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] + t[2] * 3.0 * coefficients[3];
//            output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3];
//        }
//    }
//    else if (has_velocity && has_accel)
//    {
//        // do quintic interpolation
//        double T[6];
//        generate_powers(5, duration_btwn_points.seconds(), T);
//
//        for (size_t i = 0; i < dim; ++i)
//        {
//            double start_pos = state_a.positions[i];
//            double start_vel = state_a.velocities[i];
//            double start_acc = state_a.accelerations[i];
//            double end_pos = state_b.positions[i];
//            double end_vel = state_b.velocities[i];
//            double end_acc = state_b.accelerations[i];
//
//            double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//            coefficients[0] = start_pos;
//            coefficients[1] = start_vel;
//            coefficients[2] = 0.5 * start_acc;
//            if (duration_btwn_points.seconds() != 0.0)
//            {
//                coefficients[3] = (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] +
//                                                     end_acc * T[2] - 12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) /
//                                                    (2.0 * T[3]);
//                coefficients[4] = (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] -
//                                                     2.0 * end_acc * T[2] + 16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) /
//                                                    (2.0 * T[4]);
//                coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] -
//                                                     6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) /
//                                                    (2.0 * T[5]);
//            }
//
//            output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
//                                                        t[2] * coefficients[2] + t[3] * coefficients[3] +
//                                                        t[4] * coefficients[4] + t[5] * coefficients[5];
//            output.velocities[i] = t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
//                                                         t[2] * 3.0 * coefficients[3] + t[3] * 4.0 * coefficients[4] +
//                                                         t[4] * 5.0 * coefficients[5];
//            output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3] +
//                                                                t[2] * 12.0 * coefficients[4] + t[3] * 20.0 * coefficients[5];
//        }
//    }
//}
//
//void Trajectory::deduce_from_derivatives(
//    trajectory_msgs::msg::JointTrajectoryPoint& first_state,
//    trajectory_msgs::msg::JointTrajectoryPoint& second_state, const size_t dim, const double delta_t)
//{
//    if (second_state.positions.empty())
//    {
//        second_state.positions.resize(dim);
//        if (first_state.velocities.empty())
//        {
//            first_state.velocities.resize(dim, 0.0);
//        }
//        if (second_state.velocities.empty())
//        {
//            second_state.velocities.resize(dim);
//            if (first_state.accelerations.empty())
//            {
//                first_state.accelerations.resize(dim, 0.0);
//            }
//            for (size_t i = 0; i < dim; ++i)
//            {
//                second_state.velocities[i] =
//                    first_state.velocities[i] +
//                    (first_state.accelerations[i] + second_state.accelerations[i]) * 0.5 * delta_t;
//            }
//        }
//        for (size_t i = 0; i < dim; ++i)
//        {
//            // second state velocity should be reached on the end of the segment, so use middle
//            second_state.positions[i] =
//                first_state.positions[i] +
//                (first_state.velocities[i] + second_state.velocities[i]) * 0.5 * delta_t;
//        }
//    }
//}
//
//JointPressureTrajectoryPointConstIter Trajectory::begin() const
//{
//    THROW_ON_NULLPTR(trajectory_msg_)
//
//    return trajectory_msg_->points.begin();
//}
//
//JointPressureTrajectoryPointConstIter Trajectory::end() const
//{
//    THROW_ON_NULLPTR(trajectory_msg_)
//
//    return trajectory_msg_->points.end();
//}

rclcpp::Time PmapTrajectory::time_from_start() const
{
    return joint_trajectory_controller_trajectory.time_from_start();
}

//bool Trajectory::has_trajectory_msg() const
//{
//    return trajectory_msg_.get() != nullptr;
//}
//
//std::shared_ptr<trajectory_msgs::msg::JointTrajectory> Trajectory::get_trajectory_msg() const
//{
//    return trajectory_msg_;
//}

rclcpp::Time PmapTrajectory::get_trajectory_start_time() const
{
    return joint_trajectory_controller_trajectory.get_trajectory_start_time();
}

bool PmapTrajectory::is_sampled_already() const
{
    return joint_trajectory_controller_trajectory.is_sampled_already();
}

void PmapTrajectory::set_pressure_before_trajectory_msg(const double current_pressure)
{
    pressure_before_traj_msg_ = current_pressure;
}

void PmapTrajectory::update_pressure(
    const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
    const std::vector<pma_hardware::PneumaticMuscleLinkageConfiguration>& pm_linkage_configurations
)
{
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_space_trajectory_ptr = joint_trajectory_controller_trajectory.get_trajectory_msg();
    THROW_ON_NULLPTR(joint_space_trajectory_ptr)

    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points = joint_space_trajectory_ptr->points;
    JointPressureTrajectory2DofPressureList pressure_points;
    pressure_points.reserve(points.size());

    std::vector<pma_hardware::PneumaticMuscleLinkageTelemetry> desired_pm_linkage_telemetries(pm_linkage_configurations.size());

    // For every point in the joint-space trajectory, calculate the required
    // pressure pair for each joint
    for (auto iter = points.cbegin(); iter != points.cend(); ++iter)
    {
        // Capture the ideal position values from the desired trajectory
        const std::vector<double>& desired_joint_positions = iter->positions;
        const std::vector<double>& desired_joint_velocities = iter->velocities;
        const std::vector<double>& actual_joint_positions = desired_joint_positions;
        const std::vector<double>& actual_joint_velocities = desired_joint_velocities;

        // Given the desired position and velocity, first update desired state
        // for the shoulder
        double desired_position = desired_joint_positions[0];
        double desired_velocity = desired_joint_velocities[0];
        double pulley_radius = pm_linkage_configurations[0].pulley_radius;
        desired_pm_linkage_telemetries[0].bicep_telemetry.length = pulley_radius * (PMA_HPI - desired_position);
        desired_pm_linkage_telemetries[0].bicep_telemetry.lengthening_rate = pulley_radius * desired_joint_velocity * -1;
        desired_pm_linkage_telemetries[0].tricep_telemetry.length = pulley_radius * (desired_position + PMA_HPI);
        desired_pm_linkage_telemetries[0].tricep_telemetry.lengthening_rate = pulley_radius * desired_joint_velocity;

        // And now the elbow
        desired_position = desired_joint_positions[1];
        desired_velocity = desired_joint_velocities[1];
        pulley_radius = pm_linkage_configurations[1].pulley_radius;
        desired_pm_linkage_telemetries[1].bicep_telemetry.length = pulley_radius * desired_position;
        desired_pm_linkage_telemetries[1].bicep_telemetry.lengthening_rate = pulley_radius * desired_joint_velocity;
        desired_pm_linkage_telemetries[1].tricep_telemetry.length = pulley_radius * (PMA_PI - desired_position);
        desired_pm_linkage_telemetries[1].tricep_telemetry.lengthening_rate = pulley_radius * desired_joint_velocity * -1;

        // Calculate the desired input pressure pair here, and push it into the
        // list of pressure points that is parallel to the joint trajectory
        // points
        pressure_points.push_back(
            compute_desired_input_pressures_for(
                pm_control_configuration,
                pm_linkage_configurations,
                desired_pm_linkage_telemetries,
                desired_joint_positions,
                desired_joint_velocities,
                actual_joint_positions,
                actual_joint_velocities
            )
        );
    }

    // We have calculated the overall trajectory in pressure-space
    pressure_trajectory_msg_.joint_space_trajectory_ptr = joint_space_trajectory_ptr;
    pressure_trajectory_msg_.pressure_points = pressure_points;
}
}
