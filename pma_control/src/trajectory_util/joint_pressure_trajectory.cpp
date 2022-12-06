#include "pma_control/trajectory_util/joint_pressure_trajectory.hpp"

#include <cmath>
#include <Eigen/Core>

#include "pma_hardware/description/pneumatic_muscle_linkage_telemetry.hpp"
#include "pma_hardware/math_util/compute_linkage_torques.hpp"

#define PMA_PI (M_PI)
#define PMA_HPI (PMA_PI/2.0)

namespace
{
    pma_control::JointPressureTrajectory2DofPressure compute_desired_input_pressures_for(
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
JointPressureTrajectoryPoint::JointPressureTrajectoryPoint(
    const trajectory_msgs::msg::JointTrajectoryPoint& jstp,
    const JointPressureTrajectory2DofPressure& pp
) :
    joint_space_trajectory_point(jstp),
    pressure_point(pp)
{
}

JointPressureTrajectoryPoint::JointPressureTrajectoryPoint() :
    JointPressureTrajectoryPoint(trajectory_msgs::msg::JointTrajectoryPoint(), {})
{
}

JointPressureTrajectory::JointPressureTrajectory(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> jstp,
    const JointPressureTrajectory2DofPressureList& pp
) :
    joint_space_trajectory_ptr(jstp),
    pressure_points(pp)
{
}

JointPressureTrajectory::JointPressureTrajectory() :
    JointPressureTrajectory(nullptr, {})
{
}

bool JointPressureTrajectory::point_at(const size_t index, JointPressureTrajectoryPoint& point) const
{
    const bool rc = (joint_space_trajectory_ptr.get() != nullptr)
        && (index < joint_space_trajectory_ptr->points.size())
        && (index < pressure_points.size());

    if (rc)
    {
        point = JointPressureTrajectoryPoint(
            joint_space_trajectory_ptr->points[index],
            pressure_points[index]
        );
    }
    return rc;
}

JointPressureTrajectory generate_from_joint_space_trajectory(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> jstp,
    const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
    const std::vector<pma_hardware::PneumaticMuscleLinkageConfiguration>& pm_linkage_configurations
)
{
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points = jstp->points;
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
    return JointPressureTrajectory(jstp, pressure_points);
}
}
