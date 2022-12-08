#include "pma_control/trajectory_control/pmap_trajectory_controller.hpp"

#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>

#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pma_hardware/description/pneumatic_muscle_control_configuration.hpp"
#include "pma_hardware/description/pneumatic_muscle_segment_configuration.hpp"
#include "pma_hardware/description/pneumatic_muscle_segment_telemetry.hpp"
#include "pma_util/util.hpp"
#include "rclcpp_action/create_server.hpp"

static const size_t REQUIRED_DOF = 2;

static const double PMA_PI = M_PI;
static const double PMA_HPI = PMA_PI / 2.0;

const static pma_hardware::PneumaticMuscleControlConfiguration DEFAULT_PMA_CONTROL_CONFIGURATION(
    /*a_g = */ 9.81,
    /*k_1 = */ 1.0,
    /*k_2 = */ 1.0,
    /*Gamma_1 = */ 1.0,
    /*Gamma_2 = */ 1.0,
    /*mu_1 = */ 1.0,
    /*mu_2 = */ 1.0
);

const static pma_hardware::PneumaticMuscleSegmentConfiguration DEFAULT_SHOULDER_SEGMENT_CONFIGURATION(
    /*mass = */ 1.0,
    /*pulley_radius = */ 0.075,
    /*segment_length = */ 0.5,
    /*segment_com_length = */ 0.25,
    /*pm_antagonist_pair_count = */ 2,
    /*bicep__nominal_pressure = */ 350.0,
    /*bicep__dynamics_f_0 = */ 1.0,
    /*bicep__dynamics_f_1 = */ 1.0,
    /*bicep__dynamics_k_0 = */ 1.0,
    /*bicep__dynamics_k_1 = */ 1.0,
    /*bicep__dynamics_b_0_inflating = */ 1.0,
    /*bicep__dynamics_b_1_inflating = */ 1.0,
    /*bicep__dynamics_b_0_deflating = */ 1.0,
    /*bicep__dynamics_b_1_deflating = */ 1.0,
    /*tricep__nominal_pressure = */ 350.0,
    /*tricep__dynamics_f_0 = */ 1.0,
    /*tricep__dynamics_f_1 = */ 1.0,
    /*tricep__dynamics_k_0 = */ 1.0,
    /*tricep__dynamics_k_1 = */ 1.0,
    /*tricep__dynamics_b_0_inflating = */ 1.0,
    /*tricep__dynamics_b_1_inflating = */ 1.0,
    /*tricep__dynamics_b_0_deflating = */ 1.0,
    /*tricep__dynamics_b_1_deflating = */ 1.0
);
const static pma_hardware::PneumaticMuscleSegmentConfiguration DEFAULT_ELBOW_SEGMENT_CONFIGURATION(
    /*mass = */ 1.0,
    /*pulley_radius = */ 0.075,
    /*segment_length = */ 0.5,
    /*segment_com_length = */ 0.25,
    /*pm_antagonist_pair_count = */ 2,
    /*bicep__nominal_pressure = */ 350.0,
    /*bicep__dynamics_f_0 = */ 1.0,
    /*bicep__dynamics_f_1 = */ 1.0,
    /*bicep__dynamics_k_0 = */ 1.0,
    /*bicep__dynamics_k_1 = */ 1.0,
    /*bicep__dynamics_b_0_inflating = */ 1.0,
    /*bicep__dynamics_b_1_inflating = */ 1.0,
    /*bicep__dynamics_b_0_deflating = */ 1.0,
    /*bicep__dynamics_b_1_deflating = */ 1.0,
    /*tricep__nominal_pressure = */ 350.0,
    /*tricep__dynamics_f_0 = */ 1.0,
    /*tricep__dynamics_f_1 = */ 1.0,
    /*tricep__dynamics_k_0 = */ 1.0,
    /*tricep__dynamics_k_1 = */ 1.0,
    /*tricep__dynamics_b_0_inflating = */ 1.0,
    /*tricep__dynamics_b_1_inflating = */ 1.0,
    /*tricep__dynamics_b_0_deflating = */ 1.0,
    /*tricep__dynamics_b_1_deflating = */ 1.0
);

using namespace std::chrono_literals;

namespace im = joint_trajectory_controller::interpolation_methods;

namespace
{
    rclcpp::Duration duration_from_rate(const double rate)
    {
        return rclcpp::Duration::from_seconds((rate > 0.0) ? (1.0 / rate) : 0.0);
    }

    void resize_joint_trajectory_point(
        trajectory_msgs::msg::JointTrajectory& point,
        const size_t size
    )
    {
        point.positions.resize(size, 0.0);
        point.velocities.resize(size, 0.0);
        point.accelerations.resize(size, 0.0);
        point.effort.resize(size, 0.0);
    }

    double sat(const double y)
    {
        if (std::abs(y) <= 1)
        {
            return y;
        }
        else if (y > 0.0)
        {
            return +1;
        }
        else
        {
            return -1;
        }
    }

    void compute_individual_force_contribution(
        const pma_hardware::IndividualPneumaticMuscleConfiguration& pm_configuration,
        const pma_hardware::IndividualPneumaticMuscleTelemetry& pm_telemetry,
        double& force_nominal_pressure,
        double& force_factor_input_pressure
    )
    {
        const double length = pm_telemetry.length;
        const double lengthening_rate = pm_telemetry.lengthening_rate;
        const bool is_inflating = pm_telemetry.is_inflating();
        const double P0 = pm_configuration.nominal_pressure;
        const double F0 = pm_configuration.dynamics_F_0;
        const double F1 = pm_configuration.dynamics_F_1;
        const double K0 = pm_configuration.dynamics_K_0;
        const double K1 = pm_configuration.dynamics_K_1;
        const double B0 = is_inflating ? pm_configuration.dynamics_B_0_inflating : pm_configuration.dynamics_B_0_deflating;
        const double B1 = is_inflating ? pm_configuration.dynamics_B_1_inflating : pm_configuration.dynamics_B_1_deflating;

        force_nominal_pressure = (F0 + (F1 * P0)) - (length * (K0 + (K1 * P0))) - (lengthening_rate * (B0 + (B1 * P0)));
        force_factor_input_pressure = F1 - (length * K1) - (lengthening_rate * B1);
    }

    // Requires ros params:
    // - From (5.10):
    //  - k_1
    //  - k_2
    //  - Gamma_1
    //  - Gamma_2
    //  - From (3.9)
    //   - From (A2)
    //    - m_1
    //    - m_2
    //    - l_1
    //    - l_c_1 (notably, does not need l_c_2)
    //   - From (A4): None
    //   - From (A5):
    //    - g
    //   - From (3.7a): (BEWARE! POSSIBLE TYPO??)
    //    - n_s
    //    - r_s
    //    - F_0_s
    //    - F_1_s
    //    - P_0_b_s
    //    - P_0_t_s
    //    - K_0_s
    //    - K_1_s
    //    - B_0_b_s
    //    - B_0_t_s
    //    - B_1_b_s
    //    - B_1_t_s
    //   - From (3.7c): same as (3.7a), but replace 's' (shoulder) subscript with 'e' (elbow)
    //  - From (5.3):
    //   - mu_1
    //   - mu_2
    //  - From (3.10):
    //   - From (3.7b): None
    //   - From (3.7d): same as (3.7b), but replace 's' (shoulder) subscript with 'e' (elbow)
    // Requires inputs:
    // - From (A2):
    //  - actual joint positions
    // - From (A4):
    //  - actual joint velocities
    // - From (3.7a):
    //  - x_t_s
    //  - x_dot_t_s
    //  - x_b_s
    //  - x_dot_b_s
    // - From (3.7c): same as (3.7a), but replace 's' (shoulder) subscript with 'e' (elbow)
    // - From (5.3):
    //  - desired joint positions
    // - From (5.4):
    //  - desired joint velocities
    void compute_generic_segment_torque_components(
        const pma_hardware::PneumaticMuscleSegmentConfiguration& segment_configuration,
        const pma_hardware::PneumaticMuscleSegmentTelemetry& segment_telemetry,
        const bool bicep_is_positive_contribution,
        double& torque_nominal_pressure,
        double& torque_factor_input_pressure
    )
    {
        const pma_hardware::IndividualPneumaticMuscleConfiguration* pm_configuration_positive_contribution_ptr = nullptr;
        const pma_hardware::IndividualPneumaticMuscleTelemetry* pm_telemetry_positive_contribution_ptr = nullptr;
        const pma_hardware::IndividualPneumaticMuscleConfiguration* pm_configuration_negative_contribution_ptr = nullptr;
        const pma_hardware::IndividualPneumaticMuscleTelemetry* pm_telemetry_negative_contribution_ptr = nullptr;

        if (bicep_is_positive_contribution)
        {
            pm_configuration_positive_contribution_ptr = &(segment_configuration.bicep_configuration);
            pm_telemetry_positive_contribution_ptr = &(segment_telemetry.bicep_telemetry);
            pm_configuration_negative_contribution_ptr = &(segment_configuration.tricep_configuration);
            pm_telemetry_negative_contribution_ptr = &(segment_telemetry.tricep_telemetry);
        }
        else
        {
            pm_configuration_positive_contribution_ptr = &(segment_configuration.tricep_configuration);
            pm_telemetry_positive_contribution_ptr = &(segment_telemetry.tricep_telemetry);
            pm_configuration_negative_contribution_ptr = &(segment_configuration.bicep_configuration);
            pm_telemetry_negative_contribution_ptr = &(segment_telemetry.bicep_telemetry);
        }

        double force_nominal_pressure_positive_contribution;
        double force_factor_input_pressure_positive_contribution;
        double force_nominal_pressure_negative_contribution;
        double force_factor_input_pressure_negative_contribution;

        compute_individual_force_contribution(
            *pm_configuration_positive_contribution_ptr,
            *pm_telemetry_positive_contribution_ptr,
            force_nominal_pressure_positive_contribution,
            force_factor_input_pressure_positive_contribution
        );
        compute_individual_force_contribution(
            *pm_configuration_negative_contribution_ptr,
            *pm_telemetry_negative_contribution_ptr,
            force_nominal_pressure_negative_contribution,
            force_factor_input_pressure_negative_contribution
        );

        const double effort_factor = segment_configuration.pm_antagonist_pair_count * segment_configuration.pulley_radius;
        torque_nominal_pressure = effort_factor * (force_nominal_pressure_positive_contribution - force_nominal_pressure_negative_contribution);
        torque_factor_input_pressure = effort_factor * (force_factor_input_pressure_positive_contribution - force_factor_input_pressure_negative_contribution);
    }

    void compute_pma_robot_model(
        const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
        const std::vector<pma_hardware::PneumaticMuscleSegmentConfiguration>& pm_segment_configurations,
        const std::vector<pma_hardware::PneumaticMuscleSegmentTelemetry>& pm_segment_telemetries,
        const std::vector<double>& desired_joint_positions,
        const std::vector<double>& desired_joint_velocities,
        const std::vector<double>& actual_joint_positions,
        const std::vector<double>& actual_joint_velocities,
        pma_util::PressureList& resulting_pressure_state,
        Eigen::Vector2d& model_a,
        Eigen::Matrix2d& model_G
    )
    {
        //
        // Declare variables
        //

        const pma_hardware::PneumaticMuscleSegmentConfiguration& shoulder_segment_configuration = pm_segment_configurations[0];
        const pma_hardware::PneumaticMuscleSegmentConfiguration& elbow_segment_configuration = pm_segment_configurations[1];
        const pma_hardware::PneumaticMuscleSegmentTelemetry& shoulder_segment_telemetry = pm_segment_telemetries[0];
        const pma_hardware::PneumaticMuscleSegmentTelemetry& elbow_segment_telemetry = pm_segment_telemetries[1];

#define DECLARE_SEGMENT_VARS(config, num) \
        const double m_##num = config.mass; \
        const double l_##num = config.segment_length; \
        const double l_com_##num = config.segment_com_length; \
        const double I_##num = m_##num * l_com_##num * l_com_##num

        DECLARE_SEGMENT_VARS(shoulder_segment_configuration, 1);
        DECLARE_SEGMENT_VARS(elbow_segment_configuration, 2);

#undef DECLARE_SEGMENT_VARS

#define DECLARE_JOINT_VARS(num) \
        const double th_star_##num = actual_joint_positions[num-1]; \
        const double th_dot_star_##num = actual_joint_velocities[num-1]; \
        const double th_##num = desired_joint_positions[num-1]; \
        const double th_dot_##num = desired_joint_velocities[num-1]

        DECLARE_JOINT_VARS(1);
        DECLARE_JOINT_VARS(2);

#undef DECLARE_JOINT_VARS

        //
        // Calculate the G matrix
        //

        // Calculate the torque that is generated on each joint given the system
        // configuration and the current state of each segment
        double shoulder_torque_nominal_pressure;
        double shoulder_torque_factor_input_pressure;
        double elbow_torque_nominal_pressure;
        double elbow_torque_factor_input_pressure;

        compute_generic_segment_torque_components(
            shoulder_segment_configuration,
            shoulder_segment_telemetry,
            false,
            shoulder_torque_nominal_pressure,
            shoulder_torque_factor_input_pressure
        );
        compute_generic_segment_torque_components(
            elbow_segment_configuration,
            elbow_segment_telemetry,
            true,
            elbow_torque_nominal_pressure,
            elbow_torque_factor_input_pressure
        );

        // Calculate the inverse of the D matrix
        const double c_2 = std::cos(th_2);
        const double d11 = (2 * I_1) + (2 * I_2) + (m_2 * ((l_1 * l_1) + (2 * l_1 * l_com_2 * c_2)));
        const double d12 = I_2 + (m_2 * l_1 * l_com_2 * c_2);
        const double d22 = 2 * I_2;
        const Eigen::Matrix2d D_inv = Eigen::Matrix2d({
            {d11, d12},
            {d12, d22}
        }).inverse();

        // Calculate the G matrix
        model_G = D_inv * Eigen::Matrix2d {{
            {shoulder_torque_factor_input_pressure, 0.0},
            {0.0,                                   elbow_torque_factor_input_pressure}
        }};

        //
        // Calculate the a vector
        //

        const Eigen::Vector2d th_dot {{th_dot_1, th_dot_2}};
        const double h = -m_2 * l_1 * l_com_2 * std::sin(th_2);
        const Eigen::Matrix2d C {
            {h * th_dot_2,  (h * th_dot_1) + (h * th_dot_2)},
            {-h * th_dot_1, 0.0}
        };
        const double f_2 = m_2 * l_com_2 * pm_control_configuration.a_g * std::cos(th_1 + th_2);
        const double f_1 = (((m_1 * l_com_1) + (m_2 * l_1)) * pm_control_configuration.a_g * std::cos(th_1)) + f_2;
        const Eigen::Vector2d f {{f_1, f_2}};
        const Eigen::Vector2d tau_0 {{shoulder_torque_nominal_pressure, elbow_torque_nominal_pressure}};
        model_a = D_inv * ((C * th_dot * -1) - f + tau_0);

        // Calculate the sliding manifold sigma
        const Eigen::Vector2d th_r
        {{
            th_dot_star_1 - (pm_control_configuration.mu_1 * (th_1 - th_star_1)),
            th_dot_star_2 - (pm_control_configuration.mu_2 * (th_2 - th_star_2))
        }};
        const double sigma_1 = th_dot_1 - th_r(0);
        const double sigma_2 = th_dot_2 - th_r(1);

        // Calculate the relavant boundary layer thickness vector
        const Eigen::Vector2d ks
        {{
            pm_control_configuration.k_1 * sat(sigma_1 / pm_control_configuration.Gamma_1),
            pm_control_configuration.k_2 * sat(sigma_2 / pm_control_configuration.Gamma_2)
        }};

        // Calculate the resulting input pressures
        const Eigen::Vector2d delta_p = G.inverse() * (th_r - a - ks);
        resulting_pressure_state = {delta_p(0), delta_p(1)};
    }
}

namespace pma_control
{
SlidingMode2DofPressureTrajectoryController::SlidingMode2DofPressureTrajectoryController() :
    controller_interface::ControllerInterface(),
    joint_names_({}),
    dof_(0),
    state_publish_rate_(20.0),
    state_publisher_period_(duration_from_rate(state_publish_rate_)),
    action_monitor_rate_(50.0),
    action_monitor_period_(duration_from_rate(action_monitor_rate_)),
    interpolation_method_(im::DEFAULT_INTERPOLATION)
    pm_control_configuration(nullptr)
{
}

controller_interface::InterfaceConfiguration
SlidingMode2DofPressureTrajectoryController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    if (dof_ == 0)
    {
        fprintf(
            stderr,
            "During ros2_control interface configuration, degrees of freedom is not valid;"
            " it should be positive. Actual DOF is %zu\n",
            dof_);
        std::exit(EXIT_FAILURE);
    }
    conf.names.reserve(dof_ * command_interface_types_.size());
    for (const auto& joint_name : joint_names_)
    {
        for (const auto& interface_type : command_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::InterfaceConfiguration
SlidingMode2DofPressureTrajectoryController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(dof_ * state_interface_types_.size());
    for (const auto& joint_name : joint_names_)
    {
        for (const auto& interface_type : state_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::CallbackReturn
SlidingMode2DofPressureTrajectoryController::on_init()
{
    fprintf(stdout, "Entered on_init()\n");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

#define AUTO_DEC1(var_name, param_name, param_type) var_name = auto_declare<param_type>(param_name, var_name)
#define AUTO_DEC2(var_desc, param_type) AUTO_DEC1(var_desc##_, #var_desc, param_type)
#define AUTO_DEC_CONFIG_PARAM(param_name, param_type, value) const param_type param_name = auto_declare<param_type>(#param_name, value)

    // Declare parameters while the lifecycle node is initializing
    try
    {
        // Base controller parameters
        AUTO_DEC1(joint_names_, "joints", std::vector<std::string>);
        AUTO_DEC2(state_publish_rate, double);
        AUTO_DEC2(action_monitor_rate, double);
        interpolation_method_ = im::from_string(auto_declare<std::string>(
            "interpolation_method",
            im::InterpolationMethodMap.at(interpolation_method_)
        ));

        // Parameters specific to sliding-mode control
        AUTO_DEC_CONFIG_PARAM(acceleration_gravity, double, DEFAULT_PMA_CONTROL_CONFIGURATION.a_g);
        AUTO_DEC_CONFIG_PARAM(sliding_mode_control_k_1, double, DEFAULT_PMA_CONTROL_CONFIGURATION.k_1);
        AUTO_DEC_CONFIG_PARAM(sliding_mode_control_k_2, double, DEFAULT_PMA_CONTROL_CONFIGURATION.k_2);
        AUTO_DEC_CONFIG_PARAM(sliding_mode_control_Gamma_1, double, DEFAULT_PMA_CONTROL_CONFIGURATION.Gamma_1);
        AUTO_DEC_CONFIG_PARAM(sliding_mode_control_Gamma_2, double, DEFAULT_PMA_CONTROL_CONFIGURATION.Gamma_2);
        AUTO_DEC_CONFIG_PARAM(sliding_mode_control_mu_1, double, DEFAULT_PMA_CONTROL_CONFIGURATION.mu_1);
        AUTO_DEC_CONFIG_PARAM(sliding_mode_control_mu_2, double, DEFAULT_PMA_CONTROL_CONFIGURATION.mu_2);

        // Parameters specific to robot properties
        AUTO_DEC_CONFIG_PARAM(shoulder_mass, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.mass);
        AUTO_DEC_CONFIG_PARAM(shoulder_pulley_radius, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.pulley_radius);
        AUTO_DEC_CONFIG_PARAM(shoulder_segment_length, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.segment_length);
        AUTO_DEC_CONFIG_PARAM(shoulder_segment_com_length, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.segment_com_length);
        AUTO_DEC_CONFIG_PARAM(shoulder_pm_antagonist_pair_count, int, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.pm_antagonist_pair_count);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_nominal_pressure, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.nominal_pressure);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_F_0, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_F_0);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_F_1, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_F_1);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_K_0, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_K_0);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_K_1, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_K_1);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_B_0_inflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_0_inflating);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_B_1_inflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_1_inflating);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_B_0_deflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_0_deflating);
        AUTO_DEC_CONFIG_PARAM(shoulder_bicep_dynamics_B_1_deflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_1_deflating);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_nominal_pressure, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.nominal_pressure);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_F_0, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_F_0);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_F_1, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_F_1);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_K_0, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_K_0);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_K_1, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_K_1);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_B_0_inflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_0_inflating);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_B_1_inflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_1_inflating);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_B_0_deflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_0_deflating);
        AUTO_DEC_CONFIG_PARAM(shoulder_tricep_dynamics_B_1_deflating, double, DEFAULT_SHOULDER_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_1_deflating);
        AUTO_DEC_CONFIG_PARAM(elbow_mass, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.mass);
        AUTO_DEC_CONFIG_PARAM(elbow_pulley_radius, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.pulley_radius);
        AUTO_DEC_CONFIG_PARAM(elbow_segment_length, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.segment_length);
        AUTO_DEC_CONFIG_PARAM(elbow_segment_com_length, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.segment_com_length);
        AUTO_DEC_CONFIG_PARAM(elbow_pm_antagonist_pair_count, int, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.pm_antagonist_pair_count);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_nominal_pressure, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.nominal_pressure);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_F_0, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_F_0);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_F_1, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_F_1);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_K_0, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_K_0);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_K_1, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_K_1);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_B_0_inflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_0_inflating);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_B_1_inflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_1_inflating);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_B_0_deflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_0_deflating);
        AUTO_DEC_CONFIG_PARAM(elbow_bicep_dynamics_B_1_deflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.bicep_configuration.dynamics_B_1_deflating);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_nominal_pressure, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.nominal_pressure);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_F_0, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_F_0);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_F_1, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_F_1);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_K_0, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_K_0);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_K_1, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_K_1);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_B_0_inflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_0_inflating);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_B_1_inflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_1_inflating);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_B_0_deflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_0_deflating);
        AUTO_DEC_CONFIG_PARAM(elbow_tricep_dynamics_B_1_deflating, double, DEFAULT_ELBOW_SEGMENT_CONFIGURATION.tricep_configuration.dynamics_B_1_deflating);

        // Create the configuration data
        pm_control_configuration.reset(new pma_hardware::PneumaticMuscleControlConfiguration(
            acceleration_gravity,
            sliding_mode_control_k_1,
            sliding_mode_control_k_2,
            sliding_mode_control_Gamma_1,
            sliding_mode_control_Gamma_2,
            sliding_mode_control_mu_1,
            sliding_mode_control_mu_2
        ));
        pm_segment_configurations.clear();
        pm_segment_configurations.push_back(pma_hardware::PneumaticMuscleSegmentConfiguration(
            shoulder_mass,
            shoulder_pulley_radius,
            shoulder_segment_length,
            shoulder_segment_com_length,
            static_cast<size_t>(shoulder_pm_antagonist_pair_count),
            shoulder_bicep_nominal_pressure,
            shoulder_bicep_dynamics_F_0,
            shoulder_bicep_dynamics_F_1,
            shoulder_bicep_dynamics_K_0,
            shoulder_bicep_dynamics_K_1,
            shoulder_bicep_dynamics_B_0_inflating,
            shoulder_bicep_dynamics_B_1_inflating,
            shoulder_bicep_dynamics_B_0_deflating,
            shoulder_bicep_dynamics_B_1_deflating,
            shoulder_tricep_nominal_pressure,
            shoulder_tricep_dynamics_F_0,
            shoulder_tricep_dynamics_F_1,
            shoulder_tricep_dynamics_K_0,
            shoulder_tricep_dynamics_K_1,
            shoulder_tricep_dynamics_B_0_inflating,
            shoulder_tricep_dynamics_B_1_inflating,
            shoulder_tricep_dynamics_B_0_deflating,
            shoulder_tricep_dynamics_B_1_deflating
        ));
        pm_segment_configurations.push_back(pma_hardware::PneumaticMuscleSegmentConfiguration(
            elbow_mass,
            elbow_pulley_radius,
            elbow_segment_length,
            elbow_segment_com_length,
            static_cast<size_t>(elbow_pm_antagonist_pair_count),
            elbow_bicep_nominal_pressure,
            elbow_bicep_dynamics_F_0,
            elbow_bicep_dynamics_F_1,
            elbow_bicep_dynamics_K_0,
            elbow_bicep_dynamics_K_1,
            elbow_bicep_dynamics_B_0_inflating,
            elbow_bicep_dynamics_B_1_inflating,
            elbow_bicep_dynamics_B_0_deflating,
            elbow_bicep_dynamics_B_1_deflating,
            elbow_tricep_nominal_pressure,
            elbow_tricep_dynamics_F_0,
            elbow_tricep_dynamics_F_1,
            elbow_tricep_dynamics_K_0,
            elbow_tricep_dynamics_K_1,
            elbow_tricep_dynamics_B_0_inflating,
            elbow_tricep_dynamics_B_1_inflating,
            elbow_tricep_dynamics_B_0_deflating,
            elbow_tricep_dynamics_B_1_deflating
        ));
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
        rc = CallbackReturn::ERROR;
        goto END;
    }

#undef AUTO_DEC_CONFIG_PARAM
#undef AUTO_DEC2
#undef AUTO_DEC1

END:
    fprintf(stdout, "on_init() rc=%s\n", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn
SlidingMode2DofPressureTrajectoryController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_configure()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    const auto logger = get_node()->get_logger();

    // First, attempt to reset
    if (!reset())
    {
        RCLCPP_ERROR(logger, "Failed to reset in on_configure().");
        rc = CallbackReturn::FAILURE;
        goto END;
    }

    //
    // Update parameters on configuration
    //

    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    state_publish_rate_ = get_node()->get_parameter("state_publish_rate").get_value<double>();
    RCLCPP_INFO(logger, "Controller state will be published at %.2f Hz.", state_publish_rate_);

    action_monitor_rate_ = get_node()->get_parameter("action_monitor_rate").get_value<double>();
    RCLCPP_INFO(logger, "Action status changes will be monitored at %.2f Hz.", action_monitor_rate_);

    interpolation_method_ = im::from_string(get_node()->get_parameter("interpolation_method").as_string());
    RCLCPP_INFO(
        logger,
        "Using '%s' interpolation method.",
        im::InterpolationMethodMap.at(interpolation_method_).c_str()
    );

    if ((dof_ > 0) && (joint_names_.size() != dof_))
    {
        RCLCPP_ERROR(
            logger,
            "The SlidingMode2DofPressureTrajectoryController does not support restarting with a different number of DOF."
        );
        rc = CallbackReturn::FAILURE;
        goto END;
    }

    if (joint_names_.empty())
    {
        RCLCPP_ERROR(logger, "The 'joints' parameter is empty.");
        rc = CallbackReturn::FAILURE;
        goto END;
    }

    dof_ = joint_names_.size();
    if (REQUIRED_DOF != dof_)
    {
        RCLCPP_ERROR(
            logger,
            "The SlidingMode2DofPressureTrajectoryController only supports %zu-DOF robots, but received %zu joint names.",
            REQUIRED_DOF,
            dof_
        );
        rc = CallbackReturn::FAILURE;
        goto END;
    }

    default_tolerances_ = joint_trajectory_controller::get_segment_tolerances(*get_node(), joint_names_);

    joint_command_subscriber_ = get_node()->create_subscription<JointTrajectoryMsg>(
        "~/joint_trajectory",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&SlidingMode2DofPressureTrajectoryController::topic_callback, this, std::placeholders::_1)
    );

    //
    // Configure command and state interfaces
    //

    // Initialize storage to avoid memory allocation during activation
    joint_command_interface_.resize(command_interface_types_.size());
    joint_state_interface_.resize(state_interface_types_.size());
    pm_segment_telemetries.assign(dof_, pma_hardware::PneumaticMuscleSegmentTelemetry());

    // Print interface configuration so users can be sure it's correct
    RCLCPP_INFO(
        logger,
        "Command interfaces are [%s] and state interfaces are [%s].",
        pma_util::get_delimited_list(command_interface_types_, ", ").c_str(),
        pma_util::get_delimited_list(state_interface_types_, ", ").c_str()
    );

    //
    // Handle updates to the state publisher
    //

    state_publisher_period_ = duration_from_rate(state_publish_rate_);

    publisher_ = get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<StatePublisher>(publisher_);
    state_publisher_->lock();
    state_publisher_->msg_.joint_names = joint_names_;
    state_publisher_->msg_.desired.positions.resize(dof_);
    state_publisher_->msg_.desired.velocities.resize(dof_);
    state_publisher_->msg_.desired.accelerations.resize(dof_);
    state_publisher_->msg_.actual.positions.resize(dof_);
    state_publisher_->msg_.error.positions.resize(dof_);
    state_publisher_->unlock();

    last_state_publish_time_ = get_node()->now();

    //
    // Handle action server configuration
    //

    action_monitor_period_ = duration_from_rate(action_monitor_rate_);

    action_server_ = rclcpp_action::create_server<FollowJointTrajAction>(
        get_node()->get_node_base_interface(),
        get_node()->get_node_clock_interface(),
        get_node()->get_node_logging_interface(),
        get_node()->get_node_waitables_interface(),
        std::string(get_node()->get_name()) + "/follow_joint_trajectory",
        std::bind(&SlidingMode2DofPressureTrajectoryController::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SlidingMode2DofPressureTrajectoryController::goal_cancelled_callback, this, std::placeholders::_1),
        std::bind(&SlidingMode2DofPressureTrajectoryController::goal_accepted_callback, this, std::placeholders::_1)
    );

    resize_joint_trajectory_point(joint_space_state_current_, dof_);
    resize_joint_trajectory_point(joint_space_state_desired_, dof_);
    resize_joint_trajectory_point(joint_space_state_error_, dof_);

END:
    RCLCPP_INFO(logger, "on_configure() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn
SlidingMode2DofPressureTrajectoryController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_activate()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    // order all joints in the storage
    for (size_t index = 0; index < command_interface_types_.size(); index++)
    {
        const std::string& interface_str = command_interface_types_[index];
        auto& interface_obj = joint_command_interface_[index];
        if (!controller_interface::get_ordered_interfaces(command_interfaces_,
                                                          joint_names_,
                                                          interface_str,
                                                          interface_obj
        ))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Expected %zu '%s' command interfaces, got %zu.",
                dof_,
                interface_str.c_str(),
                interface_obj.size()
            );
            rc = CallbackReturn::ERROR;
            goto END;
        }
    }
    for (size_t index = 0; index < state_interface_types_.size(); index++)
    {
        const std::string& interface_str = state_interface_types_[index];
        auto& interface_obj = joint_state_interface_[index];
        if (!controller_interface::get_ordered_interfaces(state_interfaces_,
                                                          joint_names_,
                                                          interface_str,
                                                          interface_obj
        ))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Expected %zu '%s' state interfaces, got %zu.",
                dof_,
                interface_str.c_str(),
                interface_obj.size()
            );
            rc = CallbackReturn::ERROR;
            goto END;
        }
    }

    // Create a 'home' pose, which is a simple trajectory with the only point
    // being the current position state read off of the hardware interface
    traj_msg_home_ptr_ = std::make_shared<JointTrajectoryMsg>();
    traj_msg_home_ptr_->header.stamp.sec = 0;
    traj_msg_home_ptr_->header.stamp.nanosec = 0;
    traj_msg_home_ptr_->points.resize(1);
    traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
    traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
    traj_msg_home_ptr_->points[0].positions.resize(joint_state_interface_[0].size());
    for (size_t index = 0; index < joint_state_interface_[0].size(); ++index)
    {
        traj_msg_home_ptr_->points[0].positions[index] = joint_state_interface_[0][index].get().get_value();
    }

    traj_external_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
    traj_home_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
    traj_msg_external_point_ptr_.writeFromNonRT(JointTrajectoryMsgSharedPtr());

    subscriber_is_active_ = true;
    traj_point_active_ptr_ = &traj_external_point_ptr_;
    last_state_publish_time_ = get_node()->now();

    // Initialize the current and desired states assuming a 'zero' state
    // TODO(nick): how to reset position?
    // TODO(nick): how to reset effort?
    joint_space_state_current_.positions.resize(dof_, 0.0);
    joint_space_state_current_.velocities.resize(dof_, 0.0);
    joint_space_state_current_.accelerations.resize(dof_, 0.0);
    joint_space_state_current_.effort.resize(dof_, 0.0);
    joint_space_state_desired_ = joint_space_state_current_;

    // TODO(nick): how to reset pressure?
    last_commanded_.joint_space = joint_space_state_current_;
    last_update_time_ = get_node()->now();

END:
    RCLCPP_INFO(get_node()->get_logger(), "on_activate() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn
SlidingMode2DofPressureTrajectoryController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_deactivate()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    // TODO(nick): Command the nominal pressures
    for (size_t index = 0; index < dof_; ++index)
    {
        joint_command_interface_[0][index].get().set_value(joint_command_interface_[0][index].get().get_value());
    }

    // Clear and release interfaces
    for (auto iter = joint_command_interface_.begin(); iter != joint_command_interface_.end(); ++iter)
    {
        iter->clear();
    }
    for (auto iter = joint_state_interface_.begin(); iter != joint_state_interface_.end(); ++iter)
    {
        iter->clear();
    }
    release_interfaces();

    subscriber_is_active_ = false;

END:
    RCLCPP_INFO(get_node()->get_logger(), "on_deactivate() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn
SlidingMode2DofPressureTrajectoryController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_cleanup()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    // go home
    traj_home_point_ptr_->update(traj_msg_home_ptr_);
    traj_point_active_ptr_ = &traj_home_point_ptr_;

END:
    RCLCPP_INFO(get_node()->get_logger(), "on_cleanup() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn
SlidingMode2DofPressureTrajectoryController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_error()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    if (!reset())
    {
        rc = CallbackReturn::ERROR;
        goto END;
    }

END:
    RCLCPP_INFO(get_node()->get_logger(), "on_error() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn
SlidingMode2DofPressureTrajectoryController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_shutdown()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

END:
    RCLCPP_INFO(get_node()->get_logger(), "on_shutdown() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::return_type SlidingMode2DofPressureTrajectoryController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        return controller_interface::return_type::OK;
    }

    //
    // Check if a new external message has been received from nonRT threads
    //

    auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
    auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
    if (current_external_msg != *new_external_msg)
    {
        fill_partial_goal(*new_external_msg);
        sort_to_local_joint_order(*new_external_msg);
        // TODO(denis): Add here integration of position and velocity
        traj_external_point_ptr_->update(*new_external_msg);
    }

    //
    // Update current state in joint-space and PMA telemetry
    //

    joint_space_state_current_.time_from_start.set__sec(0);  // TODO(nick): what does this do?

    {
        // Calculate the time elapsed since the last commanded pressure, then
        // use it to estimate the current joint positions and velocities
        const double dt = (time - last_update_time_).seconds();
        for (size_t index = 0; index < dof_; index++)
        {
            joint_space_state_current_.positions[index] += (
                (js_curr.velocities[index] * dt)
                + ((js_curr.accelerations[index] * 0.5 * dt * dt))
            );
            joint_space_state_current_.positions[index] += (
                (js_curr.velocities[index] * dt)
                + ((js_curr.accelerations[index] * 0.5 * dt * dt))
            );
        }

        double r = pm_segment_configurations[0].pulley_radius;
        double th = joint_space_state_current_.positions[0];
        double th_dot = joint_space_state_current_.velocities[0];
        pm_segment_telemetries[0].tricep_telemetry.length = r * (th + PMA_HPI);
        pm_segment_telemetries[0].tricep_telemetry.lengthening_rate = r * th_dot;
        pm_segment_telemetries[0].bicep_telemetry.length = r * (PMA_HPI - th);
        pm_segment_telemetries[0].bicep_telemetry.lengthening_rate = -pm_segment_telemetries[0].tricep_telemetry.lengthening_rate;

        r = pm_segment_configurations[1].pulley_radius;
        th = joint_space_state_current_.positions[1];
        th_dot = joint_space_state_current_.velocities[1];
        pm_segment_telemetries[1].bicep_telemetry.length = r * th;
        pm_segment_telemetries[1].bicep_telemetry.lengthening_rate = r * th_dot;
        pm_segment_telemetries[1].tricep_telemetry.length = r * (PMA_PI - th);
        pm_segment_telemetries[1].tricep_telemetry.lengthening_rate = -pm_segment_telemetries[1].bicep_telemetry.lengthening_rate;
    }

    // currently carrying out a trajectory
    if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
    {
        bool first_sample = false;
        // if sampling the first time, set the point before you sample
        if (!(*traj_point_active_ptr_)->is_sampled_already())
        {
            first_sample = true;
            (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, joint_space_state_current_);
        }

        // find segment for current timestamp
        joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
        const bool valid_point = (*traj_point_active_ptr_)->sample(
            time,
            interpolation_method_,
            state_desired_.joint_space,
            start_segment_itr,
            end_segment_itr
        );

        // Transform desired state in joint-space to desired state in
        // pressure-space
        compute_pma_robot_model(
            *(pm_control_configuration.get()),
            pm_segment_configurations,
            pm_segment_telemetries,
            state_desired_.joint_space.positions,
            state_desired_.joint_space.velocities,
            state_current_.joint_space.positions,
            state_current_.joint_space.velocities,
            desired_pressures_,
            desired_a_vector_,
            desired_G_matrix_
        );

        if (valid_point)
        {
            bool tolerance_violated_while_moving = false;
            bool outside_goal_tolerance = false;
            bool within_goal_time = true;
            double time_difference = 0.0;
            const bool before_last_point = (end_segment_itr != (*traj_point_active_ptr_)->end());

            // Check state/goal tolerance
            for (size_t index = 0; index < dof_; ++index)
            {
                // Update the error, defined as 'desired - current'
                {
                    const auto& js_desi = state_desired_.joint_space;
                    const auto& js_curr = state_current_.joint_space;

                    state_error_.joint_space.positions[index] = angles::shortest_angular_distance(js_curr.positions[index], js_desi.positions[index]);
                    state_error_.joint_space.velocities[index] = js_desi.velocities[index] - js_curr.velocities[index];
                    state_error_.joint_space.accelerations[index] = js_desi.accelerations[index] - js_curr.accelerations[index];
                    state_error_.joint_space.effort[index] = js_desi.effort[index] - js_curr.effort[index];
                    state_error_.pressures[index] = state_desired_.pressures[index] - state_current_.pressures[index];
                }

                // Always check the state tolerance on the first sample in case the first sample
                // is the last point
                if ((before_last_point || first_sample) && !check_state_tolerance_per_joint(state_error_.joint_space,
                                                                                            index,
                                                                                            default_tolerances_.state_tolerance[index],
                                                                                            false
                ))
                {
                    tolerance_violated_while_moving = true;
                }
                // past the final point, check that we end up inside goal tolerance
                if (!before_last_point && !check_state_tolerance_per_joint(state_error_.joint_space,
                                                                           index,
                                                                           default_tolerances_.goal_state_tolerance[index],
                                                                           false
                ))
                {
                    outside_goal_tolerance = true;
                    if (default_tolerances_.goal_time_tolerance != 0.0)
                    {
                        // if we exceed goal_time_tolerance set it to aborted
                        const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
                        const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

                        time_difference = get_node()->now().seconds() - traj_end.seconds();

                        if (time_difference > default_tolerances_.goal_time_tolerance)
                        {
                            within_goal_time = false;
                        }
                    }
                }
            }

            // set values for next hardware write() if tolerance is met
            if (!tolerance_violated_while_moving && within_goal_time)
            {
                for (size_t index = 0; index < dof_; ++index)
                {
                    joint_command_interface_[0][index].get().set_value(desired_pressures[index]);
                }
                last_commanded_a_vector_ = desired_a_vector_;
                last_commanded_G_matrix_ = desired_G_matrix_;
                last_commanded_pressures_ = desired_pressures_;
            }

            const auto active_goal = *rt_active_goal_.readFromRT();
            if (active_goal)
            {
                // send feedback
                auto feedback = std::make_shared<FollowJointTrajAction::Feedback>();
                feedback->header.stamp = time;
                feedback->joint_names = joint_names_;

                feedback->actual = state_current_.joint_space;
                feedback->desired = state_desired_.joint_space;
                feedback->error = state_error_.joint_space;
                active_goal->setFeedback(feedback);

                // check abort
                if (tolerance_violated_while_moving)
                {
                    set_hold_position();
                    auto result = std::make_shared<FollowJointTrajAction::Result>();

                    RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
                    result->set__error_code(FollowJointTrajAction::Result::PATH_TOLERANCE_VIOLATED);
                    active_goal->setAborted(result);
                    // TODO(matthew-reynolds): Need a lock-free write here
                    // See https://github.com/ros-controls/ros2_controllers/issues/168
                    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

                    // check goal tolerance
                }
                else if (!before_last_point)
                {
                    if (!outside_goal_tolerance)
                    {
                        auto res = std::make_shared<FollowJointTrajAction::Result>();
                        res->set__error_code(FollowJointTrajAction::Result::SUCCESSFUL);
                        active_goal->setSucceeded(res);
                        // TODO(matthew-reynolds): Need a lock-free write here
                        // See https://github.com/ros-controls/ros2_controllers/issues/168
                        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

                        RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
                    }
                    else if (!within_goal_time)
                    {
                        set_hold_position();
                        auto result = std::make_shared<FollowJointTrajAction::Result>();
                        result->set__error_code(FollowJointTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
                        active_goal->setAborted(result);
                        // TODO(matthew-reynolds): Need a lock-free write here
                        // See https://github.com/ros-controls/ros2_controllers/issues/168
                        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
                        RCLCPP_WARN(
                            get_node()->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
                            time_difference);
                    }
                    // else, run another cycle while waiting for outside_goal_tolerance
                    // to be satisfied or violated within the goal_time_tolerance
                }
            }
            else if (tolerance_violated_while_moving)
            {
                set_hold_position();
                RCLCPP_ERROR(get_node()->get_logger(), "Holding position due to state tolerance violation");
            }
        }
    }

    // If it's time to do so, publish the state
    if ((state_publisher_period_.seconds() > 0.0)
        && (get_node()->now() >= (last_state_publish_time_ + state_publisher_period_))
        && state_publisher_
        && state_publisher_->trylock()
    )
    {
        state_publisher_->msg_.header.stamp = last_state_publish_time_;
        state_publisher_->msg_.desired = state_current_.joint_space;
        state_publisher_->msg_.actual = state_desired_.joint_space;
        state_publisher_->msg_.error = state_error_.joint_space;
        last_state_publish_time_ = get_node()->now();
        state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

bool SlidingMode2DofPressureTrajectoryController::reset()
{
    subscriber_is_active_ = false;
    joint_command_subscriber_.reset();

    traj_point_active_ptr_ = nullptr;
    traj_external_point_ptr_.reset();
    traj_home_point_ptr_.reset();
    traj_msg_home_ptr_.reset();

    // iterator has no default value
    // prev_traj_point_ptr_;

    pm_control_configuration = nullptr;
    pm_segment_configurations.clear();
    pm_segment_telemetries.clear();

    return true;
}

void SlidingMode2DofPressureTrajectoryController::topic_callback(const JointTrajectoryMsgSharedPtr msg)
{
    if (!validate_trajectory_msg(*msg))
    {
        return;
    }
    // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
    // always replace old msg with new one for now
    if (subscriber_is_active_)
    {
        add_new_trajectory_msg(msg);
    }
};

rclcpp_action::GoalResponse SlidingMode2DofPressureTrajectoryController::goal_received_callback(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FollowJointTrajAction::Goal> goal)
{
    RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

    // Precondition: Running controller
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Can't accept new action goals. Controller is not running."
        );
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!validate_trajectory_msg(goal->trajectory))
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SlidingMode2DofPressureTrajectoryController::goal_cancelled_callback(const FollowJointTrajActionGoalSharedPtr goal_handle)
{
    RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

    // Check that cancel request refers to currently active goal (if any)
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal && active_goal->gh_ == goal_handle)
    {
        // Controller uptime
        // Enter hold current position mode
        set_hold_position();

        RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Canceling active action goal because cancel callback received."
        );

        // Mark the current goal as canceled
        auto action_res = std::make_shared<FollowJointTrajAction::Result>();
        active_goal->setCanceled(action_res);
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SlidingMode2DofPressureTrajectoryController::goal_accepted_callback(FollowJointTrajActionGoalSharedPtr goal_handle)
{
    // Update new trajectory
    {
        preempt_active_goal();
        auto traj_msg = std::make_shared<JointTrajectoryMsg>(goal_handle->get_goal()->trajectory);

        add_new_trajectory_msg(traj_msg);
    }

    // Update the active goal
    RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
    rt_goal->preallocated_feedback_->joint_names = joint_names_;
    rt_goal->execute();
    rt_active_goal_.writeFromNonRT(rt_goal);

    // Setup goal status checking timer
    goal_handle_timer_ = get_node()->create_wall_timer(
        action_monitor_period_.to_chrono<std::chrono::seconds>(),
        std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal)
    );
}

void SlidingMode2DofPressureTrajectoryController::fill_partial_goal(
    JointTrajectoryMsgSharedPtr trajectory_msg) const
{
    // joint names in the goal are a subset of existing joints, as checked in goal_callback
    // so if the size matches, the goal contains all controller joints
    if (dof_ == trajectory_msg->joint_names.size())
    {
        return;
    }

    trajectory_msg->joint_names.reserve(dof_);

    for (size_t index = 0; index < dof_; ++index)
    {
        if (pma_util::list_contains(trajectory_msg->joint_names, joint_names_[index]))
        {
            // joint found on msg
            continue;
        }

        trajectory_msg->joint_names.push_back(joint_names_[index]);

        for (auto& it : trajectory_msg->points)
        {
            // Assume hold position with 0 velocity and acceleration for missing joints
            if (!it.positions.empty())
            {
                // copy current state if state interface exists
                it.positions.push_back(joint_state_interface_[0][index].get().get_value());
            }
            if (!it.velocities.empty())
            {
                it.velocities.push_back(0.0);
            }
            if (!it.accelerations.empty())
            {
                it.accelerations.push_back(0.0);
            }
            if (!it.effort.empty())
            {
                it.effort.push_back(0.0);
            }
        }
    }
}

void SlidingMode2DofPressureTrajectoryController::sort_to_local_joint_order(
    JointTrajectoryMsgSharedPtr trajectory_msg)
{
    // rearrange all points in the trajectory message based on mapping
    std::vector<size_t> mapping_vector = joint_trajectory_controller::mapping(trajectory_msg->joint_names, joint_names_);
    auto remap = [this](
        const std::vector<double>& to_remap,
        const std::vector<size_t>& mapping
    ) -> std::vector<double>
    {
        if (to_remap.empty())
        {
            return to_remap;
        }
        if (to_remap.size() != mapping.size())
        {
            RCLCPP_WARN(
                get_node()->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
            return to_remap;
        }
        std::vector<double> output;
        output.resize(mapping.size(), 0.0);
        for (size_t index = 0; index < mapping.size(); ++index)
        {
            output[mapping[index]] = to_remap[index];
        }
        return output;
    };

    for (size_t index = 0; index < trajectory_msg->points.size(); ++index)
    {
        trajectory_msg->points[index].positions = remap(trajectory_msg->points[index].positions, mapping_vector);
        trajectory_msg->points[index].velocities = remap(trajectory_msg->points[index].velocities, mapping_vector);
        trajectory_msg->points[index].accelerations = remap(trajectory_msg->points[index].accelerations, mapping_vector);
        trajectory_msg->points[index].effort = remap(trajectory_msg->points[index].effort, mapping_vector);
    }
}

bool SlidingMode2DofPressureTrajectoryController::validate_trajectory_point_field(
    size_t joint_names_size,
    const std::vector<double>& vector_field,
    const std::string& string_for_vector_field,
    size_t i,
    bool allow_empty
) const
{
    if (allow_empty && vector_field.empty())
    {
        return true;
    }
    if (joint_names_size != vector_field.size())
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Mismatch between joint_names (%zu) and %s (%zu) at point #%zu.",
            joint_names_size,
            string_for_vector_field.c_str(),
            vector_field.size(),
            i
        );
        return false;
    }
    return true;
}

bool SlidingMode2DofPressureTrajectoryController::validate_trajectory_msg(const JointTrajectoryMsg& trajectory) const
{
    // Goal should specify all controller joints
    if (trajectory.joint_names.size() != dof_)
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Joints on incoming trajectory don't match the controller joints."
        );
        return false;
    }

    if (trajectory.joint_names.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
        return false;
    }

    const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
    // If the starting time it set to 0.0, it means the controller should start it now.
    // Otherwise we check if the trajectory ends before the current time,
    // in which case it can be ignored.
    if (trajectory_start_time.seconds() != 0.0)
    {
        auto trajectory_end_time = trajectory_start_time;
        for (const auto& p : trajectory.points)
        {
            trajectory_end_time += p.time_from_start;
        }
        if (trajectory_end_time < get_node()->now())
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Received trajectory with non zero time start time (%f) that ends on the past (%f)",
                trajectory_start_time.seconds(), trajectory_end_time.seconds());
            return false;
        }
    }

    for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
    {
        const std::string& incoming_joint_name = trajectory.joint_names[i];

        auto it = std::find(joint_names_.begin(), joint_names_.end(), incoming_joint_name);
        if (it == joint_names_.end())
        {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Incoming joint %s doesn't match the controller's joints.",
                incoming_joint_name.c_str());
            return false;
        }
    }

    rclcpp::Duration previous_traj_time(0ms);
    for (size_t i = 0; i < trajectory.points.size(); ++i)
    {
        if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively",
                i - 1, i, previous_traj_time.seconds(),
                rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
            return false;
        }
        previous_traj_time = trajectory.points[i].time_from_start;

#define VALIDATE_TRAJ_POINT_FIELD(f, b) (validate_trajectory_point_field(trajectory.joint_names.size(), trajectory.points[i].f, #f, i, b))

        if (!VALIDATE_TRAJ_POINT_FIELD(positions, false)
            || !VALIDATE_TRAJ_POINT_FIELD(velocities, true)
            || !VALIDATE_TRAJ_POINT_FIELD(accelerations, true)
            || !VALIDATE_TRAJ_POINT_FIELD(effort, true)
        )
        {
            return false;
        }

#undef VALIDATE_TRAJ_POINT_FIELD

    }
    return true;
}

void SlidingMode2DofPressureTrajectoryController::add_new_trajectory_msg(
    const JointTrajectoryMsgSharedPtr& traj_msg)
{
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

void SlidingMode2DofPressureTrajectoryController::preempt_active_goal()
{
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal)
    {
        set_hold_position();
        auto action_res = std::make_shared<FollowJointTrajAction::Result>();
        action_res->set__error_code(FollowJointTrajAction::Result::INVALID_GOAL);
        action_res->set__error_string("Current goal cancelled due to new incoming action.");
        active_goal->setCanceled(action_res);
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
}

void SlidingMode2DofPressureTrajectoryController::set_hold_position()
{
    JointTrajectoryMsg empty_msg;
    empty_msg.header.stamp = rclcpp::Time(0);

    auto traj_msg = std::make_shared<JointTrajectoryMsg>(empty_msg);
    add_new_trajectory_msg(traj_msg);
}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pma_control::SlidingMode2DofPressureTrajectoryController, controller_interface::ControllerInterface)
