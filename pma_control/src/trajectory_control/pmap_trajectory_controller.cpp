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
#include "pma_hardware/math_utils/compute_segment_torques.hpp"
#include "pma_util/util.hpp"
#include "rclcpp_action/create_server.hpp"

#define PMA_PI (M_PI)
#define PMA_HPI (PMA_PI/2.0)

static const size_t REQUIRED_DOF = 2;

using namespace std::chrono_literals;

namespace im = joint_trajectory_controller::interpolation_methods;

// A single 2-DOF pressure point consists of the robot shoulder pressure (the
// first element in the pair) and the robot elbow pressure (the second one).
using JointPressureTrajectory2DofPressure = std::pair<double, double>;

namespace
{
    rclcpp::Duration duration_from_rate(const double rate)
    {
        return rclcpp::Duration::from_seconds((rate > 0.0) ? (1.0 / rate) : 0.0);
    }

    void resize_pma_trajectory_point(
        pma_control::PneumaticMuscleActuatorTrajectoryPoint& point,
        const size_t size
    )
    {
        point.joint_space.positions.resize(size, 0.0);
        point.joint_space.velocities.resize(size, 0.0);
        point.joint_space.accelerations.resize(size, 0.0);
        point.joint_space.effort.resize(size, 0.0);
        point.pressures.resize(size, 0.0);
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

    pma_control::PressureList compute_desired_input_pressures_for(
        const pma_hardware::PneumaticMuscleControlConfiguration& pm_control_configuration,
        const std::vector<pma_hardware::PneumaticMuscleSegmentConfiguration>& pm_segment_configurations,
        const std::vector<pma_hardware::PneumaticMuscleSegmentTelemetry>& pm_segment_telemetries,
        const std::vector<double>& desired_joint_positions,
        const std::vector<double>& desired_joint_velocities,
        const std::vector<double>& actual_joint_positions,
        const std::vector<double>& actual_joint_velocities
    )
    {
        const pma_hardware::PneumaticMuscleSegmentConfiguration& shoulder_segment_configuration = pm_segment_configurations[0];
        const pma_hardware::PneumaticMuscleSegmentConfiguration& elbow_segment_configuration = pm_segment_configurations[1];
        const pma_hardware::PneumaticMuscleSegmentTelemetry& shoulder_segment_telemetry = pm_segment_telemetries[0];
        const pma_hardware::PneumaticMuscleSegmentTelemetry& elbow_segment_telemetry = pm_segment_telemetries[1];
        double shoulder_torque_nominal_pressure;
        double shoulder_torque_factor_input_pressure;
        double elbow_torque_nominal_pressure;
        double elbow_torque_factor_input_pressure;

        pma_hardware::compute_shoulder_segment_torque_components(
            shoulder_segment_configuration,
            shoulder_segment_telemetry,
            shoulder_torque_nominal_pressure,
            shoulder_torque_factor_input_pressure
        );
        pma_hardware::compute_elbow_segment_torque_components(
            elbow_segment_configuration,
            elbow_segment_telemetry,
            elbow_torque_nominal_pressure,
            elbow_torque_factor_input_pressure
        );

#define DECLARE_SEGMENT_VARS(config, num) \
        const double m_##num = config.mass; \
        const double l_##num = config.segment_length; \
        const double l_com_##num = config.segment_com_length; \
        const double I_##num = m_##num * l_com_##num * l_com_##num; \
        const double th_star_##num = actual_joint_positions[num-1]; \
        const double th_dot_star_##num = actual_joint_velocities[num-1]; \
        const double th_##num = desired_joint_positions[num-1]; \
        const double th_dot_##num = desired_joint_velocities[num-1]

        // Declare variables for each segment
        DECLARE_SEGMENT_VARS(shoulder_segment_configuration, 1);
        DECLARE_SEGMENT_VARS(elbow_segment_configuration, 2);

#undef DECLARE_SEGMENT_VARS

        // Declare any others
        const double a_g = pm_control_configuration.a_g;
        const Eigen::Vector2d th {{th_1, th_2}};
        const Eigen::Vector2d th_dot {{th_dot_1, th_dot_2}};
        const double c_2 = std::cos(th_2);

        // Calculate the inverse of the D matrix
        const double d11 = (2 * I_1) + (2 * I_2) + (m_2 * ((l_1 * l_1) + (2 * l_1 * l_com_2 * c_2)));
        const double d12 = I_2 + (m_2 * l_1 * l_com_2 * c_2);
        const double d22 = 2 * I_2;
        const Eigen::Matrix2d D_inv = Eigen::Matrix2d({
            {d11, d12},
            {d12, d22}
        }).inverse();

        // Calculate the inverse of the G matrix
        const Eigen::Matrix2d G_inv = (D_inv * Eigen::Matrix2d({
            {shoulder_torque_factor_input_pressure, 0.0},
            {0.0,                                   elbow_torque_factor_input_pressure}
        })).inverse();

        // Calculate the a vector
        const double h = -m_2 * l_1 * l_com_2 * std::sin(th_2);
        const Eigen::Matrix2d C
        {
            {h * th_dot_2,  (h * th_dot_1) + (h * th_dot_2)},
            {-h * th_dot_1, 0.0}
        };
        const double f_2 = m_2 * l_com_2 * a_g * std::cos(th_1 + th_2);
        const double f_1 = (((m_1 * l_com_1) + (m_2 * l_1)) * a_g * std::cos(th_1)) + f_2;
        const Eigen::Vector2d f {{f_1, f_2}};
        const Eigen::Vector2d tau_0 {{shoulder_torque_nominal_pressure, elbow_torque_nominal_pressure}};
        const Eigen::Vector2d a = D_inv * ((C * th_dot * -1) - f + tau_0);

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

        // Calculate the ideal input pressures
        const Eigen::Vector2d delta_p = G_inv * (th_r - a - ks);

        // Finished, return the input pressures as a list
        return {delta_p(0), delta_p(1)};
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
    interpolation_method_(im::DEFAULT_INTERPOLATION),
    acceleration_gravity_(9.81),
    sliding_mode_control_k_1_(1.0),
    sliding_mode_control_k_2_(1.0),
    sliding_mode_control_Gamma_1_(1.0),
    sliding_mode_control_Gamma_2_(1.0),
    sliding_mode_control_mu_1_(1.0),
    sliding_mode_control_mu_2_(1.0)
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

    // Declare parameters while the lifecycle node is initializing
    try {
        // Base controller parameters
        AUTO_DEC1(joint_names_, "joints", std::vector<std::string>);
        AUTO_DEC2(state_publish_rate, double);
        AUTO_DEC2(action_monitor_rate, double);
        interpolation_method_ = im::from_string(auto_declare<std::string>(
            "interpolation_method",
            im::InterpolationMethodMap.at(interpolation_method_)
        ));

        // Parameters specific to sliding-mode control
        AUTO_DEC2(acceleration_gravity, double);
        AUTO_DEC2(sliding_mode_control_k_1, double);
        AUTO_DEC2(sliding_mode_control_k_2, double);
        AUTO_DEC2(sliding_mode_control_Gamma_1, double);
        AUTO_DEC2(sliding_mode_control_Gamma_2, double);
        AUTO_DEC2(sliding_mode_control_mu_1, double);
        AUTO_DEC2(sliding_mode_control_mu_2, double);
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
        rc = CallbackReturn::ERROR;
        goto END;
    }

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

    resize_pma_trajectory_point(state_current_, dof_);
    resize_pma_trajectory_point(state_desired_, dof_);
    resize_pma_trajectory_point(state_error_, dof_);

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

    // Initialize the current and desired states from the hardware interfaces
    read_state_from_hardware(state_current_);
    state_desired_ = state_current_;

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

    // Check if a new external message has been received from nonRT threads
    auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
    auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
    if (current_external_msg != *new_external_msg)
    {
        fill_partial_goal(*new_external_msg);
        sort_to_local_joint_order(*new_external_msg);
        // TODO(denis): Add here integration of position and velocity
        traj_external_point_ptr_->update(*new_external_msg);
    }

    // current state update
    state_current_.joint_space.time_from_start.set__sec(0);
    read_state_from_hardware(state_current_);

    // currently carrying out a trajectory
    if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
    {
        bool first_sample = false;
        // if sampling the first time, set the point before you sample
        if (!(*traj_point_active_ptr_)->is_sampled_already())
        {
            first_sample = true;
            (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current_.joint_space);
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
                // Convert desired state in joint-space to desired state in
                // pressure-space
                const PressureList desired_pressures = compute_desired_input_pressures_for(
                    *(pm_control_configuration.get()),
                    pm_segment_configurations,
                    pm_segment_telemetries,
                    state_desired_.joint_space.positions,
                    state_desired_.joint_space.velocities,
                    state_current_.joint_space.positions,
                    state_current_.joint_space.velocities
                );

                // Set values for next hardware write()
                for (size_t index = 0; index < dof_; ++index)
                {
                    joint_command_interface_[0][index].get().set_value(desired_pressures[index]);
                }
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

void SlidingMode2DofPressureTrajectoryController::read_state_from_hardware(PneumaticMuscleActuatorTrajectoryPoint& state)
{
    // Assign values from the hardware
    for (size_t index = 0; index < dof_; ++index)
    {
        state.joint_space.positions[index]     = joint_state_interface_[1][index].get().get_value();
        state.joint_space.velocities[index]    = joint_state_interface_[2][index].get().get_value();
        state.joint_space.accelerations[index] = joint_state_interface_[3][index].get().get_value();
        state.joint_space.effort[index]        = joint_state_interface_[4][index].get().get_value();
        state.pressures[index]                 = joint_state_interface_[5][index].get().get_value();
    }
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
