#include "pma_control/trajectory_control/pmap_trajectory_controller.hpp"

#include <stddef.h>
#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/header.hpp"

#include "pma_util/util.hpp"

namespace im = joint_trajectory_controller::interpolation_methods;

namespace pma_control
{
PmapTrajectoryController::PmapTrajectoryController() :
    controller_interface::ControllerInterface(),
    joint_names_({}),
    dof_(0)
{
}

controller_interface::InterfaceConfiguration
PmapTrajectoryController::command_interface_configuration() const
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
    for (const auto& joint_name : command_joint_names_)
    {
        for (const auto& interface_type : command_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::InterfaceConfiguration
PmapTrajectoryController::state_interface_configuration() const
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

controller_interface::CallbackReturn PmapTrajectoryController::on_init()
{
    fprintf(stdout, "Entered on_init()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    try
    {
        // with the lifecycle node being initialized, we can declare parameters
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_joint_names_ = auto_declare<std::vector<std::string>>("command_joints", command_joint_names_);
        allow_partial_joints_goal_ = auto_declare<bool>("allow_partial_joints_goal", allow_partial_joints_goal_);
        open_loop_control_ = auto_declare<bool>("open_loop_control", open_loop_control_);
        allow_integration_in_goal_trajectories_ = auto_declare<bool>("allow_integration_in_goal_trajectories", allow_integration_in_goal_trajectories_);
        state_publish_rate_ = auto_declare<double>("state_publish_rate", 50.0);
        action_monitor_rate_ = auto_declare<double>("action_monitor_rate", 20.0);

        const std::string interpolation_string = auto_declare<std::string>(
            "interpolation_method",
            im::InterpolationMethodMap.at(im::DEFAULT_INTERPOLATION)
        );
        interpolation_method_ = im::from_string(interpolation_string);
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        rc = CallbackReturn::ERROR;
        goto END;
    }

END:
    fprintf(stdout, "on_init() rc=%s \n", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn PmapTrajectoryController::on_configure(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_configure()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    const auto logger = get_node()->get_logger();

    const std::string interpolation_string = get_node()->get_parameter("interpolation_method").as_string();

    // update parameters
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if ((dof_ > 0) && (joint_names_.size() != dof_))
    {
        RCLCPP_ERROR(
            logger,
            "The PmapTrajectoryController does not support restarting with a different number of DOF"
        );
        // TODO(andyz): update vector lengths if num. joints did change and re-initialize them so we
        // can continue
        rc = CallbackReturn::FAILURE;
        goto END;
    }

    dof_ = joint_names_.size();

    // TODO(destogl): why is this here? Add comment or move
    if (!reset())
    {
        rc = CallbackReturn::FAILURE;
        goto END;
    }

    if (joint_names_.empty())
    {
        // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
        RCLCPP_WARN(logger, "'joints' parameter is empty.");
    }

    command_joint_names_ = get_node()->get_parameter("command_joints").as_string_array();

    if (command_joint_names_.empty())
    {
        command_joint_names_ = joint_names_;
        RCLCPP_INFO(
            logger,
            "No specific joint names are used for command interfaces. Using 'joints' parameter."
        );
    }
    else if (command_joint_names_.size() != joint_names_.size())
    {
        RCLCPP_ERROR(
            logger,
            "'command_joints' parameter has to have the same size as 'joints' parameter."
        );
        rc = CallbackReturn::FAILURE;
        goto END;
    }

    // Check if only allowed interface types are used and initialize storage to avoid memory
    // allocation during activation
    joint_command_interface_.resize(command_interface_types_.size());
    joint_state_interface_.resize(state_interface_types_.size());

    if (use_closed_loop_pid_adapter_)
    {
        pids_.resize(dof_);
        ff_velocity_scale_.resize(dof_);
        tmp_command_.resize(dof_, 0.0);

        // Init PID gains from ROS parameter server
        for (size_t i = 0; i < pids_.size(); ++i)
        {
            const std::string prefix = "gains." + command_joint_names_[i];
            const auto k_p = auto_declare<double>(prefix + ".p", 0.0);
            const auto k_i = auto_declare<double>(prefix + ".i", 0.0);
            const auto k_d = auto_declare<double>(prefix + ".d", 0.0);
            const auto i_clamp = auto_declare<double>(prefix + ".i_clamp", 0.0);
            ff_velocity_scale_[i] =
                auto_declare<double>("ff_velocity_scale/" + command_joint_names_[i], 0.0);
            // Initialize PID
            pids_[i] = std::make_shared<control_toolbox::Pid>(k_p, k_i, k_d, i_clamp, -i_clamp);
        }
    }

    // Print output so users can be sure the interface setup is correct
    RCLCPP_INFO(
        logger,
        "Command interfaces are [%s] and state interfaces are [%s].",
        pma_util::get_delimited_list(command_interface_types_, ", ").c_str(),
        pma_util::get_delimited_list(state_interface_types_, ", ").c_str()
    );

    default_tolerances_ = joint_trajectory_controller::get_segment_tolerances(*get_node(), command_joint_names_);

    // Read parameters customizing controller for special cases
    open_loop_control_ = get_node()->get_parameter("open_loop_control").get_value<bool>();
    allow_integration_in_goal_trajectories_ = get_node()->get_parameter("allow_integration_in_goal_trajectories").get_value<bool>();

    interpolation_method_ = im::from_string(interpolation_string);
    RCLCPP_INFO(
        logger,
        "Using '%s' interpolation method.",
        im::InterpolationMethodMap.at(interpolation_method_).c_str()
    );

    joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "~/joint_trajectory",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&PmapTrajectoryController::topic_callback, this, std::placeholders::_1)
    );

    // State publisher
    state_publish_rate_ = get_node()->get_parameter("state_publish_rate").get_value<double>();
    RCLCPP_INFO(logger, "Controller state will be published at %.2f Hz.", state_publish_rate_);
    if (state_publish_rate_ > 0.0)
    {
        state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate_);
    }
    else
    {
        state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
    }

    publisher_ = get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<StatePublisher>(publisher_);

    state_publisher_->lock();
    state_publisher_->msg_.joint_names = command_joint_names_;
    state_publisher_->msg_.desired.positions.resize(dof_);
    state_publisher_->msg_.desired.velocities.resize(dof_);
    state_publisher_->msg_.desired.accelerations.resize(dof_);
    state_publisher_->msg_.actual.positions.resize(dof_);
    state_publisher_->msg_.error.positions.resize(dof_);
    state_publisher_->unlock();

    last_state_publish_time_ = get_node()->now();

    // action server configuration
    allow_partial_joints_goal_ = get_node()->get_parameter("allow_partial_joints_goal").get_value<bool>();
    if (allow_partial_joints_goal_)
    {
        RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
    }

    action_monitor_rate_ = get_node()->get_parameter("action_monitor_rate").get_value<double>();
    RCLCPP_INFO(logger, "Action status changes will be monitored at %.2f Hz.", action_monitor_rate_);
    action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / action_monitor_rate_);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
        get_node()->get_node_base_interface(),
        get_node()->get_node_clock_interface(),
        get_node()->get_node_logging_interface(),
        get_node()->get_node_waitables_interface(),
        std::string(get_node()->get_name()) + "/follow_joint_trajectory",
        std::bind(&PmapTrajectoryController::goal_received_callback, this, _1, _2),
        std::bind(&PmapTrajectoryController::goal_cancelled_callback, this, _1),
        std::bind(&PmapTrajectoryController::goal_accepted_callback, this, _1)
    );

    resize_joint_trajectory_point(state_current_, dof_);
    resize_joint_trajectory_point(state_desired_, dof_);
    resize_joint_trajectory_point(state_error_, dof_);
    resize_joint_trajectory_point(last_commanded_state_, dof_);

END:
    RCLCPP_INFO(logger, "on_configure() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn PmapTrajectoryController::on_activate(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_activate()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    trajectory_msgs::msg::JointTrajectoryPoint state;

    // order all joints in the storage
    for (size_t index = 0; index < command_interface_types_.size(); index++)
    {
        const std::string& interface_str = command_interface_types_[index];
        auto& interface_obj = joint_command_interface_[index];
        if (!controller_interface::get_ordered_interfaces(command_interfaces_,
                                                          command_joint_names_,
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

    // Store 'home' pose
    traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
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
    traj_msg_external_point_ptr_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

    subscriber_is_active_ = true;
    traj_point_active_ptr_ = &traj_external_point_ptr_;
    last_state_publish_time_ = get_node()->now();

    // Initialize current state storage if hardware state has tracking offset
    read_state_from_hardware(state_current_);
    read_state_from_hardware(state_desired_);
    read_state_from_hardware(last_commanded_state_);
    // Handle restart of controller by reading from commands if
    // those are not nan
    resize_joint_trajectory_point(state, dof_);
//    if (read_state_from_command_interfaces(state))
//    {
//        state_current_ = state;
//        state_desired_ = state;
//        last_commanded_state_ = state;
//    }

END:
    RCLCPP_INFO(get_node()->get_logger(), "on_activate() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::CallbackReturn PmapTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_deactivate()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    // Halt in pressure-space by commanding the current pressure
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

controller_interface::CallbackReturn PmapTrajectoryController::on_cleanup(
    const rclcpp_lifecycle::State &)
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

controller_interface::CallbackReturn PmapTrajectoryController::on_error(
    const rclcpp_lifecycle::State &)
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

controller_interface::CallbackReturn PmapTrajectoryController::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Entered on_shutdown()");

    controller_interface::CallbackReturn rc = CallbackReturn::SUCCESS;

    // TODO(karsten1987): what to do?

END:
    RCLCPP_INFO(get_node()->get_logger(), "on_shutdown() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

controller_interface::return_type PmapTrajectoryController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        return controller_interface::return_type::OK;
    }

    auto compute_error_for_joint = [&](JointTrajectoryPoint& error,
                                       int index,
                                       const JointTrajectoryPoint& current,
                                       const JointTrajectoryPoint& desired)
    {
        // error defined as the difference between current and desired
        error.positions[index] = angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    };

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

    // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
    // changed, but its value only?
    auto assign_interface_from_point = [&](auto& joint_interface,
                                           const std::vector<double>& trajectory_point_interface)
    {
        for (size_t index = 0; index < dof_; ++index)
        {
            joint_interface[index].get().set_value(trajectory_point_interface[index]);
        }
    };

    // current state update
    state_current_.time_from_start.set__sec(0);
    read_state_from_hardware(state_current_);

    // currently carrying out a trajectory
    if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
    {
        bool first_sample = false;
        // if sampling the first time, set the point before you sample
        if (!(*traj_point_active_ptr_)->is_sampled_already())
        {
            first_sample = true;
            if (open_loop_control_)
            {
                (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, last_commanded_state_);
            }
            else
            {
                (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current_);
            }
        }

        // find segment for current timestamp
        joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
        const bool valid_point = (*traj_point_active_ptr_)->sample(
            time,
            interpolation_method_,
            state_desired_,
            start_segment_itr,
            end_segment_itr
        );

        if (valid_point)
        {
            bool tolerance_violated_while_moving = false;
            bool outside_goal_tolerance = false;
            bool within_goal_time = true;
            double time_difference = 0.0;
            const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

            // Check state/goal tolerance
            for (size_t index = 0; index < dof_; ++index)
            {
                compute_error_for_joint(state_error_, index, state_current_, state_desired_);

                // Always check the state tolerance on the first sample in case the first sample
                // is the last point
                if (
                    (before_last_point || first_sample) &&
                    !check_state_tolerance_per_joint(
                        state_error_, index, default_tolerances_.state_tolerance[index], false))
                {
                    tolerance_violated_while_moving = true;
                }
                // past the final point, check that we end up inside goal tolerance
                if (
                    !before_last_point &&
                    !check_state_tolerance_per_joint(
                        state_error_, index, default_tolerances_.goal_state_tolerance[index], false))
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
                if (use_closed_loop_pid_adapter_)
                {
                    // Update PIDs
                    for (auto i = 0ul; i < dof_; ++i)
                    {
                        tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
                                                            pids_[i]->computeCommand(
                                                                state_desired_.positions[i] - state_current_.positions[i],
                                                                state_desired_.velocities[i] - state_current_.velocities[i],
                                                                (uint64_t)period.nanoseconds());
                    }
                }

                // set values for next hardware write()
                assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);

                // store the previous command. Used in open-loop control mode
                last_commanded_state_ = state_desired_;
            }

            const auto active_goal = *rt_active_goal_.readFromRT();
            if (active_goal)
            {
                // send feedback
                auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
                feedback->header.stamp = time;
                feedback->joint_names = joint_names_;

                feedback->actual = state_current_;
                feedback->desired = state_desired_;
                feedback->error = state_error_;
                active_goal->setFeedback(feedback);

                // check abort
                if (tolerance_violated_while_moving)
                {
                    set_hold_position();
                    auto result = std::make_shared<FollowJTrajAction::Result>();

                    RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
                    result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
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
                        auto res = std::make_shared<FollowJTrajAction::Result>();
                        res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
                        active_goal->setSucceeded(res);
                        // TODO(matthew-reynolds): Need a lock-free write here
                        // See https://github.com/ros-controls/ros2_controllers/issues/168
                        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

                        RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
                    }
                    else if (!within_goal_time)
                    {
                        set_hold_position();
                        auto result = std::make_shared<FollowJTrajAction::Result>();
                        result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
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

    publish_state(state_desired_, state_current_, state_error_);
    return controller_interface::return_type::OK;
}

void PmapTrajectoryController::read_state_from_hardware(JointTrajectoryPoint& state)
{
    auto assign_point_from_interface =
        [&](std::vector<double>& trajectory_point_interface, const auto& joint_interface)
    {
        for (size_t index = 0; index < dof_; ++index)
        {
            trajectory_point_interface[index] = joint_interface[index].get().get_value();
        }
    };

    // Assign values from the hardware
    // Position states always exist
    assign_point_from_interface(state.positions, joint_state_interface_[0]);
    // velocity and acceleration states are optional
    // Make empty so the property is ignored during interpolation
    state.velocities.clear();
    state.accelerations.clear();
}

bool PmapTrajectoryController::read_state_from_command_interfaces(JointTrajectoryPoint& state)
{
    bool has_values = true;

    auto assign_point_from_interface =
        [&](std::vector<double>& trajectory_point_interface, const auto& joint_interface)
    {
        for (size_t index = 0; index < dof_; ++index)
        {
            trajectory_point_interface[index] = joint_interface[index].get().get_value();
        }
    };

    auto interface_has_values = [](const auto& joint_interface)
    {
        return std::find_if(
                         joint_interface.begin(), joint_interface.end(),
                         [](const auto& interface)
                         { return std::isnan(interface.get().get_value()); }) == joint_interface.end();
    };

    // Assign values from the command interfaces as state. Therefore needs check for both.
    // Position state interface has to exist always
    if (interface_has_values(joint_command_interface_[0]))
    {
        assign_point_from_interface(state.positions, joint_command_interface_[0]);
    }
    else
    {
        state.positions.clear();
        has_values = false;
    }
    state.velocities.clear();
    state.accelerations.clear();

    return has_values;
}

bool PmapTrajectoryController::reset()
{
    subscriber_is_active_ = false;
    joint_command_subscriber_.reset();

    for (const auto& pid : pids_)
    {
        pid->reset();
    }

    // iterator has no default value
    // prev_traj_point_ptr_;
    traj_point_active_ptr_ = nullptr;
    traj_external_point_ptr_.reset();
    traj_home_point_ptr_.reset();
    traj_msg_home_ptr_.reset();

    // reset pids
    for (const auto& pid : pids_)
    {
        pid->reset();
    }

    return true;
}

void PmapTrajectoryController::publish_state(
    const JointTrajectoryPoint& desired_state, const JointTrajectoryPoint& current_state,
    const JointTrajectoryPoint& state_error)
{
    if (state_publisher_period_.seconds() <= 0.0)
    {
        return;
    }

    if (get_node()->now() < (last_state_publish_time_ + state_publisher_period_))
    {
        return;
    }

    if (state_publisher_ && state_publisher_->trylock())
    {
        last_state_publish_time_ = get_node()->now();
        state_publisher_->msg_.header.stamp = last_state_publish_time_;
        state_publisher_->msg_.desired.positions = desired_state.positions;
        state_publisher_->msg_.desired.velocities = desired_state.velocities;
        state_publisher_->msg_.desired.accelerations = desired_state.accelerations;
        state_publisher_->msg_.actual.positions = current_state.positions;
        state_publisher_->msg_.error.positions = state_error.positions;
        state_publisher_->unlockAndPublish();
    }
}

void PmapTrajectoryController::topic_callback(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
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

rclcpp_action::GoalResponse PmapTrajectoryController::goal_received_callback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
    RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

    // Precondition: Running controller
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!validate_trajectory_msg(goal->trajectory))
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PmapTrajectoryController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
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
            get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

        // Mark the current goal as canceled
        auto action_res = std::make_shared<FollowJTrajAction::Result>();
        active_goal->setCanceled(action_res);
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PmapTrajectoryController::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
    // Update new trajectory
    {
        preempt_active_goal();
        auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

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

void PmapTrajectoryController::fill_partial_goal(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
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
        {
            if (
                std::find(
                    trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
                    joint_names_[index]) != trajectory_msg->joint_names.end())
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
                    if (!std::isnan(joint_command_interface_[0][index].get().get_value()))
                    {
                        // copy last command if cmd interface exists
                        it.positions.push_back(joint_command_interface_[0][index].get().get_value());
                    }
                    else
                    {
                        // copy current state if state interface exists
                        it.positions.push_back(joint_state_interface_[0][index].get().get_value());
                    }
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
}

void PmapTrajectoryController::sort_to_local_joint_order(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg)
{
    // rearrange all points in the trajectory message based on mapping
    std::vector<size_t> mapping_vector = joint_trajectory_controller::mapping(trajectory_msg->joint_names, joint_names_);
    auto remap = [this](
                                 const std::vector<double>& to_remap,
                                 const std::vector<size_t>& mapping) -> std::vector<double>
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
            auto map_index = mapping[index];
            output[map_index] = to_remap[index];
        }
        return output;
    };

    for (size_t index = 0; index < trajectory_msg->points.size(); ++index)
    {
        trajectory_msg->points[index].positions =
            remap(trajectory_msg->points[index].positions, mapping_vector);

        trajectory_msg->points[index].velocities =
            remap(trajectory_msg->points[index].velocities, mapping_vector);

        trajectory_msg->points[index].accelerations =
            remap(trajectory_msg->points[index].accelerations, mapping_vector);

        trajectory_msg->points[index].effort =
            remap(trajectory_msg->points[index].effort, mapping_vector);
    }
}

bool PmapTrajectoryController::validate_trajectory_point_field(
    size_t joint_names_size, const std::vector<double>& vector_field,
    const std::string& string_for_vector_field, size_t i, bool allow_empty) const
{
    if (allow_empty && vector_field.empty())
    {
        return true;
    }
    if (joint_names_size != vector_field.size())
    {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Mismatch between joint_names (%zu) and %s (%zu) at point #%zu.",
            joint_names_size, string_for_vector_field.c_str(), vector_field.size(), i);
        return false;
    }
    return true;
}

bool PmapTrajectoryController::validate_trajectory_msg(
    const trajectory_msgs::msg::JointTrajectory& trajectory) const
{
    // If partial joints goals are not allowed, goal should specify all controller joints
    if (!allow_partial_joints_goal_)
    {
        if (trajectory.joint_names.size() != dof_)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Joints on incoming trajectory don't match the controller joints.");
            return false;
        }
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

        const size_t joint_count = trajectory.joint_names.size();
        const auto& points = trajectory.points;
        // This currently supports only position, velocity and acceleration inputs
        if (allow_integration_in_goal_trajectories_)
        {
            const bool all_empty = points[i].positions.empty() && points[i].velocities.empty() &&
                                                         points[i].accelerations.empty();
            const bool position_error =
                !points[i].positions.empty() &&
                !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false);
            const bool velocity_error =
                !points[i].velocities.empty() &&
                !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, false);
            const bool acceleration_error =
                !points[i].accelerations.empty() &&
                !validate_trajectory_point_field(
                    joint_count, points[i].accelerations, "accelerations", i, false);
            if (all_empty || position_error || velocity_error || acceleration_error)
            {
                return false;
            }
        }
        else if (
            !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false) ||
            !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, true) ||
            !validate_trajectory_point_field(
                joint_count, points[i].accelerations, "accelerations", i, true) ||
            !validate_trajectory_point_field(joint_count, points[i].effort, "effort", i, true))
        {
            return false;
        }
    }
    return true;
}

void PmapTrajectoryController::add_new_trajectory_msg(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& traj_msg)
{
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

void PmapTrajectoryController::preempt_active_goal()
{
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal)
    {
        set_hold_position();
        auto action_res = std::make_shared<FollowJTrajAction::Result>();
        action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
        action_res->set__error_string("Current goal cancelled due to new incoming action.");
        active_goal->setCanceled(action_res);
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
}

void PmapTrajectoryController::set_hold_position()
{
    trajectory_msgs::msg::JointTrajectory empty_msg;
    empty_msg.header.stamp = rclcpp::Time(0);

    auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg);
    add_new_trajectory_msg(traj_msg);
}

void PmapTrajectoryController::resize_joint_trajectory_point(
    trajectory_msgs::msg::JointTrajectoryPoint& point, size_t size)
{
    point.positions.resize(size, 0.0);
}

}   // namespace pma_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pma_control::PmapTrajectoryController, controller_interface::ControllerInterface)
