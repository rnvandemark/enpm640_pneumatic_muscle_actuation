// Copyright (c) 2021 ros2_control Development Team
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

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/interpolation_methods.hpp"
#include "joint_trajectory_controller/tolerances.hpp"
#include "pma_control/trajectory_util/pma_trajectory_types.hpp"
#include "pma_hardware/types/hardware_interface_type_values.hpp"
#include "rclcpp_action/server.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"

// Forward declarations
namespace rclcpp_action {
    template <typename ActionT>
    class ServerGoalHandle;
}
namespace joint_trajectory_controller {
    class Trajectory;
}
namespace pma_hardware {
    struct PneumaticMuscleControlConfiguration;
    struct PneumaticMuscleSegmentConfiguration;
    struct PneumaticMuscleSegmentTelemetry;
}

namespace pma_control
{
class SlidingMode2DofPressureTrajectoryController : public controller_interface::ControllerInterface
{
protected:
    template <typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

    using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
    using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
    using StatePublisherPtr = std::unique_ptr<StatePublisher>;

    using FollowJointTrajAction = control_msgs::action::FollowJointTrajectory;
    using FollowJointTrajActionGoalSharedPtr = std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajAction>>;
    using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJointTrajAction>;
    using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
    using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

    using JointTrajectoryMsg = trajectory_msgs::msg::JointTrajectory;
    using JointTrajectoryMsgSharedPtr = std::shared_ptr<JointTrajectoryMsg>;

public:
    SlidingMode2DofPressureTrajectoryController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) override;

protected:
    /**
     * The list of command interface types required by this controller.
     */
    const std::vector<std::string> command_interface_types_ =
    {
        pma_hardware::HW_IF_PRESSURE,
    };
    /**
     * The list of state interface types required by this controller.
     */
    const std::vector<std::string> state_interface_types_ =
    {
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION,
        hardware_interface::HW_IF_EFFORT,
        pma_hardware::HW_IF_PRESSURE,
    };

    /**
     * The ordered list of names of the joints in the robot we are controlling.
     * This also acts as the list of commandable joints.
     */
    std::vector<std::string> joint_names_;
    /**
     * The degrees of freedom of the robot we are controlling. Once
     * initialized, this must be 2.
     */
    size_t dof_;
    /**
     * The default tolerances of each segment in the robot arm.
     */
    joint_trajectory_controller::SegmentTolerances default_tolerances_;

    /**
     * The desired rate, in Hz, at which this controller will write joint
     * trajectory controller state updates on @a state_publisher_. The default
     * value is 20.0.
     */
    double state_publish_rate_;
    /**
     * The period described by @a state_publish_rate_.
     */
    rclcpp::Duration state_publisher_period_;

    /**
     * The desired rate, in Hz, at which this controller will monitor changes
     * to the action status of accepted trajectory goals. The default value is
     * 50.0.
     */
    double action_monitor_rate_;
    /**
     * The period described by @a action_monitor_rate_.
     */
    rclcpp::Duration action_monitor_period_;

    /**
     * The interpolation method that is to be used to interpolate joint-space
     * trajectories. The default value is DEFAULT_INTERPOLATION.
     */
    joint_trajectory_controller::interpolation_methods::InterpolationMethod interpolation_method_;

    /**
     * The scalar magnitude of acceleration due to gravity, in m/s^2. Defaults
     * to 9.81.
     */
    double acceleration_gravity_;

    /**
     * The k1 design constant for sliding-mode control. Defaults to 1.0.
     */
    double sliding_mode_control_k_1_;

    /**
     * The k2 design constant for sliding-mode control. Defaults to 1.0.
     */
    double sliding_mode_control_k_2_;

    /**
     * The Gamma1 design constant for sliding-mode control. Defaults to 1.0.
     */
    double sliding_mode_control_Gamma_1_;

    /**
     * The Gamma2 design constant for sliding-mode control. Defaults to 1.0.
     */
    double sliding_mode_control_Gamma_2_;

    /**
     * The mu1 design constant for sliding-mode control. Defaults to 1.0.
     */
    double sliding_mode_control_mu_1_;

    /**
     * The mu2 design constant for sliding-mode control. Defaults to 1.0.
     */
    double sliding_mode_control_mu_2_;

    /**
     * These command interfaces are defined as the types in
     * @a command_interface_types_. For each type, the interfaces are ordered
     * so that i-th position matches i-th index in joint_names_.
     */
    InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
    /**
     * These state interfaces are defined as the types in
     * @a state_interface_types_. For each type, the interfaces are ordered so
     * that i-th position matches i-th index in joint_names_.
     */
    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

    // Preallocate variables used in the realtime update() function
    PneumaticMuscleActuatorTrajectoryPoint state_current_;
    PneumaticMuscleActuatorTrajectoryPoint state_desired_;
    PneumaticMuscleActuatorTrajectoryPoint state_error_;

    /**
     * The last commanded shoulder and elbow pressures (the first and second
     * elements of the list, respectively).
     */
    PressureList last_commanded_pressures_;

    // TODO(karsten1987): eventually activate and deactivate subscriber directly when its supported
    bool subscriber_is_active_ = false;
    rclcpp::Subscription<JointTrajectoryMsg>::SharedPtr joint_command_subscriber_ = nullptr;

    realtime_tools::RealtimeBuffer<JointTrajectoryMsgSharedPtr> traj_msg_external_point_ptr_;
    std::shared_ptr<joint_trajectory_controller::Trajectory>* traj_point_active_ptr_ = nullptr;
    std::shared_ptr<joint_trajectory_controller::Trajectory> traj_external_point_ptr_ = nullptr;
    /**
     * A simple 'home' pose, (re)set during on_activate().
     */
    std::shared_ptr<joint_trajectory_controller::Trajectory> traj_home_point_ptr_ = nullptr;
    JointTrajectoryMsgSharedPtr traj_msg_home_ptr_ = nullptr;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_;
    StatePublisherPtr state_publisher_;
    rclcpp::Time last_state_publish_time_;

    rclcpp_action::Server<FollowJointTrajAction>::SharedPtr action_server_;
    /// Currently active action goal, if any
    RealtimeGoalHandleBuffer rt_active_goal_;
    rclcpp::TimerBase::SharedPtr goal_handle_timer_;

    /**
     * The collection of static configuration parameters specific to the
     * sliding-mode control law implemented by this controller.
     */
    std::shared_ptr<pma_hardware::PneumaticMuscleControlConfiguration> pm_control_configuration;
    /**
     * A list where each element is the collection of static configuration
     * parameters of a single segment in the pneumatic muscle robot arm.
     */
    std::vector<pma_hardware::PneumaticMuscleSegmentConfiguration> pm_segment_configurations;
    /**
     * A list where each element is the up-to-date telemetry of a single
     * segment in the pneumatic muscle robot arm.
     */
    std::vector<pma_hardware::PneumaticMuscleSegmentTelemetry> pm_segment_telemetries;

    // callback for topic interface
    void topic_callback(const JointTrajectoryMsgSharedPtr msg);

    // callbacks for action_server_
    rclcpp_action::GoalResponse goal_received_callback(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FollowJointTrajAction::Goal> goal
    );
    rclcpp_action::CancelResponse goal_cancelled_callback(const FollowJointTrajActionGoalSharedPtr goal_handle);
    void goal_accepted_callback(FollowJointTrajActionGoalSharedPtr goal_handle);

    // fill trajectory_msg so it matches joints controlled by this controller
    // positions set to current position, velocities, accelerations and efforts to 0.0
    void fill_partial_goal(JointTrajectoryMsgSharedPtr trajectory_msg) const;
    // sorts the joints of the incoming message to our local order
    void sort_to_local_joint_order(JointTrajectoryMsgSharedPtr trajectory_msg);
    bool validate_trajectory_msg(const JointTrajectoryMsg& trajectory) const;
    void add_new_trajectory_msg(const JointTrajectoryMsgSharedPtr& traj_msg);
    bool validate_trajectory_point_field(
        size_t joint_names_size,
        const std::vector<double>& vector_field,
        const std::string& string_for_vector_field,
        size_t i,
        bool allow_empty
    ) const;

    void preempt_active_goal();
    void set_hold_position();

    void read_state_from_hardware(PneumaticMuscleActuatorTrajectoryPoint& state);

    bool reset();
};
}
