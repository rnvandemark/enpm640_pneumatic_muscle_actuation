// Copyright 2021 Department of Engineering Cybernetics, NTNU
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

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pma_hardware/types/hardware_interface_type_values.hpp"

namespace pma_hardware
{
class PmaPlanar2DofInterfaceHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PmaPlanar2DofInterfaceHardware);

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

protected:
    const std::vector<std::string> command_interface_types_ =
    {
        pma_hardware::HW_IF_PRESSURE,
    };
    const std::vector<std::string> state_interface_types_ =
    {
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION,
        hardware_interface::HW_IF_EFFORT,
        pma_hardware::HW_IF_PRESSURE,
    };

    // Parameters for the RRBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;

    // Store the commands for the simulated robot
    std::vector<double> hw_commands_pressures_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    std::vector<double> hw_states_accelerations_;
    std::vector<double> hw_states_efforts_;
    std::vector<double> hw_states_pressures_;
};
}
