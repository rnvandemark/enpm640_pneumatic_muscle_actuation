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

#include "pma_hardware/interface/pma_planar_2dof_interface.hpp"

#include <cmath>
#include <limits>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pma_util/util.hpp"

#define PMAI_LOG(level, ...) RCLCPP_##level(rclcpp::get_logger("PmaPlanar2DofInterfaceHardware"), __VA_ARGS__)

namespace {
    std::vector<std::string> get_interface_info_list_names(const std::vector<hardware_interface::InterfaceInfo>& list)
    {
        std::vector<std::string> names;
        names.reserve(list.size());
        std::for_each(list.cbegin(),
                      list.cend(),
                      [&](const hardware_interface::InterfaceInfo& i)
                      {
                          names.push_back(i.name);
                      }
        );
        return names;
    }
}

namespace pma_hardware
{
std::vector<hardware_interface::CommandInterface> PmaPlanar2DofInterfaceHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(info_.joints[i].name, pma_hardware::HW_IF_PRESSURE, &hw_commands_pressures_[i]);
    }
    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> PmaPlanar2DofInterfaceHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, pma_hardware::HW_IF_PRESSURE, &hw_states_pressures_[i]);
    }
    return state_interfaces;
}

hardware_interface::CallbackReturn PmaPlanar2DofInterfaceHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    PMAI_LOG(INFO, "Entered on_init()");

    const size_t num_joints = info_.joints.size();

    hardware_interface::CallbackReturn rc = hardware_interface::SystemInterface::on_init(info);
    if (hardware_interface::CallbackReturn::SUCCESS != rc)
    {
        PMAI_LOG(ERROR, "Base class was unsuccessful!");
        goto END;
    }

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    
    hw_commands_pressures_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_states_positions_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_states_accelerations_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_states_pressures_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

    for (auto iter = info_.joints.cbegin(); iter != info_.joints.cend(); ++iter)
    {
        //
        // Confirm we have exactly the expected command and state interfaces on
        // each joint
        //

        const hardware_interface::ComponentInfo& joint = *iter;

        if (command_interface_types_.size() != joint.command_interfaces.size())
        {
            PMAI_LOG(
                ERROR,
                "Joint '%s' has %zu command interfaces. %zu expected.",
                joint.name.c_str(),
                joint.command_interfaces.size(),
                command_interface_types_.size()
            );
            rc = hardware_interface::CallbackReturn::ERROR;
            goto END;
        }

        const std::vector<std::string> command_interface_names = get_interface_info_list_names(joint.command_interfaces);
        if (!std::all_of(command_interface_types_.cbegin(),
                         command_interface_types_.cend(),
                         [&](const std::string& str)
                         {
                             return pma_util::list_contains(command_interface_names, str);
                         }
        ))
        {
            PMAI_LOG(
                ERROR,
                "Joint '%s' has [%s] command interfaces. Expected [%s].",
                joint.name.c_str(),
                pma_util::get_delimited_list(command_interface_names, ", ").c_str(),
                pma_util::get_delimited_list(command_interface_types_, ", ").c_str()
            );
            rc = hardware_interface::CallbackReturn::ERROR;
            goto END;
        }

        if (state_interface_types_.size() != joint.state_interfaces.size())
        {
            PMAI_LOG(
                ERROR,
                "Joint '%s' has %zu state interfaces. %zu expected.",
                joint.name.c_str(),
                joint.state_interfaces.size(),
                state_interface_types_.size()
            );
            rc = hardware_interface::CallbackReturn::ERROR;
            goto END;
        }

        const std::vector<std::string> state_interface_names = get_interface_info_list_names(joint.state_interfaces);
        if (!std::all_of(state_interface_types_.cbegin(),
                         state_interface_types_.cend(),
                         [&](const std::string& str)
                         {
                             return pma_util::list_contains(state_interface_names, str);
                         }
        ))
        {
            PMAI_LOG(
                ERROR,
                "Joint '%s' has [%s] state interfaces. Expected [%s].",
                joint.name.c_str(),
                pma_util::get_delimited_list(state_interface_names, ", ").c_str(),
                pma_util::get_delimited_list(state_interface_types_, ", ").c_str()
            );
            rc = hardware_interface::CallbackReturn::ERROR;
            goto END;
        }
    }

END:
    PMAI_LOG(INFO, "on_init() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

hardware_interface::CallbackReturn PmaPlanar2DofInterfaceHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    PMAI_LOG(INFO, "Entered on_activate()");

    hardware_interface::CallbackReturn rc = hardware_interface::CallbackReturn::SUCCESS;

    // Set some default values
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        if (std::isnan(hw_commands_pressures_[i]))
        {
            hw_commands_pressures_[i] = 0;
        }
        if (std::isnan(hw_states_positions_[i]))
        {
            hw_states_positions_[i] = 0;
        }
        if (std::isnan(hw_states_velocities_[i]))
        {
            hw_states_velocities_[i] = 0;
        }
        if (std::isnan(hw_states_accelerations_[i]))
        {
            hw_states_accelerations_[i] = 0;
        }
        if (std::isnan(hw_states_efforts_[i]))
        {
            hw_states_efforts_[i] = 0;
        }
        if (std::isnan(hw_states_pressures_[i]))
        {
            hw_states_pressures_[i] = 0;
        }
    }

END:
    PMAI_LOG(INFO, "on_activate() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

hardware_interface::CallbackReturn PmaPlanar2DofInterfaceHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    PMAI_LOG(INFO, "Entered on_deactivate()");

    hardware_interface::CallbackReturn rc = hardware_interface::CallbackReturn::SUCCESS;

END:
    PMAI_LOG(INFO, "on_deactivate() rc=%s", pma_util::callback_return_to_string(rc));
    return rc;
}

hardware_interface::return_type PmaPlanar2DofInterfaceHardware::read(const rclcpp::Time& /*time*/,
                                                                     const rclcpp::Duration& period)
{
//    for (size_t i = 0; i < hw_states_positions_.size(); i++)
//    {
//        hw_states_accelerations_[i] = hw_commands_accelerations_[i];
//        hw_states_velocities_[i] += (hw_states_accelerations_[i] * period.seconds()) / hw_slowdown_;
//        hw_states_positions_[i] += (hw_states_velocities_[i] * period.seconds()) / hw_slowdown_;
//        // TODO: log?
//    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PmaPlanar2DofInterfaceHardware::write(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/)
{
    // TODO

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PmaPlanar2DofInterfaceHardware::prepare_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // Not supported!
    return hardware_interface::return_type::ERROR;
}
}

#undef PMAI_LOG

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pma_hardware::PmaPlanar2DofInterfaceHardware, hardware_interface::SystemInterface)
