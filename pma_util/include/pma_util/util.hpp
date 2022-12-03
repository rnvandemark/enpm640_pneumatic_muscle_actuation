#pragma once

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <vector>
#include <string>
#include <algorithm>

namespace pma_util
{
inline const char* callback_return_to_string(const rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn& cr)
{
    switch (cr)
    {
#define CR2STR_CASE(xxx) case rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::xxx: return #xxx
        CR2STR_CASE(SUCCESS);
        CR2STR_CASE(FAILURE);
        CR2STR_CASE(ERROR);
#undef CR2STR_CASE
    }
    return "UNKNOWN";
}

template <typename T>
inline bool list_contains(const std::vector<T>& list, const T& elem)
{
    return list.cend() != std::find(list.cbegin(), list.cend(), elem);
}

inline std::string get_delimited_list(
    const std::vector<std::string>& list,
    const std::string& delimiter)
{
    return list.empty() ? "" : std::accumulate(
        std::next(list.cbegin()),
        list.cend(),
        list[0],
        [&](const std::string& str, const std::string& t)
        {
            return std::move(str) + delimiter + t;
        }
    );
};
}
