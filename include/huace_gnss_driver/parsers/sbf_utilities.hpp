#pragma once

#include <cstdint>
#include <type_traits>

template <class T, class = void>
struct has_block_header : std::false_type
{
};

template <class T>
struct has_block_header<T, std::void_t<decltype(T::block_header)>> : std::true_type
{
};

/**
 * validValue
 * @brief Check if value is not set to Do-Not-Use
 */
template <typename T>
[[nodiscard]] bool validValue(T s)
{
    static_assert(std::is_same<uint16_t, T>::value ||
                  std::is_same<uint32_t, T>::value ||
                  std::is_same<float, T>::value || std::is_same<double, T>::value);
    if (std::is_same<uint16_t, T>::value)
    {
        return (s != static_cast<uint16_t>(65535));
    } else if (std::is_same<uint32_t, T>::value)
    {
        return (s != 4294967295u);
    } else if (std::is_same<float, T>::value)
    {
        return (!std::isnan(s) && (s != -2e10f));
    } else if (std::is_same<double, T>::value)
    {
        return (!std::isnan(s) && (s != -2e10));
    }
}