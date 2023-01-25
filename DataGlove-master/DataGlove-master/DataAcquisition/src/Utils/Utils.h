#pragma once
#include <stdint.h>

template <typename T1, typename T2>
auto mod(T1 x, T2 y);

template <typename T>
T mean(T array[], uint32_t len);

template <typename T>
T median(T array[], uint32_t len);

template <typename T>
T norm(T array[], uint32_t len);

template <typename T>
T abs_tp(T n);

#include "Utils.tpp"
