#pragma once
#include <stdint.h>

enum class BmsState : uint8_t {
    INIT    = 0,
    STARTUP,
    NORMAL,
    BALANCE,
    SLEEP,
    FAULT,
    CHARGING,
    DRIVE,
};