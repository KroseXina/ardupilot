#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_ADRC \
        LOG_ADRC_R_RATE, \
        LOG_ADRC_P_RATE

struct PACKED log_adrc {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float z1;
        float z2;
        float ez1;
        float u0;
};

#define LOG_STRUCTURE_FROM_ADRC   \
        { LOG_ADRC_R_RATE, sizeof(log_adrc), \
        "ADRR", "Qffff", "TimeUS,RZ1,RZ2,REZ1,RU0", "sE---", "F0000" }, \
        { LOG_ADRC_P_RATE, sizeof(log_adrc), \
        "ADRP", "Qffff", "TimeUS,PZ1,PZ2,PEZ1,PU0", "sE---", "F0000" }
