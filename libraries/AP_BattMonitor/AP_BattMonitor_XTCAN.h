#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#if HAL_XTCAN_ENABLED
#include <AP_XTCAN/AP_XTCAN.h>

class AP_BattMonitor_XTCAN : public AP_BattMonitor_Backend
{
public:
    // Constructor
    AP_BattMonitor_XTCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params,uint8_t instance);

    void init() override;
    void read() override;

    bool has_current() const override {return true;}
    bool has_temperature() const override {return true;}

    bool capacity_remaining_pct(uint8_t &percentage) const override;

private:
    uint8_t _instance;
    AP_XTCAN *_xtcan{nullptr};

    uint8_t _soc{};
};

#endif //HAL_XTCAN_ENABLED