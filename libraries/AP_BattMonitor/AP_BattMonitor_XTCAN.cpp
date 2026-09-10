#include <AP_HAL/AP_HAL.h>

#if HAL_XTCAN_ENABLED
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_XTCAN.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>

#include <AP_XTCAN/AP_XTCAN.h>

AP_BattMonitor_XTCAN::AP_BattMonitor_XTCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params, uint8_t instance)
    : AP_BattMonitor_Backend(mon, mon_state, params),
    _instance(instance)
{

}

void AP_BattMonitor_XTCAN::init()
{
#if HAL_XTCAN_ENABLED
    uint8_t num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < num_drivers; i++)
    {
        AP_XTCAN *ap_xtcan = AP_XTCAN::get_xtcan(i);
        if(ap_xtcan != nullptr)
        {
            _xtcan = ap_xtcan;
            break;
        }
    } 
#endif
    _state.healthy = false;
}

void AP_BattMonitor_XTCAN::read()
{
    if(_xtcan == nullptr) {
        return;
    }

    AP_XTCAN::XTCAN_BAT bat{};
    const uint32_t tnow = AP_HAL::micros();
    const uint32_t dt_us = tnow - _state.last_time_micros;

    if(!_xtcan->xtcan_get_bat_info(bat,_instance)) {
        if(dt_us > 5*1e6) { //5s超时 
            _state.healthy = false;
        }
        return;
    }
    
    _state.voltage = bat.voltage;
    _state.healthy = true;

    _state.current_amps = bat.current;
    update_consumed(_state, dt_us);

    _state.temperature = bat.temperature;
    _state.has_state_of_health_pct = true;
    _state.state_of_health_pct = (uint8_t)bat.soh;

    _state.last_time_micros = tnow;

    _soc = (uint8_t)bat.soc;
}

bool AP_BattMonitor_XTCAN::capacity_remaining_pct(uint8_t &percentage) const
{
    if(!_state.healthy) {
        return false;
    }

    percentage = _soc;
    return true;
}

#endif //HAL_XTCAN_ENABLED