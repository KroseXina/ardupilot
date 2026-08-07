#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_XTCAN_ENABLED
#include "AP_XTCAN.h"
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_XTCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: Own node ID
    // @Description: XTCAN node ID used by the driver itself on this network
    // @Range: 1 125
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_XTCAN, _xtcan_node, 1),

    AP_GROUPEND
};

AP_XTCAN::AP_XTCAN(const int driver_index) :
_driver_index(driver_index)
{
    AP_Param::setup_object_defaults(this,var_info);
}

AP_XTCAN::~AP_XTCAN()
{

}

AP_XTCAN *AP_XTCAN::get_xtcan(uint8_t driver_index)
{
if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::XTCAN) {
        return nullptr;
    }
    return static_cast<AP_XTCAN*>(AP::can().get_driver(driver_index));
}

bool AP_XTCAN::add_interface(AP_HAL::CANIface* can_iface)
{
    if (_can_iface != nullptr) {
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        return false;
    }

    if (!_can_iface->is_initialized()) {
        return false;
    }

    if (!_can_iface->set_event_handle(&sem_handle)) {
        return false;
    }
    return true;
}

void AP_XTCAN::init(uint8_t driver_index)
{
    _driver_index = driver_index;

    if (_initialized) {
        return;
    }

    snprintf(_thread_name, sizeof(_thread_name), "XTCAN_%u", driver_index);
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_XTCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        return;
    }

    _initialized = true;
}

bool AP_XTCAN::prearm_check(char* reason, uint8_t reason_len)
{
    return true;
}

bool AP_XTCAN::write_frame(AP_HAL::CANFrame &out_frame, uint32_t timeout_us)
{
    if(!_initialized) return false;

    bool read_select = false;
    bool write_select = true;
    const uint64_t deadline_us = AP_HAL::micros64() + timeout_us;
    bool ret = _can_iface->select(read_select, write_select, &out_frame, deadline_us);

    if (!ret || !write_select) {
        return false;
    }

    return (_can_iface->send(out_frame, deadline_us, AP_HAL::CANIface::AbortOnError) == 1);
}

bool AP_XTCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint32_t timeout_us)
{
    if(!_initialized) return false;

    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, AP_HAL::micros64() + timeout_us);

    if (!ret || !read_select) {
        // No frame available
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

void AP_XTCAN::loop()
{
    AP_HAL::CANFrame rxFrame {};

    while(true)
    {
        if(!_initialized)
        {
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        hal.scheduler->delay_microseconds(1000);

        while(read_frame(rxFrame,0))
        {
            static uint16_t cor = 0;
            static uint32_t total_weight = 0;
            static uint8_t count = 0;
            uint32_t avg_weight = 0;
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XT:id = ""%" PRIu32,rxFrame.id);
            uint32_t id = rxFrame.id & AP_HAL::CANFrame::MaskExtID; //扩展帧，协议会将最高位置为1,需要还原&0x1FFFFFFF
            if((id >> 4) == 0x8132) //称重模块(id分为0x81326与0x81320)
            {
                switch (id & 0xF)
                {
                    case 0x6:   //0x81326,称重数据中的修正部分
                        if(count > 0) //先对上一组进行修正
                        {
                            avg_weight = (total_weight / count) + cor;
                            //加锁并赋值
                            WITH_SEMAPHORE(_sem);
                            _xtcan_frame.frametype = XTCAN_Frame::FrameType::WEIGHT;
                            _xtcan_frame.data = avg_weight / 1000.f;
                            _new_wdata = true;
                        }
                        cor = (rxFrame.data[5] << 8 ) | rxFrame.data[4];
                        total_weight = 0;
                        count = 0;
                        break;
                    case 0x0:   //0x81320,称重数据
                        total_weight += (rxFrame.data[6] << 16 ) | (rxFrame.data[5] << 8 ) | rxFrame.data[4];
                        count ++;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

bool AP_XTCAN::xtcan_get_frame(XTCAN_Frame &frame)
{
    //加锁并读取
    WITH_SEMAPHORE(_sem);
    if(!_new_wdata) return false;

    frame = _xtcan_frame;
    _new_wdata = false;

    return true;
}

#endif //HAL_XTCAN_ENABLED