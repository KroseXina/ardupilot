#pragma once

/* 
* 实现对自定义CAN协议的解析
* 定义为一级CAN驱动，与DroneCAN不能同时存在于一个物理CAN口 
*/

#include <AP_HAL/AP_HAL.h>

#if HAL_XTCAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_Param/AP_Param.h>

class AP_XTCAN : public AP_CANDriver
{
public:
    AP_XTCAN(const int driver_index);
    ~AP_XTCAN();

    static const struct AP_Param::GroupInfo var_info[];

    // Return XTCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_XTCAN *get_xtcan(uint8_t driver_index);

    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // test if the Piccolo CAN driver is ready to be armed
    bool prearm_check(char* reason, uint8_t reason_len);

    struct XTCAN_Frame
    {
        enum class FrameType : uint8_t
        {
            NONE = 0,
            WEIGHT = 1,
        };

        FrameType frametype;
        float data;
    };

    bool xtcan_get_frame(XTCAN_Frame &frame);

private:
    void loop();
    bool write_frame(AP_HAL::CANFrame &out_frame, uint32_t timeout_us);
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint32_t timeout_us);

    //parameters
    AP_Int8 _xtcan_node;

    bool _initialized{false};
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_BinarySemaphore sem_handle;
    HAL_Semaphore _sem;

    XTCAN_Frame _xtcan_frame{};
    bool _new_wdata{false};
};

#endif //HAL_XTCAN_ENABLED