#pragma once

#include <stdint.h>
#include <AP_ADSB/AP_ADSB.h> 

class XT
{
public:
    enum XT_result
    {
        NONE = 0,
        RECV_LORA = 1,
        RECV_MISEND = 2
    };

    XT_result handle_recv_data(uint8_t *data, int len, uint8_t id);
    void publish_adsb();
    uint16_t targets_count() {return _target_count;}
    const mavlink_adsb_vehicle_t &target(uint8_t i) {return _targets[i];}

private:
    static constexpr uint8_t MAX_TARGETS = 32;
    mavlink_adsb_vehicle_t _targets[MAX_TARGETS];
    uint16_t _target_count{0};

    /* 串口通信定义 */
    /* 与lora模块的通信协议定义(uint8_t)：0xAA 0x55 length payload CRC_L CRC_H*/
    /* lora传输的payload结构体,长度为1+4+4=9字节，整帧长度为14字节 */
    /* 传递mission已完成的通信协议定义(uint8_t): 0XAA 0xFF id bool */
    /* payload为0x00或0x01，整帧长度4字节 */
    struct lora_struct
    {
        uint8_t node_id;
        int32_t lat; //[degE7] Latitude
        int32_t lon; //[degE7] Longitude
    };

    /* 定义状态机进行拼包 */
    enum parse_state
    {
        WAIT_HEAD1,
        WAIT_HEAD2,
        WAIT_LENGTH,
        WAIT_PAYLOAD,
        WAIT_CRC1,
        WAIT_CRC2,
        WAIT_ID,
        WAIT_BOOL
    };

    lora_struct _rec_struct;
    uint8_t _rec_id{0};

    uint16_t crc_ccitt(const uint8_t *data, uint8_t len)
    {
        uint16_t crc = 0xFFFF;
        for(int i=0;i<len;i++)
        {
            crc ^= (uint16_t)data[i] << 8;
            for(int j=0;j<8;j++)
            {
                if(crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }
};