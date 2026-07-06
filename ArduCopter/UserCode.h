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

    /* 处理串口接收的数据 */
    XT_result handle_recv_data(uint8_t *data, int len, uint8_t id);

    /* 发布为ADSB信息，并维护&排序targets列表 */
    void publish_adsb();

    /* 返回总的targets数量 */
    uint16_t targets_count() {return _target_count;}

    /* 返回指定index的target信息 */
    const mavlink_adsb_vehicle_t &target(uint8_t i) {return _targets[i];}

    /* gcs修改上传后，清理map */
    void clear_map();

    /* 将修改后的loiter航点与初始targets进行匹配，保存到map_icao */
    void match_target(AP_Mission::Mission_Command &cmd);

    /* 返回存储在map_icao中的ICAO_adress值 */
    uint32_t get_map_icao(uint32_t i) {return _map_icao[i];}

    /* 根据map_icao反向查找对应的targets.因targets是依据icao排序的，使用二分查找 */
    const mavlink_adsb_vehicle_t *find_tar_from_map(uint32_t adress);

    bool xt_build_mis{false};
    bool auto_mission{false};
    uint8_t xt_id{0};

private:
    static constexpr uint8_t MAX_TARGETS = 32;
    static constexpr float MATCH_DISTANCE = 3.0f;
    mavlink_adsb_vehicle_t _targets[MAX_TARGETS];
    uint16_t _target_count{0};

    //存储gcs修改重新上传后的index对应的icao_adress
    uint32_t _map_icao[MAX_TARGETS]{0};
    bool _matched[MAX_TARGETS]{false};

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