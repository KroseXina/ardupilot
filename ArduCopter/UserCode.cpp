#include "Copter.h"
#include "UserParameters.h"
#include "UserCode.h"

XT::XT_result XT::handle_recv_data(uint8_t *data, int len, uint8_t id)
{
	static parse_state parse = WAIT_HEAD1;
	static uint8_t payload[64];
	static uint8_t index = 0;
	static uint8_t length = 0;
	static uint16_t recv_rcr = 0;

	uint16_t cal_crc;

    for(int i=0;i<len;i++)
	{
		uint8_t byte = data[i];
		switch (parse)
		{
		case WAIT_HEAD1:
			if(byte == 0xAA)
				parse = WAIT_HEAD2;
			break;
		case WAIT_HEAD2:
			if(byte == 0x55) //是lora定位信息
				parse = WAIT_LENGTH;
			else if(byte == 0xFF)  //是mission通知信息
				parse = WAIT_ID;
			else
				parse = WAIT_HEAD1;
			break;
        	case WAIT_LENGTH:
            		length = byte;
            		index = 0;

            		if (length == 0 || length > sizeof(payload))
                		parse = WAIT_HEAD1;
            		else
                		parse = WAIT_PAYLOAD;
            		break;
		case WAIT_PAYLOAD:
			payload[index++] = byte;
			if(index >= length)
				parse = WAIT_CRC1;
			break;
		case WAIT_CRC1:
			recv_rcr = byte;
			parse = WAIT_CRC2;
			break;
		case WAIT_CRC2:
			parse = WAIT_HEAD1;
			recv_rcr |= ((uint16_t)byte << 8);
			//收到完整一帧，进行校验
			cal_crc = crc_ccitt(payload,length);
			if(cal_crc == recv_rcr)
			{
				//通过校验
				_rec_struct.node_id = payload[0];
				_rec_struct.lat = (int32_t)payload[1]
						|((int32_t)payload[2] << 8)
						|((int32_t)payload[3] << 16)
						|((int32_t)payload[4] << 24);
				_rec_struct.lon = (int32_t)payload[5]
						|((int32_t)payload[6] << 8)
						|((int32_t)payload[7] << 16)
						|((int32_t)payload[8] << 24);
				return RECV_LORA;
			}
			break;
		case WAIT_ID:
			_rec_id = byte;
			if((id != 0) && (_rec_id == id - 1)) //仅当id不为0,且收到了前一个id信息，才返回RECV_MISEND
				parse = WAIT_BOOL;
			else
				parse = WAIT_HEAD1;
			break;
		case WAIT_BOOL:
			parse = WAIT_HEAD1;
			if(byte == 0x01)
				return RECV_MISEND;
			break;
		}
	}
    
    return NONE;
}

void XT::publish_adsb()
{
		//gcs().send_text(MAV_SEVERITY_INFO,"XT receive lora.");
        AP_ADSB::adsb_vehicle_t vehicle{};
        vehicle.info.ICAO_address = _rec_struct.node_id + 1000;
        snprintf(vehicle.info.callsign,sizeof(vehicle.info.callsign),"XT_%02d",_rec_struct.node_id);
        vehicle.info.lat = _rec_struct.lat;
        vehicle.info.lon = _rec_struct.lon;
        vehicle.info.altitude = 100;
        vehicle.info.altitude_type = 0;
        vehicle.info.emitter_type = ADSB_EMITTER_TYPE_UAV;
        vehicle.info.flags = ADSB_FLAGS_VALID_COORDS |
                            ADSB_FLAGS_VALID_ALTITUDE |
                            ADSB_FLAGS_VALID_CALLSIGN |
                            ADSB_FLAGS_BARO_VALID;

        vehicle.last_update_ms = AP_HAL::millis();

        copter.adsb.handle_adsb_vehicle(vehicle);

		//整理targets列表
		bool found = false;
		for(int i=0; i<_target_count; ++i)
		{
			if(_targets[i].ICAO_address == vehicle.info.ICAO_address)
			{
				_targets[i] = vehicle.info;
				found = true;
				break;
			}
		}
		if(!found && _target_count < MAX_TARGETS)
		{
			_targets[_target_count ++] = vehicle.info;
		}
		//为_targets排序
		for (int i = 0; i < _target_count - 1; ++i)
		{
			for (int j = i + 1; j < _target_count; ++j)
			{
				if (_targets[j].ICAO_address < _targets[i].ICAO_address)
				{
					auto tmp = _targets[i];
					_targets[i] = _targets[j];
					_targets[j] = tmp;
				}
			}
		}
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // 使用scripting口接收自定义串口数据
    xt_uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Scripting,0);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
//10hz进程：处理串口相关，接收数据/发送mission end
void Copter::userhook_MediumLoop()
{
    if(xt_uart == nullptr) return;

	uint8_t xt_id = g2.user_parameters.get_xt_id();

	// 若收到recv_misend,进行自动任务。（等待至少5s执行）
	static uint16_t count = 0;
	static bool auto_mission = false;

	AP_Mission& mission = copter.mode_auto.mission;

	if(auto_mission)
	{
		count ++;
		if(count > 50)
		{
			count = 0;
			auto_mission = false;

			//判断当前是否有mission(AP中home点始终作为第一个点)
			if(mission.num_commands() > 1)
			{
				if(!copter.motors->armed())
				{
					copter.arming.arm(AP_Arming::Method::SCRIPTING);

					copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND);
					mission.start();
				}
			}
		}
	}

	// 若任务完成,发送misend,发送5次（1s发送一次）
	static bool vehicle_in_mission = false;
    if(copter.motors->armed() && (copter.flightmode == &copter.mode_auto))
    {
		vehicle_in_mission = true;
	}

	if(!copter.motors->armed())
	{
        if(vehicle_in_mission && (mission.state() == AP_Mission::MISSION_COMPLETE))
        {
			static uint16_t lcount = 0;
			static uint16_t total_count = 0;
			lcount ++;
			if(lcount >= 10)
			{
				lcount = 0;
				total_count ++;

				uint8_t frame[4];
				frame[0] = 0xAA;
				frame[1] = 0xFF;
				frame[2] = xt_id;
				frame[3] = 0x01;

				xt_uart->write(frame, 4);
			}
			if(total_count >= 5)
			{
				total_count = 0;
				vehicle_in_mission = false;
			}
		}
	}

	// 处理接收数据
    uint8_t buf[64];
    uint16_t n = MIN(xt_uart->available(),sizeof(buf));
    if(n == 0) return;
    
    xt_uart->read(buf, n);

    XT::XT_result result;
    result = xt.handle_recv_data(buf, n, xt_id);

    switch (result)
    {
    case XT::NONE :
        break;
    
    case XT::RECV_LORA :
		xt.publish_adsb();
        break;

	case XT::RECV_MISEND :
		auto_mission = true;
		break;
    }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
//依据信标建立航线
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
	if(ch_flag == RC_Channel::AuxSwitchPos::HIGH)
	{
		//GPS是否有效
    	if(copter.gps.status() < AP_GPS_FixType::FIX_3D)
		{
			gcs().send_text(MAV_SEVERITY_WARNING, "XT FUNC1: No valid GPS.");
			return;
		}
		
		AP_Mission& mission = copter.mode_auto.mission;
		//清除现有mission
		mission.clear();

		AP_Mission::Mission_Command cmd{};
		
		//1.takeoff(在设定第一条cmd时，AP_Mission会自动添加一个home作为index 0)
		cmd.id = MAV_CMD_NAV_TAKEOFF;
		cmd.p1 = 0;
		cmd.content.location = Location{0,0,300,Location::AltFrame::ABOVE_HOME};
		mission.add_cmd(cmd);

		//2.依据targets列表添加航点
		if(xt.targets_count() > 0)
		{
			for(int i = 0; i < xt.targets_count(); ++i)
			{
				cmd = {};
				cmd.id = MAV_CMD_NAV_LOITER_TIME;
				cmd.p1 = 5;
				cmd.content.location = Location{xt.target(i).lat,
												xt.target(i).lon,
												300, Location::AltFrame::ABOVE_HOME};
				mission.add_cmd(cmd);
			}
		}

		//3.RTL
		cmd = {};
		cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
		mission.add_cmd(cmd);

		mission.set_current_cmd(1);
	}
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
