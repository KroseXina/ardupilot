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
}

void XT::clear_map()
{
	memset(_matched, 0, sizeof(_matched));
	memset(_map_icao, 0, sizeof(_map_icao));
}

void XT::match_target(AP_Mission::Mission_Command &cmd)
{
	float best_dist = FLT_MAX;
	int16_t best_target = -1;
	//遍历所有的targets，寻找与目标航点最近邻的点进行匹配
	for(uint16_t i=0;i<_target_count;i++)
	{
		if(_matched[i]) continue;
		Location loc;
		loc.lat = _targets[i].lat;
		loc.lng = _targets[i].lon;
		float distance = cmd.content.location.get_distance(loc);
		if(distance < best_dist)
		{
			best_dist = distance;
			best_target = i;
		}
	}

	if(best_target >= 0 && best_dist < MATCH_DISTANCE)
	{
		_map_icao[cmd.index] = _targets[best_target].ICAO_address;
		_matched[best_target] = true;
	}
}

const mavlink_adsb_vehicle_t *XT::find_tar_from_map(uint32_t adress)
{
	uint32_t left = 0;
	uint32_t right = _target_count - 1;

	while(left <= right)
	{
		uint32_t mid = (left + right) >> 1;
		if(_targets[mid].ICAO_address == adress)
		{
			return &_targets[mid];
		}

		if(_targets[mid].ICAO_address < adress)
		{
			left = mid + 1;
		}
		else
		{
			right = mid - 1;
		}
	}

	return nullptr;
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    //使用scripting口接收自定义串口数据
    xt_uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Scripting,0);
	//使用scrip1发送舵机PWM
	SRV_Channels::set_output_pwm(SRV_Channel::Function::k_scripting1, 1000);
	//初始化XTCAN
#if HAL_XTCAN_ENABLED
    uint8_t num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < num_drivers; i++)
    {
        AP_XTCAN *ap_xtcan = AP_XTCAN::get_xtcan(i);
        if(ap_xtcan != nullptr)
        {
            xtcan = ap_xtcan;
            break;
        }
    } 
#endif
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
/* 10hz：
1.处理串口,发布ADSB；
2.处理CAN口数据(称重数据)
*/
void Copter::userhook_MediumLoop()
{
	//处理串口数据
    if(xt_uart != nullptr)
	{
		uint8_t buf[64];
		uint16_t n = MIN(xt_uart->available(),sizeof(buf));
		if(n > 0)
		{
			xt_uart->read(buf, n);

			XT::XT_result result;
			result = xt.handle_recv_data(buf, n, xt.xt_id);
			switch (result)
			{
			case XT::NONE :
				break;
			
			case XT::RECV_LORA :
				xt.publish_adsb();
				break;
			}
		}
	}

	//处理称重数据
#if HAL_XTCAN_ENABLED
	if(xtcan != nullptr)
	{
		AP_XTCAN::XTCAN_Frame frame;
		static uint16_t count = 0;
		if(xtcan->xtcan_get_frame(frame))
		{
			if(frame.frametype == AP_XTCAN::XTCAN_Frame::FrameType::WEIGHT)
			{
				xt.WetData = frame.data;
				xt.WetValid = true;
				count = 0;
			}
		}
		else //设置3s超时
		{
			count ++;
			if(count > 30)
			{
				xt.WetValid = false;
				count = 0;
			}
		}
	}
#endif
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
/* 1hz：
1.发布mission end，以及自动mission；
2.在使用了信标建立航线的前提下，gcs上传航点后进行march；
3.每次到达航点打开舵机，自动修订下一个航点；
4.根据参数设置，打印重量信息。
*/
void Copter::userhook_SuperSlowLoop()
{
	xt.xt_id = g2.user_parameters.get_xt_id();

	AP_Mission& mission = copter.mode_auto.mission;

	//飞机是否进入了mission
	static bool vehicle_in_mission = false;
    if(copter.motors->armed() && (copter.flightmode == &copter.mode_auto))
    {
		vehicle_in_mission = true;

		//如果是由信标建立的任务，进行投喂及下一点修正
		if(xt.xt_build_mis)
		{
			AP_Mission::Mission_Command current;
			AP_Mission::Mission_Command next;

			current = mission.get_current_nav_cmd();
			if(current.id == MAV_CMD_NAV_LOITER_TIME)
			{
				if(copter.wp_nav->reached_wp_destination())
				{
					//等待约1s后打开舵机
					static bool wait = true;
					if(wait)
					{
						static uint8_t count = 0;
						count ++;
						if(count > 1)
						{
							SRV_Channels::set_output_pwm(SRV_Channel::Function::k_scripting1, 2000);
							count = 0;
							wait = false;
						}
					}

					//如果下一个点还是投喂，进行修正
					if(mission.read_cmd_from_storage(current.index + 1, next) && (next.id == MAV_CMD_NAV_LOITER_TIME))
					{
						//更新下一个点的坐标信息
						uint32_t icao = xt.get_map_icao(next.index);
						if(icao == 0) return;
						const mavlink_adsb_vehicle_t *target = xt.find_tar_from_map(icao);
						if(target == nullptr)
						{
							gcs().send_text(MAV_SEVERITY_ERROR,"XT：Target Lost.");
							return;
						}
						Location new_nextLoc = next.content.location;
						new_nextLoc.lat = target->lat;
						new_nextLoc.lng = target->lon;

						//误差超过2m时进行修正					
						if(next.content.location.get_distance(new_nextLoc) > 2.0f)
						{
							next.content.location.lat = new_nextLoc.lat;
							next.content.location.lng = new_nextLoc.lng;
							if(!mission.replace_cmd(next.index, next))
							{
								gcs().send_text(MAV_SEVERITY_WARNING,"XT：Change WP Failed.");
								return;
							};
						}
					}

					//TODO：关闭舵机，然后让mode_auto中verify_loiter_time判断为true
				}
			}
		}
	}

	if(!copter.motors->armed())
	{
		//未解锁的前提下，接收gcs是否上传了航点
		if(mission.xt_wp_trans)
		{
			//gcs().send_text(MAV_SEVERITY_INFO,"XT:WP Trans Complete.");
			//对新航点和targets进行匹配
			if(xt.xt_build_mis)
			{
				//先清理当前匹配关系
				xt.clear_map();
				AP_Mission::Mission_Command cmd;
				for(uint16_t i = 0; i < mission.num_commands(); i++)
				{
					mission.read_cmd_from_storage(i, cmd);
					if(cmd.id == MAV_CMD_NAV_LOITER_TIME)
					{
						xt.match_target(cmd);
					}
				}
			}
			mission.xt_wp_trans = false;
		}

		//完成了飞行任务后执行
        if(vehicle_in_mission && (mission.state() == AP_Mission::MISSION_COMPLETE))
        {
			//如果是信标建立的任务，注销_xt_build_mis
			if(xt.xt_build_mis)
			{
				xt.xt_build_mis = false;
			}
			vehicle_in_mission = false;
		}
	}

	//打印重量信息
	if(g2.user_parameters.get_enPrtWET() != 0)
	{
		if(xt.WetValid)
			gcs().send_text(MAV_SEVERITY_INFO,"XT:weight = %2f.",xt.WetData);
		else
			gcs().send_text(MAV_SEVERITY_WARNING,"XT:weight data timeout!");
	}
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
		cmd.content.location = copter.current_loc;
		cmd.content.location.alt = 300;
		cmd.content.location.relative_alt = static_cast<uint8_t>(Location::AltFrame::ABOVE_HOME);
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
		
		xt.xt_build_mis = true;
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
