#include "open_tel_mavlink.h"//  qq:624668529

#include "mavlink_usart_fifo.h"
#include "define.h"
#include "stdint.h"
#include "led.h"
#include "lcd.h"
#include "delay.h"


mavlink_gps_raw_int_t GPS_data;

////Add By BigW
typedef uint8_t bool;
//typedef struct {
//    char c;
//} prog_char_t;
//	
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
int8_t control_mode = STABILIZE;

mavlink_channel_t           chan;
mavlink_heartbeat_t         heartbeat;
mavlink_attitude_t          attitude;
mavlink_mission_item_t 			guided_target;
mavlink_mission_count_t   						 mission_count;
mavlink_mission_request_t  						 mission_request;
mavlink_request_data_stream_t					 tx_request_data_stream;
mavlink_vfr_hud_t											 rx_vfr_hud;
mavlink_request_data_stream_t					 tx_request_data_stream;
mavlink_mission_ack_t                  mission_ack;
mavlink_rc_channels_raw_t              rc_input;


////End Add By BigW
//// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// this costs us 51 bytes, but means that low priority
// messages don't block the CPU
static mavlink_statustext_t pending_status;
// true when we have received at least 1 MAVLink packet
//static bool mavlink_active;
// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false
void handleMessage(mavlink_message_t* msg);
/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */
static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }
    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif
    // we are armed if we are not initialising
    if (0){//motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static void NOINLINE send_requst(mavlink_channel_t chan)
{
    //Vector3f omega_I = ahrs.get_gyro_drift();
	
    mavlink_msg_request_data_stream_send(
        chan,
        mavlink_system.sysid,//requst_sys_id,//omega_I.x,
				mavlink_system.compid,//requst_commond_id,
				tx_request_data_stream.req_stream_id,
				tx_request_data_stream.req_message_rate,
        tx_request_data_stream.start_stop      
        );
}

static void NOINLINE send_mission_item_home(mavlink_channel_t chan)
{
	mavlink_msg_mission_item_send(
		chan,
		mavlink_system.sysid,
		mavlink_system.compid,
		0,//航点序号
		MAV_FRAME_GLOBAL,//坐标系，地面站发送时只有
		16,//MAV_CMD_NAV_WAYPOINT
		0,//不设为当前航点
		0,//自动开始下个航点
		0.0,//驻留时间，秒
	  0.0,//航点半径
		0.0,//顺时针
		0.0,//
		34.000000,//	经度		
		113.000000,//纬度
		100.0	//高度
		);
}

static void NOINLINE send_mission_item_waypoint(mavlink_channel_t chan)
{
	mavlink_msg_mission_item_send(
		chan,
		mavlink_system.sysid,
		mavlink_system.compid,
		guided_target.seq,//航点序号
		MAV_FRAME_GLOBAL_RELATIVE_ALT,
	//坐标系，地面站发送时只有home点是0(MAV_FRAME_GLOBAL)
	//其余航点均为3(MAV_FRAME_GLOBAL_RELATIVE_ALT)	                           
		16,//MAV_CMD_NAV_WAYPOINT
		0,//设为当前航点
		0,//不自动开始下个航点
		0.0,//驻留时间，秒
	  0.0,//航点半径
		0.0,//顺时针
		0.0,//指向设定
	//地面站发送时普通航点的这四个参数均为0
		guided_target.x,//	纬度		
		guided_target.y,//  经度
		guided_target.z	//高度
		);
}


//注意count位置,除了我自己,已经误导过2个人了QAQ
static void NOINLINE send_mission_count(mavlink_channel_t chan)
{
	mavlink_msg_mission_count_send(
			chan,
//			2,
			mavlink_system.sysid,
			mavlink_system.compid,
			3//航点数量,带上home
	);
}

static void NOINLINE send_mission_ack(mavlink_channel_t chan)
{
	mavlink_msg_mission_ack_send(
		chan,
		mavlink_system.sysid,
		mavlink_system.compid,
		0
);
}


static void NOINLINE send_statustext(mavlink_channel_t chan)
{
}
// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    return false;
}
// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = serial_free();
    if (telemetry_delayed(chan)) {
        return false;
    }
    switch(id) {
      case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        break;
      case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
				break;
			case MSG_ID_REQUEST_DATA_STREAM:
				CHECK_PAYLOAD_SIZE(REQUEST_DATA_STREAM);
				send_requst(chan);
				break;
			case MSG_ID_MISSION_ITEM_HOME:
				CHECK_PAYLOAD_SIZE(MISSION_ITEM);
				send_mission_item_home(chan);
				break;
			case MSG_ID_MISSION_ITEM_WAYPOINT:
				CHECK_PAYLOAD_SIZE(MISSION_ITEM);
				send_mission_item_waypoint(chan);
				break;
			case MSG_ID_MISSION_COUNT:
				CHECK_PAYLOAD_SIZE(MISSION_COUNT);
				send_mission_count(chan);
				break;
			case MSG_ID_MISSION_ACK:
				CHECK_PAYLOAD_SIZE(MISSION_ACK);
				send_mission_ack(chan);
				break;
			
		  default:
			  break;
    }
    return true;
}

#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];



// send a message using mavlink
void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];
    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }
    if (id == MSG_RETRY_DEFERRED) {
        return;
    }
    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }
    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}
void mavlink_send_text(mavlink_channel_t chan, enum gcs_severity severity, char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }
    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        pending_status.severity = (uint8_t)severity;
        mav_array_memcpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(
            chan,
            severity,
            str);
    }
}
void update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;
    // process received bytes
    while(serial_available())
    {
        uint8_t c = serial_read_ch(); 
        // Try to get a new message
				//从串口读到的数据中解析出每帧消息并写到结构体msg中
        if (mavlink_parse_char(chan, c, &msg, &status)) {
					
//此处可打印msgid等信息 
					
//					  printf("%c",c);
//if(msg.msgid!=0&&msg.msgid!=66)	
//if(msg.msgid==0)// 	received=1;							
//if(msg.msgid>=38&&msg.msgid<=48&&msg.msgid!=42)	
//if(msg.msgid!=0&&msg.msgid!=66)					
{
	
//	u3_printf("sys:%d,seq:%d,compid:%d,,\r\n",msg.msgid, msg.seq, msg.compid);

}
					handleMessage(&msg);//结构体成员赋值，有解析任务的回复信息

        }

				
    }
}

//解包函数,可按需要仿照如下格式添加case和对应的decode函数
void handleMessage(mavlink_message_t* msg)
{
    //struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid) {
			
        case MAVLINK_MSG_ID_HEARTBEAT: {											//0
					  mavlink_msg_heartbeat_decode(msg, &heartbeat); 	//解析心跳包
					
						u3_printf("\r\nmode:%s",(heartbeat.custom_mode==0?"stabilize":(heartbeat.custom_mode==4?"guided":\
																			 heartbeat.custom_mode==16?"poshold":"else")));
						u3_printf(" %d\r\n",heartbeat.custom_mode);
					
		
            break;
        }
				
				case MAVLINK_MSG_ID_ATTITUDE: {												//				
					  mavlink_msg_attitude_decode(msg, &attitude);

//此处打印姿态信息
					u3_printf("roll:%f,pitch:%f\r\n",attitude.roll*180/Pi,attitude.pitch*180/Pi);
					  break;
				}
							
				
				case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
					  mavlink_msg_rc_channels_raw_decode(msg, &rc_input); 

//此处可打印遥控输入信息					
//					u3_printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",rc_input.chan1_raw,rc_input.chan2_raw,rc_input.chan3_raw,rc_input.chan4_raw,rc_input.chan5_raw,rc_input.chan6_raw,rc_input.chan7_raw,\
																									rc_input.chan8_raw);
					  break;
				}
				
				
				case MAVLINK_MSG_ID_MISSION_ACK: {
						mavlink_msg_mission_ack_decode(msg,&mission_ack);
					u3_printf("mission_ack:%d\r\n",mission_ack.type);
					break;
				}

				case MAVLINK_MSG_ID_MISSION_REQUEST:{
					mavlink_msg_mission_request_decode(msg,&mission_request);
					u3_printf("mission_request:%d\r\n",mission_request.seq);
					mission_received=1;
					break;
				}					

				
				case MAVLINK_MSG_ID_VFR_HUD:{
					mavlink_msg_vfr_hud_decode(msg,&rx_vfr_hud);
					
					break;
				}
                case 24:{
                
                    mavlink_msg_gps_raw_int_decode(msg,&GPS_data);
                    break;
                }
				
				default:
					  break;
				
    }     // end switch
	
//根据mission_request.seq,请求的航点序号发送航点
		if(mission_received)
		{
			if(mission_request.seq==0)
			{
				delay_ms(10);
				mavlink_send_message(MAVLINK_COMM_0,MSG_ID_MISSION_ITEM_HOME , 0);//设置家的位置，位置在发送函数中进行了形参赋值
			}
			
			if(mission_request.seq==1)
			{
				guided_target.seq=1;
				guided_target.x=30.1234567;
				guided_target.y=113.1234567;
				guided_target.z=12.34;
				
				delay_ms(10);
				mavlink_send_message(MAVLINK_COMM_0,MSG_ID_MISSION_ITEM_WAYPOINT , 0);//设置航点1
				
			}
			
			if(mission_request.seq==2)
			{
				
				guided_target.seq=2;
				guided_target.x=31.1234567;
				guided_target.y=114.1234567;
				guided_target.z=23.45;
				delay_ms(10);
				mavlink_send_message(MAVLINK_COMM_0,MSG_ID_MISSION_ITEM_WAYPOINT , 0);//设置航点2
				
			}

			mission_received=0;
		}
		
		
} // end handle mavlink
