#ifndef __OPEN_TEL_MAVLINK_H//??:????  qq:624668529
#define __OPEN_TEL_MAVLINK_H
//#include "./minimal/minimal/minimal.h"
#include "define.h"
#include "mavlink_avoid_errors.h"
#include "stdint.h"
#include "usart3.h"
#include "timer.h"

#define Pi 3.1415926

extern u8 mission_received; 

extern mavlink_request_data_stream_t					 tx_request_data_stream;
extern fifo_t uart_rx_fifo, uart_tx_fifo;

extern mavlink_mission_request_t  mission_request;
extern mavlink_system_t mavlink_system;


extern mavlink_mission_item_t 		 guided_target;
extern mavlink_attitude_t          attitude;
extern mavlink_rc_channels_raw_t   rc_input;
extern mavlink_vfr_hud_t						rx_vfr_hud;
extern mavlink_heartbeat_t         heartbeat;



void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
void update(void);
void handleMessage(mavlink_message_t* msg);
#endif /*__OPENTEL_MAVLINK_H*/
