/** @file mavlink_avoid_errors.h//作者：恒久力行 qq:624668529
 *	@简介:本文由王朔添加，用来统一解决，mavlink报错信息
 *	@see QQ624668529
 */
#ifndef MAVLINK_AVOID_ERRORS_H
#define MAVLINK_AVOID_ERRORS_H
#include "stdio.h"
#include "stdint.h"

/*解决..\MAVLINK\common\../mavlink_types.h(53): error:  #20: identifier "pack" is undefined*/
#define MAVPACKED( __Declaration__ )  __Declaration__ 
/*解决..\MAVLINK\common\../mavlink_types.h(53): error:  #3092: anonymous unions are only supported in --gnu mode, or when enabled with #pragma anon_unions*/
#pragma anon_unions
#define inline __inline
//#ifndef memset//由王朔添加 2018-08-24
//void *memset(void *dest, int data, size_t length) { 
//	uint32_t i;
//	int *point = dest;	
//	for(i=0; i<length; i++) point[i] = data; 
//	return dest;	
//} 
//#endif
//#ifndef memcpy//由王朔添加 2018-08-24
//void *memcpy(void *dest, const void *src, size_t n)
//{
//	unsigned char *pout = (unsigned char*)dest;
//	unsigned char *pin = (unsigned char*)src;
//	while (n-- > 0) *pout++ = *pin++;
//	return dest;
//}
//#endif
#include "mavlink_types.h"
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
//#define MAVLINK_SEPARATE_HELPERS
//mavlink_system_t mavlink_system = {0,0};
//mavlink_system_t mavlink_system = {
//	1,
//	1
//}; // System ID, 1-255, Component/Subsystem ID, 1-255
//void comm_send_ch(mavlink_channel_t chan, uint8_t buf)
//{
//	chan=chan;
//	USART_SendData(USART1, buf);         //向串口1发送数据
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//}
#include "mavlink.h"
#include "mavlink_helpers.h"

#endif //AVLINK_AVOID_ERRORS_H
