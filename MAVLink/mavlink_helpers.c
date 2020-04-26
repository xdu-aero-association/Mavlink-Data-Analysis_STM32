#include "mavlink_helpers.h"
#include "usart.h"

void comm_send_ch(mavlink_channel_t chan, uint8_t buf)
{
	chan=chan;
	USART_SendData(USART1, buf);         //???1????
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//??????
}
