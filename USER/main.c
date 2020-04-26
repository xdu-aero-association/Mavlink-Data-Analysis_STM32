/*
移植参考了博客园  恒久力行的  "移植mavlink到stm32详细教程，后面附快速移植方法"
链接   http://www.cnblogs.com/lovechen/p/5809709.html

说明:使用stm32向飞控写航点,也能读出飞控数据

硬件连接: stm32串口1 连接pixhawk数传口,注意数传口波特率参数要与下面串口1一致
			 stm32串口3连接电脑串口助手,查看打印信息
注意：连接飞控时其他地面站不能连接，例如USB连接mp
建议等apm/pix启动完成后重启一次stm32

qq：1032921868
有意见建议可反馈给我

*/

#include "delay.h"
#include "sys.h"
#include "led.h"
#include "string.h"

#include "usart.h"
#include "usart3.h"
#include "mavlink_types.h"
#include "open_tel_mavlink.h"

mavlink_system_t mavlink_system;

u32 last_mav_update_rc_time_count=0,time_100ms_count=0,time_10ms_count=0;

u8 update_1hz_finish=1,update_5hz_finish=1,update_10hz_finish=1,flag_update_arm_finish,\
	 update_600ms_finfsh=1,update_20hz_finish=1;
u8 mission_received=0; 


#define UART_TX_BUFFER_SIZE        511
#define UART_RX_BUFFER_SIZE        511

extern fifo_t uart_rx_fifo, uart_tx_fifo;
extern uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE], uart_rx_buf[UART_RX_BUFFER_SIZE];

int main(void)
{
	delay_init();	    	 //延时函数初始化	  
	LED_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(57600);	 	//串口1初始化为57600
	usart3_init(57600);
	TIM3_Int_Init(999,719);//定时器10ms中断
	
	fifo_init(&uart_tx_fifo, uart_tx_buf, UART_TX_BUFFER_SIZE);	//初始化fifo
	fifo_init(&uart_rx_fifo, uart_rx_buf, UART_RX_BUFFER_SIZE);
	
//实测除70号超越控制,发给飞控的其他消息并不受该值影响
	mavlink_system.sysid = MAV_TYPE_GENERIC;
	mavlink_system.compid = MAV_COMP_ID_ALL;

	
	//请求数据流,建议等apm/pix启动完成后重启32,保证飞控收到66号消息
	//发送成功后可在串口3看到打印的飞控的数据
	
	{
				tx_request_data_stream.req_stream_id=0;   //请求全部数据流
				tx_request_data_stream.req_message_rate=2;//数据速率,这里2hz
        tx_request_data_stream.start_stop=1; 
				mavlink_send_message(MAVLINK_COMM_0, MSG_ID_REQUEST_DATA_STREAM, 0);
	}
	
	delay_ms(5000);
	
	//发送航点数量消息,开启一个航点写入的过程
	mavlink_send_message(MAVLINK_COMM_0, MSG_ID_MISSION_COUNT, 0);//MSG_ID_MISSION_COUNT  44
//另一部分航点设置在接收解析中,设置两个航点的流程如下：
	//GCS->Drone  MISSION_COUNT
	//Drone->GCS  MISSION_REQUEST(0)
	//GCS->Drone  MISSION_ITEM(0)
	//Drone->GCS  MISSION_REQUEST(1)
	//GCS->Drone  MISSION_ITEM(1)
	//Drone->GCS  MISSION_ACK
	while(1)
	{
		if(!update_1hz_finish)
			{	
				mavlink_send_message(MAVLINK_COMM_0, MSG_HEARTBEAT, 0);//发送心跳包
				
				//可检测是否能收到飞控数据	
//				u3_printf("alt:%5.2f\r\n",rx_vfr_hud.alt);
				

				update_1hz_finish=1;
				LED0=!LED0;
			}			
			
	  	update();//接收并解析mavlink消息的函数，包含接收到回应信息后发送的航点信息，发送航点的函数在 "open_tel_mavlink.c"中
			
			if(0) {break;} //不加有个莫名其妙的警告
			
	}
	
	return 0;
	
}

 
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
			time_10ms_count++;
			
			if(time_10ms_count%5==0)
			{
				update_20hz_finish=0;
			}
			
			if(time_10ms_count%10==0)
			{
			time_100ms_count++;			
			update_10hz_finish=0;

			if(time_100ms_count%2==0)
				{
					update_5hz_finish=0;
				}
			
			if(time_100ms_count%10==0)
				{
				  update_1hz_finish=0; 
					
				}
			}
		}
}

