#include "sys.h"//作者：恒久力行 qq:624668529
#include "mavlink_usart_fifo.h"
#define UART_TX_BUFFER_SIZE        4095
#define UART_RX_BUFFER_SIZE        4095
fifo_t uart_rx_fifo, uart_tx_fifo;
uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE], uart_rx_buf[UART_RX_BUFFER_SIZE];
/** @brief 读FIFO
  * @param fifo 待读缓冲区
	*        *ch   读到的数据
	* @return 
	*       正确读取,1; 无数据,0
  */
uint8_t fifo_read_ch(fifo_t* fifo, uint8_t* ch)
{
	if(fifo->tail == fifo->head) return false;
	*ch = fifo->buf[fifo->tail];  
	
	if(++fifo->tail >= fifo->length) fifo->tail = 0;
  return true;
}
/** @brief 写一字节到FIFO
  * @param fifo 待写入缓冲区
	*        ch   待写入的数据
	* @return 
	*        正确,1; 缓冲区满,0
  */
uint8_t fifo_write_ch(fifo_t* fifo, uint8_t ch)
{
	uint16_t h = fifo->head;
	
	if(++h >= fifo->length) h = 0;
	if(h == fifo->tail) return false;
	
	fifo->buf[fifo->head] = ch;
	fifo->head = h;
  return true;
}
/** @brief 返回缓冲区剩余字节长度
  * @param fifo 
	* @return 
	*        剩余空间
  *
  * @note  剩余字节长度大于等于2时，才可写入数据
  */
uint16_t fifo_free(fifo_t* fifo)  
{
	uint16_t free;
	
	if(fifo->head >= fifo->tail) free = fifo->tail + (fifo->length - fifo->head);
	else free = fifo->tail - fifo->head;
	
  return free;
}
uint16_t fifo_used(fifo_t* fifo)
{
	uint16_t used;
	
	if(fifo->head >= fifo->tail) used = fifo->head - fifo->tail;
	else used = fifo->head + (fifo->length - fifo->tail);
	
	return used;	
}
/** @brief 初始化缓冲区
  * @param *fifo
  *        *buf 
  *        length
  */
void fifo_init(fifo_t* fifo, uint8_t* buf, uint16_t length)  
{
	uint16_t i;
	
	fifo->buf = buf;
	fifo->length = length;
	fifo->head = 0;
	fifo->tail = 0;
	
	for(i=0; i<length; i++) fifo->buf[i] = 0;	
}
/** @brief 写数据到串口，启动发射
  *        
  * @note 数据写入发射缓冲区后，启动发射中断，在中断程序，数据自动发出
  */
uint8_t serial_write_buf(uint8_t* buf, uint16_t length) {
	uint16_t i;
	
	if(length == 0) return false;
  for(i = 0; length > 0; length--, i++)	{
		fifo_write_ch(&uart_tx_fifo, buf[i]);
	}	
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	return true;
}
/** @brief 自串口读数据 
  * @return 一字节数据
  */
uint8_t serial_read_ch(void){
	uint8_t ch;	
	fifo_read_ch(&uart_rx_fifo, &ch);	
	return ch;
}
/** @breif 检测发射缓冲区剩余字节长度 
  * @return 剩余字节长度
  */
uint16_t serial_free(void){
	return fifo_free(&uart_tx_fifo);
}
uint16_t serial_available(void){
	return fifo_used(&uart_rx_fifo);
}
void USART1_IRQHandler(void)
{			
  uint8_t c;	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//接收中断
  {	
		c = USART_ReceiveData(USART1);  
		fifo_write_ch(&uart_rx_fifo, c);		
    //USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
  }
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)//发送中断
  {   		
		if(fifo_read_ch(&uart_tx_fifo, &c)) 
			USART_SendData(USART1, c);     
		else 
			USART_SendData(USART1, 0x55);			
    if (fifo_used(&uart_tx_fifo) == 0) // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
    {
      // Disable the EVAL_COM1 Transmit interrupt 
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
  }		
}
