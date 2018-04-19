#include "uart.h"
#include "serial.h"

char g_uart1_rxbuffer[CONFIG_UART1_RXBUFSIZE];
char g_uart1_txbuffer[CONFIG_UART1_TXBUFSIZE];

static void up_rxint(struct uart_dev_s *dev, bool enable);
static void up_txint(struct uart_dev_s *dev, bool enable);

static const struct uart_ops_s g_uart_ops =
{
  .rxint          = up_rxint,
  .txint          = up_txint,
};

struct uart_buffer_s  uart1_xmit = {
	.size    = CONFIG_UART1_TXBUFSIZE,
	.head 	= 0,
	.tail   = 0,
	.buffer	 = g_uart1_txbuffer,
	
	.total_len 	   = 0,
	.over  		   = 0,
	.user_buf_over = 0	
};	

struct uart_buffer_s uart1_recv = {
	.size	= CONFIG_UART1_RXBUFSIZE,
	.head 	= 0,
	.tail   = 0,
	.buffer = g_uart1_rxbuffer,	

	.total_len 		= 0,
	.over   		= 0,
	.user_buf_over  = 0	
}; 	

struct uart_dev_s uart1_dev = {
	.xmit	= &uart1_xmit,
	.recv 	= &uart1_recv,
	.uart   = USART1,

	.ops    = &g_uart_ops,
};

char g_uart3_rxbuffer[CONFIG_UART3_RXBUFSIZE];
char g_uart3_txbuffer[CONFIG_UART3_TXBUFSIZE];

struct uart_buffer_s  uart3_tx = {
	.size    = CONFIG_UART3_TXBUFSIZE,
	.head 	= 0,
	.tail   = 0,
	.buffer	 = g_uart3_txbuffer,
	
	.total_len 	   = 0,
	.over  		   = 0,
	.user_buf_over = 0	
};	

struct uart_buffer_s uart3_recv = {
	.size	= CONFIG_UART3_RXBUFSIZE,
	.head 	= 0,
	.tail   = 0,
	.buffer = g_uart3_rxbuffer,	

	.total_len 		= 0,
	.over   		= 0,
	.user_buf_over  = 0	
}; 	

struct uart_dev_s uart3_dev = {
	.xmit	= &uart3_tx,
	.recv 	= &uart3_recv,
	.uart   = USART3,
	.ops    = &g_uart_ops,
}; 

static void up_txint(struct uart_dev_s *dev, bool enable)
{
	
	USART_ITConfig(dev->uart, USART_IT_TXE, enable);
}

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
	USART_ITConfig(dev->uart,USART_IT_RXNE, enable);
}

/*******************************************************************************
* 函数名  : void uart_init1(u32 bound)
* 描述    : USART1发送字符串
* 输入    : u32 bound  波特率(57600, 115200, 460800...)
* 输出    : 
* 返回    :  
* 说明    : 
*******************************************************************************/
void uart_init1(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1, ENABLE);                    //使能串口1 
}


/*******************************************************************************
* 函数名  : void uart_init2(u32 bound)
* 描述    : USART2发送字符串
* 输入    : u32 bound  波特率(57600, 115200, 460800...)
* 输出    : 
* 返回    :  
* 说明    : 
*******************************************************************************/
void uart_init2(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART1，GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟

     //USART1_TX   GPIOA.2
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器


	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART2, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART2, ENABLE);                    //使能串口1 
}


/*******************************************************************************
* 函数名  : void uart_init2(u32 bound)
* 描述    : USART2发送字符串
* 输入    : u32 bound  波特率(57600, 115200, 460800...)
* 输出    : 
* 返回    :  
* 说明    : 
*******************************************************************************/
void uart_init3(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART1，GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能USART1，GPIOA时钟

     //USART3_TX  
	  GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN; //PA.2
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	//USART3_RX	 
	GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART3, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    //使能串口1 
}

void uart_common(struct uart_dev_s *dev)
{
	if (USART_GetITStatus(dev->uart, USART_IT_RXNE)!= RESET) {    
		uint8_t val = (uint8_t)dev->uart->DR;
		uart_rx_ringbuffer_push_from_usart(dev, &val); 
	}
	
	if(USART_GetITStatus(dev->uart, USART_IT_TXE) != RESET) {
		if(uart_tx_ringbuffer_pop_to_usart(dev) == 0) {
			USART_ITConfig(dev->uart, USART_IT_TXE, DISABLE);
		}
	}
}
/*******************************************************************************
* 函数名  : void USART1_IRQHandler(void)
* 描述    : 串口1的中断服务程序 
* 输入    : 
* 输出    : 
* 返回    : 
* 说明    : 
*******************************************************************************/	
void USART1_IRQHandler(void)
{     
	uart_common(&uart1_dev);
}

void USART3_IRQHandler(void)                	          
{
	uart_common(&uart3_dev);
}

/*******************************************************************************
* 函数名  : int uart_send(USART_TypeDef* USARTx, uint8_t *buf, uint16_t length)
* 描述    : USART2发送字符串
* 输入    : USART_TypeDef* USARTx : USART1, USART2, USART3, UART4, UART5
            uint8_t *buf		  : 发送的缓冲区的首地址
            uint16_t length       : 发送数据的长度
* 输出    : -1： 数据长度不对   0：正确发送
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
int uart_send(USART_TypeDef* USARTx, uint8_t *buf, uint16_t length)
{ 
	uint16_t i = 0;
	
	if (length == 0 )
	{
		return -1;
	}
  
	for (i = 0; i < length; i++)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET); 
		USART_SendData(USARTx ,*buf++);
	}

	return 0;
}
