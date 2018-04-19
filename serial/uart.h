

#ifndef __UART_H
#define __UART_H

#include "Common.h"

#define USART1_TX_PIN		             PA9	 
#define USART1_RX_PIN		             PA10 
                                        
#define USART2_TX_PIN		             PA2 
#define USART2_RX_PIN		             PA3 
                                        
#define USART3_TX_PIN		             PB10 
#define USART3_RX_PIN		             PB11
                                        
#define UART4_TX_PIN		             PC10	 
#define UART4_RX_PIN		             PC11
                                        
#define UART5_TX_PIN		             PC12 
#define UART5_RX_PIN		             PD2  

#define CONFIG_UART1_RXBUFSIZE    			100
#define CONFIG_UART1_TXBUFSIZE 	 			100

#define CONFIG_UART3_TXBUFSIZE    			100
#define CONFIG_UART3_RXBUFSIZE 	 			  100

extern struct uart_dev_s uart1_dev;
extern struct uart_dev_s uart3_dev;

extern void uart_init1(u32 bound);
extern void uart_init2(u32 bound);
extern void uart_init3(u32 bound);
extern int uart_send(USART_TypeDef* USARTx, uint8_t *buf, uint16_t length);


#endif /* __UART_H  */

