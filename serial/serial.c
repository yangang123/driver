#include "serial.h"
#include "uart.h"

int ringbuffer_count(struct uart_buffer_s *lb)
{
	int n = lb->head - lb->tail;

	if (n < 0) {
		n += lb->size;
	}

	return n;
}
                                                                          
int ioctl(int fd, int cmd, uint32_t *val)
{   
	struct uart_dev_s *dev = NULL;
	if (fd == 0) {
		dev = &uart1_dev;
	} else if (fd == 2){
       dev = &uart3_dev;
	} else {
		return -1;
	}
	
	switch (cmd) {
		case AVAILABLE_READ_LEN_CMD:
			*val = (uint32_t)ringbuffer_count(dev->recv);
			break;		
	}		
}

int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
     USART_SendData(USART1, (uint8_t) ch);

	return ch;
}

void uart_rx_ringbuffer_push_from_usart(struct uart_dev_s *dev, uint8_t *u_data)
{
	dev->recv->buffer[dev->recv->head] = *u_data;
	dev->recv->head = (dev->recv->head + 1) % dev->recv->size;
}

int uart_rx_ringbuffer_pop (struct uart_dev_s *dev, uint8_t *data)
{
	uart_disablerxint(dev);
	*data = dev->recv->buffer[dev->recv->tail];
	dev->recv->tail = (dev->recv->tail + 1) % dev->recv->size;
	uart_enablerxint(dev);
	return 0;
}

void uart_tx_ringbuffer_push(struct uart_dev_s *dev , uint8_t *u_data)
{	
	uart_disabletxint(dev);
	
	dev->xmit->buffer[dev->xmit->head] = *u_data;
	dev->xmit->head = (dev->xmit->head + 1) % dev->xmit->size;

	uart_enabletxint(dev);
}

uint8_t uart_tx_ringbuffer_pop_to_usart (struct uart_dev_s *dev)
{    
	if (dev->xmit->head != dev->xmit->tail) {
		USART_SendData(dev->uart, dev->xmit->buffer[dev->xmit->tail]);
		dev->xmit->tail = (dev->xmit->tail + 1) % dev->xmit->size;

		return 1;
	}

	return 0;
}

void write(int fd, uint8_t *buf, uint16_t len)
{	
	struct uart_dev_s *dev = NULL;
	if (fd == 0) {
		dev = &uart1_dev;
	} else if (fd == 2) {
       dev = &uart3_dev;
	} else {
		return;
	}

	for(uint8_t i = 0; i < len; i++) {
		uart_tx_ringbuffer_push(dev, &buf[i]);
	}
}

void read(int fd, uint8_t *buf, uint16_t len) 
{
	struct uart_dev_s *dev = NULL;
	if (fd == 0) {
		dev = &uart1_dev;
	} else if (fd == 2){
       dev = &uart3_dev;
	} else {
		return;
	}
    
	for(uint8_t i = 0; i < len; i++) {        
		uint8_t value = 0;
		uart_rx_ringbuffer_pop(dev, &value);
		buf[i] = value;
	}
}

int uart_ioctl(int fd, int cmd, uint32_t *arg)
{  
	struct uart_dev_s *dev = NULL;
	if (fd == 0) {
		dev = &uart1_dev;
	} else if (fd == 2){
       dev = &uart3_dev;
	} else {
		return -1;
	}

	switch (cmd) {
		case AVAILABLE_READ_LEN_CMD:
			*arg = ringbuffer_count(dev->recv);
			break;
	}

}