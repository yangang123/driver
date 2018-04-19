
#ifndef __SERIAL_H
#define __SERIAL_H

#include "Common.h"

#define AVAILABLE_READ_LEN_CMD        0x0005                                                                                   
#define AVAILABLE_WRITE_LEN_CMD       0x0006   

#define uart_enabletxint(dev)    dev->ops->txint(dev, true)
#define uart_disabletxint(dev)   dev->ops->txint(dev, false)
#define uart_enablerxint(dev)    dev->ops->rxint(dev, true)
#define uart_disablerxint(dev)   dev->ops->rxint(dev, false)

struct uart_buffer_s
{
  volatile int16_t head;   /* Index to the head [IN] index in the buffer */
  volatile int16_t tail;   /* Index to the tail [OUT] index in the buffer */
  int16_t      size;   	   /* The allocated size of the buffer */
  char        *buffer;     /* Pointer to the allocated buffer memory */

  uint32_t     total_len; 
  uint32_t     over;
  uint32_t     user_buf_over;
};

struct uart_dev_s {
	struct uart_buffer_s *xmit;
	struct uart_buffer_s *recv;
	USART_TypeDef* uart;

 const struct uart_ops_s *ops; 
};

struct uart_ops_s
{
   void (*rxint)( struct uart_dev_s *dev, bool enable);
   void (*txint)( struct uart_dev_s *dev, bool enable);
};


int ringbuffer_count(struct uart_buffer_s *lb);
void uart_rx_ringbuffer_push_from_usart(struct uart_dev_s *dev, uint8_t *u_data);
uint8_t uart_tx_ringbuffer_pop_to_usart (struct uart_dev_s *dev);

void write(int fd, uint8_t *buf, uint16_t len);
void read(int fd, uint8_t *buf, uint16_t len);
int  ioctl(int fd, int cmd, uint32_t *arg);

#endif /* __SERIAL_H  */

