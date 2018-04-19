#include "serial.h"
#include "uart.h"

/*
 fd : 0 是串口1
 fd : 2 是串口3

*/

void uart_test()
{   
	/* 初始化串口 */
    uart_init1(115200);
	uart_init3(9600);
	
	uint8_t buf[10];

	for (uint8_t i =0; i < 10; i++) {
		buf[i] = i;
	}
    
	/* 发送数据 */
	write(0,  buf,  sizeof(buf));

	while(1) {
 		
		/* 获取数据的长度 */
		uint32_t data_cnt = 0;   
		if( ioctl(0, AVAILABLE_READ_LEN_CMD, &data_cnt) < 0) {
			continue;
		}
        /* 获取数据  */
		uint8_t buf[10];
		if (data_cnt > 0)  {
			read((int)0, &buf[0], data_cnt);
		}
	}
}
