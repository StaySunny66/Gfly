#ifndef _LED_H
#define _LED_H

void Led_init();
void Led_Set(unsigned char LED,unsigned char STATE);


#define LED_ON 1
#define LED_OFF 0 

#define ACK_LED  1
#define SYS_LED  2


#endif