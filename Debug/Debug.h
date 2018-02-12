/*#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED
*/
#include"stm32f0xx.h"

/**
*To enable debug set DEBUG to 1
*To disable debug set DEBUG to 0
*/
#define DEBUG 1

void debugInit(void);
void debugSend(char StringToSend[]);
void debugSend2(char StringToSend[], int len);
void debugReceive();
void _printfLngS(char* text ,int32_t x);
//#endif /* DEBUG_H_INCLUDED */
