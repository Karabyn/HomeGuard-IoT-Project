#include "stm32f0xx.h"
#include "debug.h"
#include "NRF24L01.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_spi.h"
#include "string.h"

/*
 * !!! variable to indicate and switch between transmit/receive mode
 * set 0 for transmitter, 1 for receiver!
 * 1: Transmit On, Receive Off
 * 0: Transmit Off, Receive On
 */
#define transmit 0

extern uint8_t RX_BUF[];
extern uint8_t TX_BUF[];

RCC_ClocksTypeDef RCC_Clocks;
static volatile uint32_t TimingDelay;
void Delay(uint32_t nTime);

int main(void)
{ 
	SystemInit();

	/* Configure SysTick IRQ and SysTick Timer to generate interrupts every 500µs */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency /100);

	//Configure the SysTick for millisecond interrupts
	//SysTick_Config(SystemCoreClock/1000);

	// Configure and enable USART1
	debugInit();
	// Send data with USART
	debugSend("USART1 Enabled!\r\n");
	//Initialize nRF24L01
	nRF24L01_Initial();
	debugSend("NRF2401 Initialized!\r\n");

	if(transmit) // transmit mode is turned on
	{
		strcat(TX_BUF, "Hello, Petro Karabyn!\r\n");
		//debugSend("nRF24L01 Send mode\r\n");
	}
	else // receive mode is turned on
	{
		RX_Mode();
		debugSend("nRF24L01 Receive mode \r\n");
	}

	while(1)
	{
		Delay(100);
		if(transmit)
		{		
			NRF24L01_Send();
			Delay(100);
		}
		else
		{
			NRF24L01_Receive();
			Delay(100);
		}
	}
}

void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
  
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{

  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }

}
