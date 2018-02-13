#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_spi.h"
#include "debug.h"
#include "NRF24L01.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

//SR04 definitions
#define SR04_Echo 		GPIO_Pin_1
#define SR04_EchoLine 	EXTI_Line1
#define SR04_EchoIRQ 	EXTI0_1_IRQn

#define SR04_TIMIRQ 	TIM2_IRQn
#define SR04_Prescaler 	2
#define SR04_Trig 		GPIO_Pin_0

#define SR04_GPIO 		GPIOA
#define SR04_Timer 		TIM2

#define FallingEdge 0
#define RisingEdge 1

//Define LED pins
#define GreenLED_Pin GPIO_Pin_9
#define OrangeLED_Pin GPIO_Pin_8
#define BlueLED_Pin GPIO_Pin_7
#define RedLED_Pin GPIO_Pin_6

#define LED_GPIO GPIOC

// Define motion sensor pin
#define MotionSensor_Pin GPIO_Pin_13
#define MotionSensor_GPIO GPIOA

//Peripheral type definitions
GPIO_InitTypeDef G;
TIM_TimeBaseInitTypeDef TB;
NVIC_InitTypeDef N;
EXTI_InitTypeDef E;
RCC_ClocksTypeDef RC;

//Volatile interrupts
volatile uint8_t PulseEnded = 1, InterruptEdge = 0, TimerOverflow = 0;
volatile uint32_t PulseTime = 0;
volatile uint32_t MSec = 0;

/*
 * !!! variable to indicate and switch between transmit/receive mode
 * set 0 for transmitter, 1 for receiver!
 * 1: Transmit On, Receive Off
 * 0: Transmit Off, Receive On
 */
#define transmit 1

extern uint8_t RX_BUF[];
extern uint8_t TX_BUF[];

RCC_ClocksTypeDef RCC_Clocks;
static volatile uint32_t TimingDelay;
void Delay(uint32_t nTime);


//This function initializes the edge interrupt for the SR04 ultrasonic
//sensor and uses the SR04 timer (Timer 2) to create the
//10us pulse required for the sensor. Once the 10us pulse has been
//generated, the timer is disabled and the variable InterruptEdge
//is set to 0 to denote the next interrupt will be the rising edge.
void SR04_SendPulse(void){
	//Ensure that the timer is initially disabled
	TIM_Cmd(SR04_Timer, DISABLE);

	//Set the timer count to 0
	TIM_SetCounter(SR04_Timer, 0);

	//Ensure that PulseEnded and TimerOverflow are zero as the SR04
	//hasn't even been sent the pulse yet!
	PulseEnded = 0;
	TimerOverflow = 0;

	//Enable the timer
	TIM_Cmd(SR04_Timer, ENABLE);
	//Set the trigger pin high
	GPIO_SetBits(SR04_GPIO, SR04_Trig);

	//Wait until at least 10us. With a prescalar of 1 the timer will
	//need to count up to at least 250 until the trigger pin can go
	//low.
	while(TIM_GetCounter(SR04_Timer) < (500/(SR04_Prescaler+1))){
		GPIO_SetBits(SR04_GPIO, SR04_Trig);
	}

	//After 10us has passed, the trigger pin can go low.
	GPIO_ResetBits(SR04_GPIO, SR04_Trig);

	//Disable the timer so it doesnt carry on running and potentially
	//overflow.
	TIM_Cmd(SR04_Timer, DISABLE);

	//Set the current interrupt edge to be detected as rising. The SR04
	//ultrasonic sensor has a brief pause after the trigger signal where
	//8 40KHz pulses are sent. After these 40KHz pulses, the echo signal
	//goes high and the time that it is high is the time it took for the
	//8 pulses to leave the transducer, hit an object and return!
	InterruptEdge = RisingEdge;

	//Reset the counter back to 0 to that the counter can start counting
	//essentially as soon as the rising edge interrupt is detected.
	TIM_SetCounter(SR04_Timer, 0);
}

//In here is the external line interrupt handler. The external line used
//is defined by SR04_EchoLine. Using EXTI Line 1.
void EXTI0_1_IRQHandler(void){
	//If the interrupt has been triggered
	if(EXTI_GetITStatus(SR04_EchoLine) == SET){
		//Clear the interrupt pending bit
		EXTI_ClearITPendingBit(SR04_EchoLine);

		//On rising edge (Interrupt Edge will be 0 on rising edge)
		if(InterruptEdge == RisingEdge){
			//Start the counter to time the length of the returned pulse!
			TIM_Cmd(SR04_Timer, ENABLE);

			//Set the next edge to be falling. The 0 and 1 convention
			InterruptEdge = FallingEdge;
		}
		else{ //On falling edge (Interrupt Edge will be 1 on falling edge)
			//Disable the timer to stop the count
			TIM_Cmd(SR04_Timer, DISABLE);

			//Set PulseTime to the length the timer was counting for
			PulseTime = TIM_GetCounter(SR04_Timer);

			//Indicate that the SR04 returned pulse has ended.
			PulseEnded = 1;
		}
	}
}

//Enable the Timer overflow interrupt for the timer 2.
//Detect whether the timer timed out before a pulse was returned.
//Timer 2 is a 32 bit timer and has a pretty long time to timeout
//but this also stops the main loop code from blocking
//if the SR04 somehow gets disconnected.
void TIM2_IRQHandler(void){

	//Timer overflowed before pulse ended (Update interrupt)
	if(TIM_GetITStatus(SR04_Timer, TIM_IT_Update) == SET){
		//Clear the pending interrupt bit
		TIM_ClearITPendingBit(SR04_Timer, TIM_IT_Update);

		//Disable the timer to stop further counting
		TIM_Cmd(SR04_Timer, DISABLE);

		//Set the TimerOverflow flag and PulseEnded flag
		TimerOverflow = 1;
		PulseEnded = 1;
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

/*
//Standard SysTick time keeping interrupt! Increments the variable MSec
//every millisecond.
void SysTick_Handler(void){
	MSec++;
}

void DelayMs(uint32_t T){
	volatile uint32_t MSS = MSec;
	while((MSec-MSS)<T) asm volatile("nop");
}
*/

void intializeLeds(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	G.GPIO_Pin = GreenLED_Pin | OrangeLED_Pin | BlueLED_Pin | RedLED_Pin; //Set pins inside the struct
	G.GPIO_Mode = GPIO_Mode_OUT; //Set GPIO pins as output
	G.GPIO_OType = GPIO_OType_PP; //Ensure output is push-pull vs open drain
	G.GPIO_PuPd = GPIO_PuPd_NOPULL; //No internal pullup resistors required
	G.GPIO_Speed = GPIO_Speed_Level_1; //Set GPIO speed to lowest
	GPIO_Init(LED_GPIO, &G); //Assign struct to LED_GPIO
}

void initializeMotionSensor(void)
{
	G.GPIO_Pin = MotionSensor_Pin;
	G.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(MotionSensor_GPIO, &G);
}

void blinkLeds(void)
{
	GPIO_SetBits(LED_GPIO, GreenLED_Pin);
	GPIO_SetBits(LED_GPIO, OrangeLED_Pin);
	GPIO_SetBits(LED_GPIO, BlueLED_Pin);
	GPIO_SetBits(LED_GPIO, RedLED_Pin);
	Delay(200);
	GPIO_ResetBits(LED_GPIO, GreenLED_Pin);
	GPIO_ResetBits(LED_GPIO, OrangeLED_Pin);
	GPIO_ResetBits(LED_GPIO, BlueLED_Pin);
	GPIO_ResetBits(LED_GPIO, RedLED_Pin);
	Delay(200);
}

void resetLeds(void){
	GPIO_ResetBits(LED_GPIO, GreenLED_Pin);
	GPIO_ResetBits(LED_GPIO, OrangeLED_Pin);
	GPIO_ResetBits(LED_GPIO, BlueLED_Pin);
	GPIO_ResetBits(LED_GPIO, RedLED_Pin);
}

void configureHCSRO4Sensor(void){
	//Configure the SR04_Trigger pin as an output
	G.GPIO_Pin = SR04_Trig;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_Mode = GPIO_Mode_OUT;
	G.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(SR04_GPIO, &G);

	//Configure the SR04_Echo pin as an input
	G.GPIO_Pin = SR04_Echo;
	G.GPIO_PuPd = GPIO_PuPd_UP;
	G.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(SR04_GPIO, &G);

	//Setup the SR04 timer to run as fast as possible with the longest
	//time period possible. Timer 2 allows
	//for a time period of 2^32 - 1 = ~4.29billion. This equates to a total
	//time-able period of 48e6/(2^32 - 1) = 11.2ms. If the sensor could
	//work at this kind of distance, the sensor would be able to measure up
	//to 1.92m! By increasing the prescaler, this maximum distance can be
	//increased. A prescaler of 2, increases this distance
	//up to 3.84m.
	TB.TIM_ClockDivision = TIM_CKD_DIV1;
	TB.TIM_CounterMode = TIM_CounterMode_Up;
	TB.TIM_Prescaler = SR04_Prescaler;
	TB.TIM_Period = ((uint64_t)1<<32) - 1;
	TIM_TimeBaseInit(SR04_Timer, &TB);

	//Setup the worst case scenario overflow interrupt and clear the pending
	//bit in case is was previously set!
	TIM_ClearITPendingBit(SR04_Timer, TIM_IT_Update);
	TIM_ITConfig(SR04_Timer, TIM_IT_Update, ENABLE);

	//Enable an EXTI interrupt on our EchoLine. In this example, this line will
	//be line 1. Make sure the interrupt is present on both the rising and
	//falling edge of the input. This allows us to relatively precisely time
	//the output pulse from the SR04.
	E.EXTI_Line = SR04_EchoLine;
	E.EXTI_LineCmd = ENABLE;
	E.EXTI_Mode = EXTI_Mode_Interrupt;
	E.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&E);

	//Enable the timer update interrupt
	N.NVIC_IRQChannel = SR04_TIMIRQ;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 1;
	NVIC_Init(&N);

	//Enable the EXTI line interrupt
	N.NVIC_IRQChannel = SR04_EchoIRQ;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 0;
	NVIC_Init(&N);

	//Get the clock frequency of the timer. This is required for the calculation
	//of distance.
	RCC_GetClocksFreq(&RC);
}

// USART BLOCK -------------------------------------------------------------------------------------
void InitDelayTIM6(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // enable TIM6 timer
}

int TIM6delay(uint16_t value){
	uint32_t prescaler_ms; // milisecs prescaler
	TIM6->PSC = prescaler_ms; // apply current prescaler
	TIM6->ARR = value;  // countdown limitation
	TIM6->CNT = 0; // assure countdown  set to 0
	TIM6->CR1 |= TIM_CR1_CEN;  //enable timer (automatic start)
		while((TIM6->SR & TIM_SR_UIF)==0){}  // check for interruptions upon timer overflow
		// till counting is over
		TIM6->SR &=~TIM_SR_UIF; // reset flag
	return 0;
}

void GPIO_CONFIG_FOR_USART_INIT(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable A gpio line
	GPIOA->MODER |=
			(GPIOA->MODER &
					~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10) ) \
					| (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); // setting pins to alternative function mode

	GPIOA->AFR[1] /*Alternative function register #1*/
	           = ( GPIOA->AFR[1] &~(GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2) )    /*clear those registers (alternative function for Hight register 1 and 2)*/
	          | (1 << (1*4)) | (1 << (2*4) )/*setting bits for appropriate alternative functions*/;
}

void USART1_CONFIG_INIT(void){ // PA9 -- TX    PA10 -- RX
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // USART1 ENABLE
	USART1->BRR = 480000/96; // SETTING FREQUENCY DIVISOR
	USART1->CR1 =  USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; // TRANSMITTER ENABLE, USART_ENABLE // enabling USART of STM32 controller for transmission exclusively
	while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART1->ICR |= USART_ICR_TCCF; // CLEAR TC FLAG
	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART1_IRQn,0);
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART2_CONFIG_INIT(void){ // SAME AS USART1. BUT FOR USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // USART1 ENABLE
	USART2->BRR = 480000/96; // SETTING FREQUENCY DIVISOR
	USART2->CR1 =  USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; // TRANSMITTER ENABLE, USART_ENABLE // enabling USART of STM32 controller for transmission exclusively
	while((USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART2->ICR |= USART_ICR_TCCF; // CLEAR TC FLAG
	USART2->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART2_IRQn,0);
	NVIC_EnableIRQ(USART2_IRQn);
}

void USART1_TRANSMIT(char * stringtosend){
	size_t send = 0;
	size_t sring_length = strlen(stringtosend);
	while (1) {
		if(send == sring_length){
			USART1->ICR |= USART_ICR_TCCF; // Clear transfer complete flag
			break; // return 0
		} else{
			USART1->TDR = stringtosend[send++]; //stringtosend[send++];
			TIM6delay(50);
		}
	}
}

void USART2_TRANSMIT(char * stringtosend){
	size_t send = 0;
	size_t sring_length = strlen(stringtosend);
	while (1) {
		if(send == sring_length){
			USART2->ICR |= USART_ICR_TCCF; // Clear transfer complete flag
			break;
		} else{
			USART2->TDR = stringtosend[send++];
			TIM6delay(50);
		}
	}
}

int main(void)
{ 
	SystemInit();

	//Enable clocks to GPIO and Timer 2.
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	intializeLeds();

	/** SENSORS **/

	initializeMotionSensor();
	uint8_t motionRead = 0; //Initialize motionRead variable
	uint8_t lastMotionRead = 0; // used to keep last value of motion to prevent transmitting data when nothing has changed.
	char* motion = "0";

	/* Configure SysTick IRQ and SysTick Timer to generate interrupts every 500µs */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency /100);

	configureHCSRO4Sensor();

	//Initialize the float variables.
	float Distance = 0.0f, lastDistance = 0.0f, TimeOfFlight = 0.0f;

	//The real life time in seconds for one timer tick will be stored in this
	//variable.
	float TTimePerCnt = (SR04_Prescaler+1)/(float)RC.PCLK_Frequency;

	//Everybody's favorite temperature dependent constant!
	const float SpeedOfSound = 343.0f;

	uint8_t PulseSent = 0;

	blinkLeds();

	/** RADIO MODULE  **/

	nRF24L01_Initial();

	// Configure and enable USART1
	//debugInit();
	// Send data with USART
	//debugSend("USART1 Enabled!\r\n");
	//Initialize nRF24L01
	//debugSend("NRF2401 Initialized!\r\n");

	blinkLeds();

	while(1)
	{
		/**/
		// 1. HC-SR04 Sensor Detection

		//Initially send the pulse.
		if(PulseSent == 0){
			PulseSent = 1;
			SR04_SendPulse();
		}

		//Instead of using a blocking check, by simply using an if statement and
		//an additional variable named PulseSent, we can poll to check if the
		//returned pulse has been measured. This allows the program loops to do
		//other things while waiting for the returned pulse/
		if(PulseSent && PulseEnded){
			PulseSent = 0;
			//If the timer overflowed, set PulseTime to zero. The value stored in
			//PulseTime could be checked to see if the timer overflowed and a
			//suitable error message could be displayed in this condition.
			if(TimerOverflow){
				PulseTime = 0;
			}
			//If the timer however didn't over flow, calculate the distance
			//Calculate the one way time of flight of the ultrasonic pulse.
			//To ensure that the time of flight is only one way, the PulseTime
			//is divided by two as PulseTime is the time taken for the pulses
			//from the SR04 to be emitted, hit the object and bounce back.
			//Therefore, by halving this, the one way pulsetime can be found.
			else{
				TimeOfFlight = TTimePerCnt*((float)PulseTime/2.0f);

				//SIDOT - Speed is Distance over Time or more so S = V*T
				Distance = SpeedOfSound*TimeOfFlight;

				if(Distance > 0.01){
					GPIO_SetBits(LED_GPIO, RedLED_Pin);
				}
				if(Distance > 0.5){
					GPIO_SetBits(LED_GPIO, OrangeLED_Pin);
				}
				if(Distance > 1){
					GPIO_SetBits(LED_GPIO, BlueLED_Pin);
				}
				if(Distance > 3){
					GPIO_SetBits(LED_GPIO, GreenLED_Pin);
				}
			}
			Delay(20);
			resetLeds();
		}

		// 2. Motion detection
		motionRead = GPIO_ReadInputDataBit(MotionSensor_GPIO, MotionSensor_Pin);
		if(motionRead != 0) {
			GPIO_SetBits(LED_GPIO, GreenLED_Pin);
			Delay(20);
			GPIO_ResetBits(LED_GPIO, GreenLED_Pin);
			motion = "1";
	  	} else {
	  		motion = "0";
	  	}

		// 3. MESSAGE TRANSMITION

		// This is used not to transmit unnecessary amount of times
		// when values hasn't changed from last measurement
		//if(lastMotionRead != motionRead ||
		//		fabsf(lastDistance - Distance) > 0.01) {
		//}

		//create transmit message array
		//char message[32];
		//add device id
		strcpy(TX_BUF, "'id':1;");
		//convert distance
		char distance[10];
		sprintf(distance, "%0.3f", Distance);
		//add distance
		strcat(TX_BUF, distance);
		//keep the form
		strcat(TX_BUF, ";");
		//add motion
		strcat(TX_BUF, motion);
		//closing symbols
		strcat(TX_BUF, "\r\n");

		Delay(50);
		NRF24L01_Send();
		Delay(50);

		// update values
		lastDistance = Distance;
		lastMotionRead = motionRead;

	}
}
