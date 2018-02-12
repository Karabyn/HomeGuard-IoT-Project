#include "stm32f0xx.h"
#include "NRF24L01.h"
#include "Debug.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_spi.h"
#include "stdio.h"
#include "string.h"

#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};  // Define a static send address

uint8_t RX_BUF[TX_PLOAD_WIDTH];

uint8_t TX_BUF[TX_PLOAD_WIDTH];

static void Initial_SPI(SPI_TypeDef* SPIx)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	if(SPIx==SPI1)
	{
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB ,ENABLE);
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
//
//
//		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3;
//		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
//		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
								 
		GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else if(SPIx==SPI2)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_0);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_0);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_0);
	
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		
		GPIO_Init(GPIOB, &GPIO_InitStruct);
	}

	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;

	SPI_Init(SPIx, &SPI_InitStruct);
	SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
	SPI_Cmd(SPIx, ENABLE);
}

static void SPI_Send_byte(SPI_TypeDef* SPIx,uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)==RESET);
	SPI_SendData8(SPIx,data);

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)==RESET);
	SPI_ReceiveData8(SPIx);
}

static uint8_t SPI_Receive_byte(SPI_TypeDef* SPIx,uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)==RESET);
	SPI_SendData8(SPIx,data);

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_ReceiveData8(SPIx);
}

static void delay1us(uint8_t t)
{
	while(--t);
} 

/****向寄存器reg写一个字节，同时返回状态字节**************/
uint8_t SPI_RW_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	CSN(0);
	status=SPI_Receive_byte(SPI2,reg);   //select register  and write value to it
	SPI_Send_byte(SPI2,value);   
	CSN(1);
	return(status); 
}
/****向寄存器reg读一个字节，同时返回状态字节**************/
uint8_t SPI_Read_Reg(uint8_t reg)
{
	uint8_t status;
	CSN(0);
	SPI_Send_byte(SPI2,reg);
	status=SPI_Receive_byte(SPI2,0);   //select register  and write value to it
	CSN(1);
	return(status);
}
/********读出bytes字节的数据*************************/
uint8_t SPI_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(SPI2,reg);       
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr]=SPI_Receive_byte(SPI2,0);
	CSN(1);
	return(status);
}

/****************写入bytes字节的数据*******************/
uint8_t SPI_Write_Buf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(SPI2,reg); 
	delay1us(1);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(SPI2,*pBuf++);
	CSN(1);
	return(status);
}

/*接收函数，返回1表示有数据收到，否则没有数据接收到**/
uint8_t nRF24L01_RxPacket(uint8_t* rx_buf)
{
    uint8_t status,revale=0;
	CE(0);
	delay1us(10);
	status=SPI_Receive_byte(SPI2,STATUS);	// 读取状态寄存其来判断数据接收状况
//	CE(0);
//	status=0x40;
	_printfLngU("STATUS :%2x\r\n",status);

	if(status & RX_DR)				// 判断是否接收到数据
	{
//		CE(1);
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
//		CE(0);
		revale =1;			//读取数据完成标志
	}
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS,status);   //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
	// After receiving the data RX_DR, TX_DS, MAX_PT are set high to 1 by writing a clear interrupt flag
	CE(1);
	return revale;	
}

 /****************发送函数***************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	CE(0);			//StandBy I模式
	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送
	CE(1);		 //置高CE，激发数据发送
	delay1us(10);
}

void RX_Mode(void)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
 
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // 使能接收通道0自动应答
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // 使能接收通道0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);                 // 选择射频通道0x40

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式
  	CE(1);
}

void TX_Mode(uint8_t * tx_buf)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // 使能接收通道0自动应答
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // 使能接收通道0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发10次
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);         // 选择射频通道0x40
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电
	CE(1);
	delay1us(10);
} 

/*
 *Initialize nRF24L01 pins:
 *CE - PB3
 *IRQ - PB4
 *CSN - PB5
 *Initialize SPI2.
 */
void nRF24L01_Initial(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	/*CE CSN Initial*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	/*IRQ CSN Initial*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd	=	GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
//	CE(0);			//nRF24L01_CE=0;			 chip enable
//	nRF24L01_Config();

//	CSN(0);			//nRF24L01_CSN=1;			 Spi enable
	Initial_SPI(SPI2);
	//nRF24L01_Config();
}

/****************** 配置函数********************************/
void nRF24L01_Config(void)
{
          //initial io
	//CE(0);          //        CE=0 ;chip enable
	//CSN(1);       //CSN=1   Spi disable
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e); // Set PWR_UP bit, enable CRC(2 bytes) &Prim:RX. RX_DR enabled..
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f); // Enable Pipe0
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_AW, 0x02); // Setup address width=5 bytes
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP,0x07); // TX_PWR:0dBm, Datarate:2Mbps,
}
 
void NRF24L01_Send(void)
{
    uint8_t status=0x00;
	CE(0);
	TX_Mode(TX_BUF);
	while(IRQ);
	
	delay1us(10);
	status=SPI_Read_Reg(STATUS);	// 读取状态寄存其来判断数据接收状况
	_printfLngU("STATUS : 0x%2x\r\n",status);
	if(status&TX_DS)	/*tx_ds == 0x20*/
	{
		_printfLngU("STATUS 0x%2x\r\n",status);
		_printfLngU("\r\n稴end data:%s\r\n",RX_BUF);
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);      // Clear TX, let IRQ low;
			
	}
	else if(status&MAX_RT)
	{
		_printfLngU("稴end to reach the maximum number of sending\r\n");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);      // Clear TX, let IRQ low;
	}
	CE(1);
}

void NRF24L01_Receive(void)
{   
	uint8_t status=0x01;
    //uint8_t i, status=0x01;
	Initial_SPI(SPI2);
	RX_Mode();
	CE(0);

	delay1us(10);

	status=SPI_Read_Reg(STATUS);
	char debugString[100]="";
	sprintf(debugString, "STATUS 0x%2x\r\n",status);
	debugSend(debugString);

	if(status & 0x40)
	{
		//debugSend("Accepted successfully\r\n");
		SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

		char debugString[100]="";
		// The below line was displaying RX_BUF[0] in hex  ("%x")
		//sprintf(debugString, "\r\n i=%d,Received data: %x\r\n",i, RX_BUF[0]);

		strcat(debugString, "RX_BUF:");
		strcat(debugString, RX_BUF);
		debugSend(debugString);

		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);
	}

	debugSend("END receive\r\n");
	CE(1);
}
