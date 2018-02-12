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

/****��Ĵ���regдһ���ֽڣ�ͬʱ����״̬�ֽ�**************/
uint8_t SPI_RW_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	CSN(0);
	status=SPI_Receive_byte(SPI2,reg);   //select register  and write value to it
	SPI_Send_byte(SPI2,value);   
	CSN(1);
	return(status); 
}
/****��Ĵ���reg��һ���ֽڣ�ͬʱ����״̬�ֽ�**************/
uint8_t SPI_Read_Reg(uint8_t reg)
{
	uint8_t status;
	CSN(0);
	SPI_Send_byte(SPI2,reg);
	status=SPI_Receive_byte(SPI2,0);   //select register  and write value to it
	CSN(1);
	return(status);
}
/********����bytes�ֽڵ�����*************************/
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

/****************д��bytes�ֽڵ�����*******************/
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

/*���պ���������1��ʾ�������յ�������û�����ݽ��յ�**/
uint8_t nRF24L01_RxPacket(uint8_t* rx_buf)
{
    uint8_t status,revale=0;
	CE(0);
	delay1us(10);
	status=SPI_Receive_byte(SPI2,STATUS);	// ��ȡ״̬�Ĵ������ж����ݽ���״��
//	CE(0);
//	status=0x40;
	_printfLngU("STATUS :%2x\r\n",status);

	if(status & RX_DR)				// �ж��Ƿ���յ�����
	{
//		CE(1);
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
//		CE(0);
		revale =1;			//��ȡ������ɱ�־
	}
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS,status);   //���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�־
	// After receiving the data RX_DR, TX_DS, MAX_PT are set high to 1 by writing a clear interrupt flag
	CE(1);
	return revale;	
}

 /****************���ͺ���***************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	CE(0);			//StandBy Iģʽ
	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // װ�ؽ��ն˵�ַ
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // װ������
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);   		 // IRQ�շ�����ж���Ӧ��16λCRC��������
	CE(1);		 //�ø�CE���������ݷ���
	delay1us(10);
}

void RX_Mode(void)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
 
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // ʹ�ܽ���ͨ��0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);                 // ѡ����Ƶͨ��0x40

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
  	CE(1);
}

void TX_Mode(uint8_t * tx_buf)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // װ������
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // ʹ�ܽ���ͨ��0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 40);         // ѡ����Ƶͨ��0x40
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�
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

/****************** ���ú���********************************/
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
	status=SPI_Read_Reg(STATUS);	// ��ȡ״̬�Ĵ������ж����ݽ���״��
	_printfLngU("STATUS : 0x%2x\r\n",status);
	if(status&TX_DS)	/*tx_ds == 0x20*/
	{
		_printfLngU("STATUS 0x%2x\r\n",status);
		_printfLngU("\r\n�Send data:%s\r\n",RX_BUF);
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);      // Clear TX, let IRQ low;
			
	}
	else if(status&MAX_RT)
	{
		_printfLngU("�Send to reach the maximum number of sending\r\n");
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
