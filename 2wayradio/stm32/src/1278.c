#include "stm32f4xx.h"
#include "sx1276-Fsk.h"
#include "1278.h"
#include "stm32f4xx_conf.h"
#include <stdint.h>
#include "stm32f4xx_gpio.h"
typedef   uint8_t  u8  ;

char mode          = 0x01;//0x01 lora mode
char Freq_Sel      = 0x00;//434M
char Power_Sel     = 0x04;//功率 
char Lora_Rate_Sel = 0x05;//扩频因子应为6   扩频因子为5，带宽31.2K，发载波
char BandWide_Sel  = 0x05 ;//                单收单发31.2KHZ 04
char Fsk_Rate_Sel  = 0x00;//FSK模式下的扩频因子
extern u8 loadLength, wireless_send[24],TX_power;

//参数表定义
 char sx1276_FreqTbl[1][3] =  //const  u8 sx1276_FreqTbl[1][3]
{   
   //{0x61, 0x80, 0x10}, //390MHz
    //{0x66, 0xC0, 0x12}, //411MHz 原始
  
   {0x6C, 0x80, 0x00}, //434MHz    
     //{0x70, 0x80, 0x13}, //450MHz
   //  {0x83, 0x40, 0x16}, //525MHz
  //{0x2A, 0x40, 0x00}, //169MHz

};

 const char sx1276_PowerTbl[6] =
 {
    0xFF,               //20dbm
     0xFD,               //18dbm
     0xF9,               //14dbm
     0xF6,               //11dbm
     0x90,
     0x80                 //2dbm                
 /*
   0x7F,    //14dbm
    0x7E,    //13dbm
    0x7D,    //12dbm
    0x77,   //7dbm
    0x00    //10.8-15+0
  */

 };

 const unsigned char sx1276_SpreadFactorTbl[12] =
 {
   1,2,3,4,5,6,7,8,9,10,11,12
 };

 const unsigned char sx1276_LoRaBwTbl[10] =
 {
 //7.8KHz,10.4KHz,15.6KHz,20.8KHz,31.2KHz,41.7KHz,62.5KHz,125KHz,250KHz,500KHz
   0,1,2,3,4,5,6,7,8,9
 };

 u8 SpiInOut( u8 outData )
 {
     /* Send SPIy data */
     SPI_I2S_SendData( SPI_INTERFACE, outData );
     while( SPI_I2S_GetFlagStatus( SPI_INTERFACE, SPI_I2S_FLAG_RXNE ) == RESET );
     return SPI_I2S_ReceiveData( SPI_INTERFACE );
 }


 void SX1276WriteBuffer( u8 addr, volatile char *buffer, u16 size )
 {
     u16 i;

     //NSS = 0;
     GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );

     SpiInOut( addr | 0x80 );
     for( i = 0; i < size; i++ )
     {
         SpiInOut( buffer[i] );
     }

     //NSS = 1;
     GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
 }

 void SX1276ReadBuffer( u8 addr, volatile char *buffer, u16 size )
 {
     u16 i;

     //NSS = 0;
     GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );

     SpiInOut( addr & 0x7F );

     for( i = 0; i < size; i++ )
     {
         buffer[i] = SpiInOut( 0 );
     }

     //NSS = 1;
     GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
 }

 void SX1276Write( u8 addr, char data )
 {
     SX1276WriteBuffer( addr, &data, 1 );
 }

 /*void SX1276Read( u8 addr, u8 *data )
 {
     SX1276ReadBuffer( addr, data, 1 );
 }*/

 u8 SX1276Read( u8 addr, char *data )
 {
    SX1276ReadBuffer( addr, data, 1 );
 	 return *data;
 }



 void SX1276WriteFifo( char *buffer, u16 size )
 {
     SX1276WriteBuffer( 0, buffer, size );
 }

 void SX1276ReadFifo( char *buffer, u16 size )
 {
     SX1276ReadBuffer( 0, buffer, size );
 }

 /**********************************************************
 **名字:     sx1276__Standby
 **功能:     Entry standby mode
 **输入:     None
 **输出:     None
 **********************************************************/
 void sx1276_Standby(void)
 { char test2;
   SX1276Write(LR_RegOpMode,0x09);  //待机//低频模式
  
   //SX1276Write(LR_RegOpMode,0x01);//待机//高频模式
 }

 /**********************************************************
 **名字:     sx1276__Sleep
 **功能:     Entry sleep mode
 **输入:     None
 **输出:     None
 **********************************************************/
 void sx1276_Sleep(void)
 {    char test2;

   SX1276Write(LR_RegOpMode,0x08);  //睡眠//低频模式
    
   //SX1276Write(LR_RegOpMode,0x00);//睡眠//高频模式
 }





 /*********************************************************/
 //扩频模式函数
 /*********************************************************/
 /**********************************************************
 **名字:     sx1276__EntryLoRa
 **功能:     配置进入扩频模式
 **输入:     None
 **输出:     None
 **********************************************************/
 void sx1276_EntryLoRa(void)
 {
   SX1276Write(LR_RegOpMode,0x88);  //低频模式
   //SX1276Write(LR_RegOpMode,0x80);//高频模式
 }

 /**********************************************************
 **名字:     sx1276__LoRaClearIrq
 **功能:     清楚所有中断
 **输入:     None
 **输出:     None
 **********************************************************/
 void sx1276_LoRaClearIrq(void)
 {
   SX1276Write(LR_RegIrqFlags,0xFF);
 }

 /**********************************************************
 **名字:     sx1276__LoRaEntryRx
 **功能:     进入LORa RX模式
 **输入:     None
 **输出:     0-超时错误
 **********************************************************/
 u8 sx1276_LoRaEntryRx(void)
 { u8 temp,addr;
  
  // sx1276_ConfigRX();               //基本参数配置
     sx1276_Standby();      //进入待机模式
 	//delay_ms(1);

  // SX1276Write(REG_LR_PADAC,0x84);         //正常的Rx
   SX1276Write(LR_RegHopPeriod,0x00);      //RegHopPeriod 无跳频
   SX1276Write(REG_LR_DIOMAPPING1,0x01);   //DIO0=00, DIO1=00, DIO2=00, DIO3=01
   SX1276Write(LR_RegIrqFlagsMask,0x3F);   //打开RxDone中断&超时0011 1111
   sx1276_LoRaClearIrq();

 	SX1276Write(LR_RegPayloadLength,loadLength);    //RegPayloadLength  60字节(在扩频因子为6时数据大于一字节此寄存器必须配置)

 	SX1276Write(LR_RegFifoRxBaseAddr,0);
 	SX1276Write(LR_RegFifoAddrPtr,0);
 	//addr=SX1276Read(LR_RegFifoRxBaseAddr,&addr);//Read RxBaseAddr
       //  SX1276Write(LR_RegFifoAddrPtr,addr);    //RxBaseAddr -> FiFoAddrPtr

 	//SX1276Write(LR_RegOpMode,0x8E);         //单Rx模式//低频模式10001110   序文超时会进入待机


 	SX1276Write(LR_RegOpMode,0x0D);     //fsk 连续Rx模式//低频模式

  /* SysTime = 0;
 	while(1)
 	{
 		if((SX1276Read(LR_RegModemStat,&temp)&0x04)==0x04)//Rx-on going RegModemStat//RX进行中
 			return 1;
 			break;
 		if(SysTime>=3)
 			return 0;                  //超时错误
 	}*/

 }




 /**********************************************************
 **名字:     sx1276__LoRaEntryTx
 **功能:     Entry Tx mode
 **输入:     None
 **输出:     0-超时错误
 **********************************************************/
 u8 sx1276_LoRaEntryTx(void)
 { u8 addr,temp;

 		sx1276_Standby();      //进入待机模式
   //  for(u8 i=250;i!=0;i--);//延时

 //	delay_ms(1);//?

 	SX1276Write(REG_LR_DIOMAPPING1,0x41);   //DIO0=01, DIO1=00, DIO2=00, DIO3=01

 	//SX1276Write(REG_LR_PADAC,0x87);       //Tx for 20dBm
   SX1276Write(LR_RegHopPeriod,0x00);      //RegHopPeriod 无跳频


   sx1276_LoRaClearIrq();
   SX1276Write(LR_RegIrqFlagsMask,0xF7);   //打开TxDone中断
   SX1276Write(LR_RegPayloadLength,loadLength);    //RegPayloadLength  负载长度60字节


 	SX1276Write(LR_RegFifoTxBaseAddr,0);
 	SX1276Write(LR_RegFifoAddrPtr,0);
   //addr = SX1276Read(LR_RegFifoTxBaseAddr,&addr);//FiFo数据缓冲区中发送调制器的写入基地址
   //SX1276Write(LR_RegFifoAddrPtr,addr);    //Fifo数据缓冲区中SPI接口地址指针

 	//SysTime = 0;
 /*	while(1)
 	{
 		temp=SX1276Read(LR_RegPayloadLength,&temp);
 		if(temp==21)
 		{
 			return 1;
 			break;
 		}
 		if(SysTime>=3)
 			return 0;
 	}*/
 }

 

 
 /**********************************************************
 **名字:    sx1276__LoRaTxPacket
 **功能:    Send data in LoRa mode
 **输入:    None
 **输出:    1- Send over
 **********************************************************/
 u8 sx1276_LoRaTxPacket(void)
 {  
 	SX1276WriteBuffer( 0x00, wireless_send, loadLength); 
 	SX1276Write(LR_RegOpMode,0x8B);       //Tx Mode
 }

 
 
//#define RF_OPMODE_MODULATIONTYPE_FSK   0
//#define RF_OPMODE_SLEEP  0
//#define  RF_OPMODE_FREQMODE_ACCESS_LF  0x08
 tFskSettings FskSettings={
   411,   100,   1,   1,1,1,0,1,24
 };
  /*************************************************************************************
 **名字:     sx1278__FSK_Config
 **功能:     sx1278_ base config
 **输入:     mode
 **输出:     None
 *************************************************************************************/
 void sx1278_Fsk_Config(void)
 {
   sx1276_Sleep();//改变当前模式必须进入睡眠模式
   //delay_ms(1);//延时
   
   //fsk模式  
  SX1276Write(LR_RegOpMode,0x01);  //待机 FSK
   
 SX1276WriteBuffer(LR_RegFrMsb,sx1276_FreqTbl[Freq_Sel],3);//设置频率
 
 //设置基本参数
        SX1276Write(0x26,0x04);   //0x08关AGC开低速率优化       0x04auto agc    0x00关AGC不开低速率优化
	SX1276Write(LR_RegPaConfig,sx1276_PowerTbl[Power_Sel]);//设置输出增益
	SX1276Write(LR_RegOcp,0x0B);                              //RegOcp,关闭过流保护
	SX1276Write(LR_RegLna,0x20);                          //RegLNA,使能低噪声放大器  原来是0x13
     
   // if( FskSettings.AfcOn == true )    
     {
        SX1276Write(RegRxConfig, RF_RXCONFIG_RESTARTRXONCOLLISION_OFF | RF_RXCONFIG_AFCAUTO_OFF |
                              RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);
    }
   
      SX1276Write(RegPreambleLsb,0x08);
      
    
      SX1276Write(RegPreambleDetect , RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_2 |
                                RF_PREAMBLEDETECT_DETECTORTOL_10);
    
      SX1276Write(RegRssiThresh , 0xFF); 
   
      SX1276Write(RegSyncConfig , RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | RF_SYNCCONFIG_SYNC_ON |   RF_SYNCCONFIG_SYNCSIZE_4);
 
     SX1276Write(RegSyncValue1 , 0x69);
     SX1276Write(RegSyncValue2 , 0x81);
     SX1276Write(RegSyncValue3 , 0x7E);
     SX1276Write(RegSyncValue4 , 0x96);
     
     SX1276Write(RegPacketConfig1 , RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF_PACKETCONFIG1_DCFREE_OFF |
                               ( FskSettings.CrcOn << 4 ) | RF_PACKETCONFIG1_CRCAUTOCLEAR_ON |
                               RF_PACKETCONFIG1_ADDRSFILTERING_OFF | RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT);
    //SX1276FskGetPacketCrcOn( ); // Update CrcOn on FskSettings
     SX1276Write(RegPayloadLength , FskSettings.PayloadLength);
    
     // we can now update the registers with our configuration
   // SX1276WriteBuffer( REG_OPMODE, SX1276Regs + 1, 0x70 - 1 );

     
 }
 /*************************************************************************************
 **名字:     sx1276__Config
 **功能:     sx1276_ base config
 **输入:     mode
 **输出:     None
 *************************************************************************************/
void sx1276_Config(void)
{    char test;
    u8 i;

    sx1276_Sleep();//改变当前模式必须进入睡眠模式
 	//delay_ms(1);//延时

    //扩频模式
	sx1276_EntryLoRa();
	//SX1276Write(0x5904);//Change digital regulator form 1.6V to 1.47V: see errata note
	SX1276WriteBuffer(LR_RegFrMsb,sx1276_FreqTbl[Freq_Sel],3);//设置频率
        //设置基本参数
        SX1276Write(0x26,0x04);   //0x08关AGC开低速率优化       0x04auto agc    0x00关AGC不开低速率优化	      
          SX1276Write(LR_RegPaConfig,sx1276_PowerTbl[Power_Sel]);//设置输出增益	         
        SX1276Write(LR_RegOcp,0x0B);                              //RegOcp,关闭过流保护
	SX1276Write(LR_RegLna,0x20);                          //RegLNA,使能低噪声放大器  原来是0x13

      if(TX_power==0) 
          SX1276Write(LR_RegPaConfig,sx1276_PowerTbl[Power_Sel]);//设置输出增益
	else 
          SX1276Write(LR_RegPaConfig,sx1276_PowerTbl[2]);//设置输出大功率

	if(sx1276_SpreadFactorTbl[Lora_Rate_Sel]==6)        //扩频因子=6
	{
		u8 tmp;
		SX1276Write(LR_RegModemConfig1,((sx1276_LoRaBwTbl[BandWide_Sel]<<4)+(CR<<1)+0x01));//隐式报头使能 CRC使能(0x02) & 纠错编码率 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
		//SX1276Write(LR_RegModemConfig2,((sx1276_SpreadFactorTbl[Lora_Rate_Sel]<<4)+(CRC<<2)+0x03));
                SX1276Write(LR_RegModemConfig2,((sx1276_SpreadFactorTbl[Lora_Rate_Sel]<<4)+0x01));  //CRC  disable
	 	tmp = SX1276Read(0x31,&tmp);
	 	tmp &= 0xF8;		//1111 1000
	 	tmp |= 0x05;		//0000 0101
		 SX1276Write(0x31,tmp);  //因子优化
		 SX1276Write(0x37,0x0C);//因子优化
	}
	else
	{
		SX1276Write(LR_RegModemConfig1,((sx1276_LoRaBwTbl[BandWide_Sel]<<4)+(CR<<1)+0x00));//显示报头 CRC使能(0x02) & 纠错编码率 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
		SX1276Write(LR_RegModemConfig2,((sx1276_SpreadFactorTbl[Lora_Rate_Sel]<<4)+(CRC<<2)+0x03));  //扩频因子 &  LNA gain set by the internal AGC loop
	}
	SX1276Write(LR_RegSymbTimeoutLsb,0xFF);//RegSymbTimeoutLsb Timeout = 0x3FF(Max)

	SX1276Write(LR_RegPreambleMsb,0x00);   //前导码 
	SX1276Write(LR_RegPreambleLsb,0x0F); //      前导码 

       SX1276Read(LR_RegModemConfig1,&test);

	SX1276Write(REG_LR_DIOMAPPING2,0x01);  //RegDioMapping2 DIO5=00, DIO4=01
										//DIO5 clkout DIO4 PLLlock DIO3 Vaildheader
										//DIO2 Fhsschang DIO1 Fhsschangechannel
	//SX1276Write(REG_LR_DIOMAPPING1 ,0x20); //DIO0 TxDone

   sx1276_Standby();               //进入待机模式
}