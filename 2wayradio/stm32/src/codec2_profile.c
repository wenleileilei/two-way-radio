/*---------------------------------------------------------------------------*\

  FILE........: codec2_profile.c
  AUTHOR......: David Rowe
  DATE CREATED: 30 May 2013

  Profiling Codec 2 operation on the STM32F4.

\*---------------------------------------------------------------------------*/

/*
  Copyright (C) 2014 David Rowe

  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License version 2.1, as
  published by the Free Software Foundation.  This program is
  distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program; if not, see <http://www.gnu.org/licenses/>.
*/



#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "gdb_stdio.h"
#include "codec2.h"
#include "dump.h"
#include "sine.h"
#include "machdep.h"
#include  "1278.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_it.h"


#ifdef __EMBEDDED__
#define printf gdb_stdio_printf
#define fopen gdb_stdio_fopen
#define fclose gdb_stdio_fclose
#define fread gdb_stdio_fread
#define fwrite gdb_stdio_fwrite
#endif


#define USART1_Remap_PB6_PB7

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

 //short in[320],out[320];
//static short I2S_Rx_ptr=0;
//short I2S_Rx_Buffer[320],I2S_Tx_Buffer[320];
//short I2S_Rx_Buffer_2[320],I2S_Tx_Buffer_2[320];

#define MODEM_FSK    0		
#define MODEM_LORA   1


volatile float RSSI;
char Get_NIRQ=0;
char send_over=0;
char receive=0;
 char receive_over,receive_over2; 
  char en_code,en_decode;
int loadLength=24;   //下一行同步改
volatile unsigned char wireless_receive[24],wireless_send[24],uart_receive_0[252],uart_receive_1[252],uart_send_0[252],uart_send_1[252];

   uint32_t buffer0[2560], buffer1[2560],buffer2[2560],buffer3[2560];
   uint16_t buffer_encode_0[2560],buffer_decode_0[2560],buffer_encode_1[2560],buffer_decode_1[2560];  
volatile int TX_RX_Flag=1;
void Voice_Realtime_Transfer(void);
volatile int TX_start,RX_start,FM,again,TX_power;
float SX1276ReadRssi(int modem);


static void c2demo(int mode, short inputfile[], short outputfile[])
{
	   struct CODEC2 *codec2;
	    short        inbuf[320], outbuf[320];
	    unsigned char *bits;
	    int            nsam, nbit;
	   // FILE          *fin, *fout;
	    int            frame;
	    PROFILE_VAR(enc_start, dec_start);

	    codec2 = codec2_create(mode);
	    nsam = codec2_samples_per_frame(codec2);
	   // outbuf = (short*)malloc(nsam*sizeof(short));
	   // inbuf = (short*)malloc(nsam*sizeof(short));
	    nbit = codec2_bits_per_frame(codec2);
	    bits = (unsigned char*)malloc(nbit*sizeof(char));


	    /******1600bps   80±??1??************/

	   /* for(int i=0;i<200;i++)
	    {   for(int ii=0;ii<320;ii++)
	         {
	            inbuf[ii]=inputfile[i*320+ii*2];
	            inbuf[ii]=(inbuf[ii]<<8)+inputfile[i*320+ii*2+1];
	          }

        PROFILE_SAMPLE(enc_start);*/
       for(int q=0;q<100;q++)
       {  for(int i=0;i<320;i++)
	    	  inbuf[i]=inputfile[i];


        	codec2_encode(codec2, bits, inbuf);

	      	codec2_decode(codec2,outbuf, bits);

	      	for(int i=0;i<320;i++)
	      		outputfile[i]=outbuf[i];

       }

}

void Sysclock_Init(void)
{


}


 void TIM8_PWM_Init(u32 arr,u32 psc,u32 CCR2_Val)
  {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM3时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTA时钟

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); //GPIOA6复用为定时器3

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PC7

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器3

	//初始化TIM14 Channel1 PWM模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
        TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //éè??í¨μà2μ?μ???ì?±??μ￡?ê?3?áííaò???????±èμ?PWM
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器

      TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能
      TIM_CtrlPWMOutputs(TIM8, ENABLE);
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8

  } 

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟

        TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断	

	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
         TIM_Cmd(TIM3,ENABLE); //使能定时器3
}


char  encode_en=0;
char aaa=0;
char buffer_select0,buffer_select1;
int I2S_Rx_ptr=0;
int voice_test;

void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ê1?üGPIOAê±?ó

  //GPIOF9,F103?ê??ˉéè??
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB,GPIO_Pin_0);
}





void RCC_Configuration(void)
{
	RCC_ClocksTypeDef RCC_ClockFreq;
	  /* This function fills the RCC_ClockFreq structure with the current
	  frequencies of different on chip clocks (for debug purpose) **************/
	  RCC_GetClocksFreq(&RCC_ClockFreq);


	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);


	#ifdef USART1_Remap_PB6_PB7
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	#else
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	#endif
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

}

/*******************new****************************************/

void WM8974_I2C_write(unsigned char data1,unsigned char data2)
{
      I2C_GenerateSTART(I2C3, ENABLE);
      while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));
      I2C_Send7bitAddress(I2C3,0x34, I2C_Direction_Transmitter);// 8974's  address
      while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
       I2C_SendData(I2C3, data1);
      while(!I2C_CheckEvent(I2C3,  I2C_EVENT_MASTER_BYTE_TRANSMITTED ));
      I2C_SendData(I2C3, data2);
       while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void WM8974()
{
    volatile int aa=0x10;//0x9D;//0x91;
     volatile int bb=0x10;
    while(I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

    WM8974_I2C_write(0,0); //RESET
    WM8974_I2C_write(0x03,0x1F);//Power management1                     DEC1   B100011111
    WM8974_I2C_write(0x04,0x15);//Power management2                     DEC2   B000010101
    WM8974_I2C_write(0x06,0xFF);//Power management3                     DEC3   B011111111   (bit7=1 monoout Enable)
    WM8974_I2C_write(0x08,aa);//Audio Interface                       DEC4   I2S format
    WM8974_I2C_write(0x0A,0x00);//Companding ctrl                       DEC5       0x01  回环   无效？
    WM8974_I2C_write(0x0C,bb);//Clock Gen ctrl                        DEC6       BCLK and FRAME clock are outputs    0x04inputs
    WM8974_I2C_write(0x0E,0x0A);//Additional ctrl                       DEC7   B000001010
    WM8974_I2C_write(0x10,0x06);//GPIO                                  DEC8
    WM8974_I2C_write(0x14,0x00);//DAC Control                           DEC10
    WM8974_I2C_write(0x16,0xF3);//DAC digital Vol                       DEC11  B011110011
  //WM8974_I2C_write(0x1D,0x80);//ADC Control                           DEC14  B110000000  high pass
    WM8974_I2C_write(0x1E,0xFF);//ADC Digital Vol                       DEC15  B011111111
  //WM8974_I2C_write(0x40,0x00);//ALC control1                          DEC32
    WM8974_I2C_write(0x46,0x08);//ALC Noise Gate Control                DEC35  B000001000
  //WM8974_I2C_write(0x48,0x08);//PLL N                                 DEC36  B000001000
  //WM8974_I2C_write(0x4A,0x31);//PLL K1                                DEC37
  //WM8974_I2C_write(0x4C,0x26);//PLL K2                                DEC38
  //WM8974_I2C_write(0x4E,0xE8);//PLL K3                                DEC39
    WM8974_I2C_write(0x50,0x00);//Input control                         DEC40  0x02 monoout -10dB   0x00 0dB
    WM8974_I2C_write(0x58,0x02);//Input control                         DEC44  0x02麦克风输入通  0.9xAVDD
    WM8974_I2C_write(0x5A,0x20);//INP PGA gain ctrl                     DEC45  0x00=-12dB  0x01=-11.25dB 0x10=0dB    0x3F=35.25dB
  //WM8974_I2C_write(0x21,0xFF);//ALC control2                          DEC46
    WM8974_I2C_write(0x5F,0x70);//ADC Boost ctrl                        DEC47
    WM8974_I2C_write(0x62,0x00);//Output ctrl                           DEC49  B000001110
   if(FM==1)
      WM8974_I2C_write(0x64,0x02);//SPK mixer ctrl                        DEC50  0x01 DAC直通SPK(when SX1278 work)    0x02 MICP直通SPK(when AT1846S work)
  else
        WM8974_I2C_write(0x64,0x01);
   WM8974_I2C_write(0x6C,0x3F);//SPK volume ctrl                       DEC54
    WM8974_I2C_write(0x70,0x02);//momo mixer control                    DEC56  byapass path to mono

    I2C_GenerateSTOP(I2C3, ENABLE);
  //I2C_AcknowledgeConfig(I2C1,ENABLE);
}

void I2C3_Congiguration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
   I2C_InitTypeDef  I2C_InitStructure;

  RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,ENABLE);


 GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
 GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);



         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
         GPIO_Init(GPIOA, &GPIO_InitStructure);

         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
         GPIO_Init(GPIOC, &GPIO_InitStructure);


           /* I2C 配置 */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 =0x0A;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 200000;

  /* 使能 I2C1 */
    I2C_Cmd(I2C3, ENABLE);

  /* I2C1 初始化 */
     I2C_Init(I2C3, &I2C_InitStructure);


}



 void I2S3_Mode_Config(const uint16_t _usStandard, const uint16_t _usWordLen, const uint32_t _usAudioFreq  )
    {
      I2S_InitTypeDef I2S_InitStructure;
      uint32_t n = 0;
      FlagStatus status = RESET;
      /**
      * For I2S mode, make sure that either:
      *   - I2S PLL is configured using the functions RCC_I2SCLKConfig
      *     (RCC_I2S2CLKSource_PLLI2S),
      *   RCC_PLLI2SCmd(ENABLE) and RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY).
      */
      RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
      RCC_PLLI2SCmd(ENABLE);
      for (n = 0; n < 500; n++) {
        status = RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY);
        if (status == 1)break;
      }
      
      /* 打开 I2S3 APB1 时钟 */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//xwl add
  	 
    
      //RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
      
      /* 复位 SPI3 外设到缺省状态 */
      SPI_I2S_DeInit(SPI3);
      /* I2S3 外设配置 */
      /* 配置 I2S 工作模式 */
      I2S_InitStructure.I2S_Mode =I2S_Mode_MasterTx  ;
      /* 接口标准 */
      I2S_InitStructure.I2S_Standard = _usStandard;
      /* 数据格式，16bit */
      I2S_InitStructure.I2S_DataFormat = _usWordLen;
      /* 主时钟模式 */
      I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
      /* 音频采样频率 */
      I2S_InitStructure.I2S_AudioFreq = _usAudioFreq;
      I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;
      I2S_Init(SPI3, &I2S_InitStructure);
      /* 使能 SPI3/I2S3 外设 */
    
      
    
        I2S_Cmd(SPI3, ENABLE);
      //I2S_Cmd(I2S3ext, ENABLE);//xwl add
    }






#define WM8974_I2Sx_SPI                          SPI3
#define I2Sx_TX_DMA_STREAM                 DMA1_Stream5
#define I2Sx_TX_DMA_CHANNEL                DMA_Channel_0
#define I2Sx_DMA_CLK                     RCC_AHB1Periph_DMA1
#define I2Sx_TX_DMA_STREAM_IRQn         DMA1_Stream5_IRQn




#define WM8974_I2Sx_ext                          I2S3ext
#define I2Sxext_RX_DMA_STREAM                    DMA1_Stream0
#define I2Sxext_RX_DMA_CHANNEL                   DMA_Channel_3
#define I2Sxext_RX_DMA_STREAM_IRQn               DMA1_Stream0_IRQn
#define I2Sxext_RX_DMA_IT_TCIF                   DMA_IT_HTIF5

 void I2S3ext_Mode_Config(const uint16_t _usStandard,
                          const uint16_t _usWordLen,const uint32_t _usAudioFreq)
 {
   
   /* 打开 I2S3 APB1 时钟 */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//xwl add
   
   I2S_InitTypeDef I2Sext_InitStructure;
   /* I2S2 外设配置 */
   /* 配置 I2S 工作模式 */
   I2Sext_InitStructure.I2S_Mode = I2S_Mode_MasterTx; 
   /* 接口标准 */
   I2Sext_InitStructure.I2S_Standard = _usStandard;
    /* 数据格式，16bit */
   I2Sext_InitStructure.I2S_DataFormat = _usWordLen;
   /* 主时钟模式 */
   I2Sext_InitStructure.I2S_MCLKOutput =I2S_MCLKOutput_Disable ;
   /* 音频采样频率 */
   I2Sext_InitStructure.I2S_AudioFreq = _usAudioFreq;
   I2Sext_InitStructure.I2S_CPOL = I2S_CPOL_Low;
   I2S_FullDuplexConfig(WM8974_I2Sx_ext, &I2Sext_InitStructure);
   /* 使能 SPI2/I2S2 外设 */

   I2S_Cmd(WM8974_I2Sx_ext, ENABLE); 
 }


#define WM8974_LRC_GPIO_CLK        RCC_AHB1Periph_GPIOA
#define WM8974_BCLK_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define WM8974_ADCDAT_GPIO_CLK     RCC_AHB1Periph_GPIOC
#define WM8974_DACDAT_GPIO_CLK     RCC_AHB1Periph_GPIOC
#define WM8974_MCLK_GPIO_CLK       RCC_AHB1Periph_GPIOC


#define WM8974_LRC_PIN           GPIO_Pin_15    //PA15
#define WM8974_BCLK_PIN          GPIO_Pin_10    //PC10
#define WM8974_ADCDAT_PIN        GPIO_Pin_11   //PC11
#define WM8974_DACDAT_PIN        GPIO_Pin_12   //PC12
#define WM8974_MCLK_PIN          GPIO_Pin_7     //PC7


#define WM8974_LRC_PORT       GPIOA
#define WM8974_BCLK_PORT      GPIOC
#define WM8974_ADCDAT_PORT    GPIOC
#define WM8974_DACDAT_PORT    GPIOC
#define WM8974_MCLK_PORT      GPIOC

#define WM8974_LRC_SOURCE     GPIO_PinSource15
#define WM8974_BCLK_SOURCE      GPIO_PinSource10
#define WM8974_ADCDAT_SOURCE    GPIO_PinSource11
#define WM8974_DACDAT_SOURCE    GPIO_PinSource12
#define WM8974_MCLK_SOURCE      GPIO_PinSource7

#define WM8974_LRC_AF            GPIO_AF_SPI3
#define WM8974_BCLK_AF           GPIO_AF_SPI3
#define WM8974_ADCDAT_AF         GPIO_AF_SPI3  //?
#define WM8974_DACDAT_AF         GPIO_AF_SPI3
#define WM8974_MCLK_AF           GPIO_AF_SPI3

/**
	* @brief  配置GPIO引脚用于codec应用
	* @param  无
	* @retval 无
	*/
void I2S_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

/**
	* I2S总线传输音频数据口线
	* WM8974_LRC    -> PA15/I2S2_WS
	* WM8974_BCLK   -> PC10/I2S2_CK
	* WM8974_ADCDAT -> PC11/I2S2ext_SD
	* WM8974_DACDAT -> PC12/I2S2_SD
	* WM8974_MCLK   -> PC7/I2S2_MCK
	*/
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(WM8974_LRC_GPIO_CLK|WM8974_BCLK_GPIO_CLK| \
                         WM8974_ADCDAT_GPIO_CLK|WM8974_DACDAT_GPIO_CLK| \
	                       WM8974_MCLK_GPIO_CLK, ENABLE);
         RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = WM8974_LRC_PIN;
	GPIO_Init(WM8974_LRC_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = WM8974_BCLK_PIN;
	GPIO_Init(WM8974_BCLK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = WM8974_ADCDAT_PIN;
	GPIO_Init(WM8974_ADCDAT_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = WM8974_DACDAT_PIN;
	GPIO_Init(WM8974_DACDAT_PORT, &GPIO_InitStructure);
        
    

	GPIO_InitStructure.GPIO_Pin = WM8974_MCLK_PIN;
	GPIO_Init(WM8974_MCLK_PORT, &GPIO_InitStructure);

	/* Connect pins to I2S peripheral  */
	GPIO_PinAFConfig(WM8974_LRC_PORT,    WM8974_LRC_SOURCE,    WM8974_LRC_AF);
	GPIO_PinAFConfig(WM8974_BCLK_PORT,   WM8974_BCLK_SOURCE,   WM8974_BCLK_AF);
	GPIO_PinAFConfig(WM8974_ADCDAT_PORT, WM8974_ADCDAT_SOURCE, WM8974_ADCDAT_AF);
	GPIO_PinAFConfig(WM8974_DACDAT_PORT, WM8974_DACDAT_SOURCE, WM8974_DACDAT_AF);
	GPIO_PinAFConfig(WM8974_MCLK_PORT,   WM8974_MCLK_SOURCE,   WM8974_MCLK_AF);
        //GPIO_PinAFConfig(WM8974_MCLK_PORT,   WM8974_MCLK_SOURCE,   GPIO_AF_TIM8);

}



#define CLOCK 72/8 //ê±?ó=72M

void delay_us(unsigned int us)
{
	u8 n;

	while(us--)for(n=0;n<CLOCK;n++);
}


void delay_ms(unsigned int ms)
{
	while(ms--)delay_us(1000);
}

void SPI1_GPIO_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);


  /*SCK,MOSI*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
    /*NSS*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  
  /*MISO*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用
    GPIO_InitStructure.GPIO_OType = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
 
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //
 
  /* SPI1 configuration */
  //SPI_Cmd(SPI1,DISABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS =  SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);


  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
  }



   
void RF_switch1278()
{
   GPIO_SetBits(GPIOC, GPIO_Pin_4); //RF_switch to 1278
}
void RF_switch1846()
{
   GPIO_ResetBits(GPIOC, GPIO_Pin_4); //RF_switch to 1846
}


void PA_En()
{
  
 GPIO_ResetBits(GPIOB, GPIO_Pin_4); //PA_EN //
  GPIO_SetBits(GPIOD, GPIO_Pin_2); //LNA_En
}
void PaOff()
{
  GPIO_SetBits(GPIOB, GPIO_Pin_4); //PA_off
}

void LNA_En()
{
 
   //GPIO_SetBits(GPIOB, GPIO_Pin_4); //PA_off
  GPIO_ResetBits(GPIOD, GPIO_Pin_2); //LNA_En
}

void PAandLNA_En()
{
 
GPIO_ResetBits(GPIOB, GPIO_Pin_4); //PA_EN 
  GPIO_ResetBits(GPIOD, GPIO_Pin_2); //LNA_En
}


void PA_Trans_En()
{
   GPIO_SetBits(GPIOB, GPIO_Pin_3); //PA_Trans_En
   GPIO_ResetBits(GPIOC, GPIO_Pin_5); //LNA_Trans_En
  
}

void LNA_Trans_En()
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_3); //PA_Trans_En
  GPIO_SetBits(GPIOC, GPIO_Pin_5); //LNA_Trans_En
  
}       
volatile char start,start2;

unsigned int arr=700;//658  3.11Hz
unsigned char  communication_num=0;


void GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{  
   if(GPIO_Pin== GPIO_Pin_11)//DIO0
   {
         sx1276_LoRaClearIrq();         
           if(send_over)           
		{   send_over=0;   
		    receive=1;   //允许启动接收                                                                  
               } 
                  
         else if(receive)     
		  {   u8  test;
                    //TIM3->ARR= arr;   //tim3 again initial value                    
                     RSSI = SX1276ReadRssi( MODEM_LORA);
                  start=1;                  
                    receive=0; // 打开发射时恢复 receive=0;                    
                   receive_over=1;
                   communication_num=0;
                   
		   }     
    }
    else if(GPIO_Pin==GPIO_Pin_0)
    {
       TX_RX_Flag=!TX_RX_Flag;FM++,again=0;TX_power++;
    }
}



volatile char wireless_transport_en=0;


void tt(int ENABLE_DISABLE)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  NVIC_InitTypeDef   NVIC_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断11
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE_DISABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
}	    


 
//外部中断初始化程序
//初始化PE2~4,PA0为中断输入.
void EXTIX_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  NVIC_InitTypeDef   NVIC_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);//PB11 连接到中断线11
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);  
	
  /* 配置EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;//LINE11
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE11
  EXTI_Init(&EXTI_InitStructure);//配置
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  
 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断11
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置

  /* Enable the USARTx Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}
  

void PA_LNA_configure(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

}



/*********************************************************************/
#define WM8974_I2Sx_SPI                          SPI3
#define I2Sx_TX_DMA_STREAM                 DMA1_Stream5
#define I2Sx_TX_DMA_CHANNEL                DMA_Channel_0 
#define I2Sx_DMA_CLK                     RCC_AHB1Periph_DMA1
#define I2Sx_TX_DMA_STREAM_IRQn         DMA1_Stream5_IRQn 
void I2S3_TX_DMA_Init(const uint32_t *buffer0,const uint32_t *buffer1,const uint32_t num)  
{  
  NVIC_InitTypeDef   NVIC_InitStructure;  
  DMA_InitTypeDef  DMA_InitStructure;  
  
  RCC_AHB1PeriphClockCmd(I2Sx_DMA_CLK,ENABLE);//DMA1 时钟使能  
  DMA_DeInit(I2Sx_TX_DMA_STREAM); 
  //等待 DMA1_Stream4 可配置 
  while (DMA_GetCmdStatus(I2Sx_TX_DMA_STREAM) != DISABLE) {} ;
  //清空 DMA1_Stream5 上所有中断标志 
  DMA_ClearITPendingBit(I2Sx_TX_DMA_STREAM,     
                        DMA_IT_FEIF5|DMA_IT_DMEIF5|DMA_IT_TEIF5|DMA_IT_HTIF5|DMA_IT_TCIF5);   
 
  /* 配置 DMA Stream */ 
  //通道 0 SPIx_TX 通道 
  DMA_InitStructure.DMA_Channel = I2Sx_TX_DMA_CHANNEL; 
  //外设地址为:(u32)&SPI2->DR
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&WM8974_I2Sx_SPI->DR;   
   //DMA 存储器 0 地址 
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer0; 
   //存储器到外设模式 
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
   //数据传输量 
   DMA_InitStructure.DMA_BufferSize = num; 
   //外设非增量模式 
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
   //存储器增量模式
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
   //外设数据长度:16 位 
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
   //存储器数据长度：16 位 
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
   // 使用循环模式 
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
   //高优先级 
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   //不使用 FIFO 模式 
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; 
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; 
    //外设突发单次传输 
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   //存储器突发单次传输
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
   DMA_Init(I2Sx_TX_DMA_STREAM, &DMA_InitStructure);//初始化 DMA Stream
   //双缓冲模式配置 
   //DMA_DoubleBufferModeConfig(I2Sx_TX_DMA_STREAM,(uint32_t)buffer0, DMA_Memory_0);
   DMA_DoubleBufferModeConfig(I2Sx_TX_DMA_STREAM,(uint32_t)buffer1, DMA_Memory_1); 
   //双缓冲模式开启 
   DMA_DoubleBufferModeCmd(I2Sx_TX_DMA_STREAM,ENABLE); 
   //开启传输完成中断 
    DMA_ITConfig(I2Sx_TX_DMA_STREAM,DMA_IT_TC,ENABLE);
    
   //SPI3 TX DMA 请求使能. 
   SPI_I2S_DMACmd(WM8974_I2Sx_SPI,SPI_I2S_DMAReq_Tx,ENABLE); 
   NVIC_InitStructure.NVIC_IRQChannel = I2Sx_TX_DMA_STREAM_IRQn; 
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
   NVIC_Init(&NVIC_InitStructure);
}

#define WM8974_I2Sx_ext                          I2S3ext  
#define I2Sxext_RX_DMA_STREAM                    DMA1_Stream0
#define I2Sxext_RX_DMA_CHANNEL                   DMA_Channel_3
#define I2Sxext_RX_DMA_STREAM_IRQn               DMA1_Stream0_IRQn     
#define I2Sxext_RX_DMA_IT_TCIF                   DMA_IT_HTIF5

 

 void I2Sxext_RX_DMA_Init(const uint32_t *buffer0,const uint32_t *buffer1, const uint32_t num)  
 {  
   NVIC_InitTypeDef   NVIC_InitStructure;   
    DMA_InitTypeDef  DMA_InitStructure;  
    RCC_AHB1PeriphClockCmd(I2Sx_DMA_CLK,ENABLE);  
    DMA_DeInit(I2Sxext_RX_DMA_STREAM); 
    while (DMA_GetCmdStatus(I2Sxext_RX_DMA_STREAM) != DISABLE) {} ;
    
    DMA_ClearITPendingBit(I2Sxext_RX_DMA_STREAM, 
                          DMA_IT_FEIF3|DMA_IT_DMEIF3|DMA_IT_TEIF3|DMA_IT_HTIF3|DMA_IT_TCIF3);  
    /* 配置 DMA Stream */ 
    DMA_InitStructure.DMA_Channel = I2Sxext_RX_DMA_CHANNEL; 
    DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&WM8974_I2Sx_ext->DR; 
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer0; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; 
    DMA_InitStructure.DMA_BufferSize = num; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord ;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable ; 
    DMA_InitStructure.DMA_FIFOThreshold =  DMA_FIFOThreshold_1QuarterFull; 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_MemoryBurst_Single; 
    DMA_Init(I2Sxext_RX_DMA_STREAM, &DMA_InitStructure); 
   
     //双缓冲模式配置 
    DMA_DoubleBufferModeConfig(I2Sxext_RX_DMA_STREAM, (uint32_t)buffer1,DMA_Memory_1); 
    //双缓冲模式开启 
    DMA_DoubleBufferModeCmd(I2Sxext_RX_DMA_STREAM,ENABLE);
    //开启传输完成中断 
    DMA_ITConfig(I2Sxext_RX_DMA_STREAM,DMA_IT_TC,DISABLE);
    //DMA 请求使能. 
    SPI_I2S_DMACmd(WM8974_I2Sx_ext,SPI_I2S_DMAReq_Rx,ENABLE); 
    
    NVIC_InitStructure.NVIC_IRQChannel = I2Sxext_RX_DMA_STREAM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE; 
    NVIC_Init(&NVIC_InitStructure); 
 }





/*#define I2Sx_TX_DMA_IT_TCIF           DMA_IT_HTIF0

 //I2S DMA 回调函数

void (*I2S_DMA_TX_Callback)(void);

void I2Sx_TX_DMA_STREAM_IRQFUN(void)
{
  //DMA 传输完成标志
  if (DMA_GetITStatus(I2Sx_TX_DMA_STREAM,I2Sx_TX_DMA_IT_TCIF)==SET)
  {
    //清 DMA 传输完成标准

    DMA_ClearITPendingBit(I2Sx_TX_DMA_STREAM,I2Sx_TX_DMA_IT_TCIF);
    //执行回调函数,读取数据等操作在这里面处理
    I2S_DMA_TX_Callback();
  }
}*/

int LED_flag=0;
//定时器3中断服务函数  自动发射
void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
  {
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
   // TIM_Cmd(TIM3,DISABLE); //关闭定时器3
       start2=1;                  
     receive=0;                
      receive_over=1;
    
     
    if(LED_flag==0)
    {
    //   GPIO_SetBits(GPIOB,GPIO_Pin_0); 
       LED_flag=1;
    }
    else
    {    LED_flag=0;
     //  GPIO_ResetBits(GPIOB,GPIO_Pin_0); 
    
    } 
   
  }}

int www;

//DMA 产生中断
void DMA1_Stream5_IRQHandler (void)          //  send  interrupt
{
  
if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5) == SET)
    {    
     //  DMA_ClearFlag(DMA1_Stream5, DMA_IT_TC);      
         DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
      /*  www=!www;  
        if(www==1)
           GPIO_SetBits(GPIOB, GPIO_Pin_0);
        else if(www==0)
           GPIO_ResetBits(GPIOB, GPIO_Pin_0);
         */  
        //TIM_Cmd(TIM3,ENABLE); //使能定时器3      
     buffer_select1=!buffer_select1; 
      en_code=1;
         buffer_select0=!buffer_select0; 
      if( buffer_select1==1)
         { for(int i=0;i<1280;i++)
            {                        
             // buffer3[i]=0;
              buffer2[i]=buffer_decode_0[i];  
               buffer_encode_0[i]=buffer0[i];
            }           
         }
         else   if( buffer_select1==0)     
         {  for(int i=0;i<1280;i++)
             {  
             //  buffer2[i]=0; 
               buffer3[i]=buffer_decode_1[i]; 
                buffer_encode_1[i]=buffer1[i]; 
             }
         }
 
    }
}

void DMA1_Stream0_IRQHandler (void)   //receive  interrupt
{
  
/*if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF5) == SET)
    {    
           
         DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF5);
      
       en_code=1;
      buffer_select0=!buffer_select0; 
      if( buffer_select0==1)
         { for(int i=0;i<1280;i++)
            {                      
              buffer_encode_0[i]=buffer0[i];
            }           
         }
         else   if( buffer_select0==0)      
         {  for(int i=0;i<1280;i++)
             {
               buffer_encode_1[i]=buffer1[i]; 
             }
         }
 
    }*/
}


void NVIC_Config()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannel  = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    NVIC_Init(&NVIC_InitStructure);
}

void USART_Gpio_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA  , ENABLE);
    
    //PA2->TX  PA3->Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
}

void  usart_configure(void)
{
    NVIC_Config();
    USART_Gpio_Config();
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    USART_InitTypeDef USART_InitStructure;
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART2,&USART_InitStructure);   
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
    
    USART_Cmd(USART2,ENABLE);
  
}

char uart_trans_En=0;



/***************************************************/

char test=0;

 

//I2S2 初始化 //参数 I2S_Standard: @ref SPI_I2S_Standard  I2S 标准, //参数 I2S_Mode: @ref SPI_I2S_Mode //参数 I2S_Clock_Polarity    @ref SPI_I2S_Clock_Polarity: //参数 I2S_DataFormat：@ref SPI_I2S_Data_Format : 
/*void I2S2_Init(u16 I2S_Standard,u16 I2S_Mode,u16 I2S_Clock_Polarity,u16 I2S_DataFormat) 
{      I2S_InitTypeDef I2S_InitStructure;  
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能 SPI2 时钟  
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE); //复位 SPI2 
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//结束复位     
      I2S_InitStructure.I2S_Mode=I2S_Mode;//IIS 模式  
      I2S_InitStructure.I2S_Standard=I2S_Standard;//IIS 标准 
      I2S_InitStructure.I2S_DataFormat=I2S_DataFormat;//IIS 数据长度 
      I2S_InitStructure.I2S_MCLKOutput=I2S_MCLKOutput_Disable;//主时钟输出禁止  
      I2S_InitStructure.I2S_AudioFreq=I2S_AudioFreq_Default;//IIS 频率设置 
      I2S_InitStructure.I2S_CPOL=I2S_Clock_Polarity;//空闲状态时钟电平  
      I2S_Init(SPI2,&I2S_InitStructure);//初始化 IIS 
      SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);//SPI2 TX DMA 请求使能.     
      I2S_Cmd(SPI2,ENABLE);//SPI2 I2S EN 使能. 
} */
   
/*u8 I2S2_SampleRate_Set(u32 samplerate) 
{   u8 i=0; 
    u32 tempreg=0;
    samplerate/=10;//缩小 10 倍    
    for(i=0;i<(sizeof(I2S_PSC_TBL)/10);i++)//看看改采样率是否可以支持 
    {   if(samplerate==I2S_PSC_TBL[i][0])break;  } 
    RCC_PLLI2SCmd(DISABLE);//先关闭 PLLI2S  
    if(i==(sizeof(I2S_PSC_TBL)/10))
      return 1;//搜遍了也找不到  
    RCC_PLLI2SConfig((u32)I2S_PSC_TBL[i][1],(u32)I2S_PSC_TBL[i][2]);     //设置 I2SxCLK 的频率(x=2)  设置 PLLI2SN PLLI2SR 
    RCC->CR|=1<<26;     //开启 I2S 时钟  
    while((RCC->CR&1<<27)==0);  //等待 I2S 时钟开启成功.   
    tempreg=I2S_PSC_TBL[i][3]<<0; //设置 I2SDIV 
    tempreg|=I2S_PSC_TBL[i][3]<<8; //设置 ODD 位  
    tempreg|=1<<9;     //使能 MCKOE 位,输出 MCK  
    SPI2->I2SPR=tempreg;   //设置 I2SPR 寄存器  
    return 0; 
} */

void TX()
{ 
   PA_En();        
  PA_Trans_En();
  wireless_transport_en=0;             	           
  sx1276_LoRaEntryTx();  //单向02ELS接收时注释掉
  sx1276_LoRaTxPacket();
  send_over=1;
  receive_over=0;
}

void I2C_write(char addr,char data1,char data0)
{
       I2C_GenerateSTART(I2C3, ENABLE);
   while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT)); 
  I2C_Send7bitAddress(I2C3,0xE2, I2C_Direction_Transmitter);//
    while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  
      I2C_SendData(I2C3, addr);                       
    while(!I2C_CheckEvent(I2C3,  I2C_EVENT_MASTER_BYTE_TRANSMITTED )); 
    I2C_SendData(I2C3, data1);   
    while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(I2C3, data0);     
    while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void AT1846S()
{   unsigned int test1846_0,test1846_1;  
  while(1)
 {  
    if(again==0)
  {  again=1;
    
    while(I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY)); 
    
   { I2C_write(0x30,0x00,0x01);
    delay_ms(500);
    I2C_write(0x30,0x00,0x04);
    I2C_write(0x04,0x0F,0xD1);
    I2C_write(0x31,0x00,0x31);
    I2C_write(0x33,0x44,0xA5);
    I2C_write(0x34,0x2B,0x87);
    I2C_write(0x41,0x47,0x0F);
    I2C_write(0x44,0x0D,0xFF);
    I2C_write(0x47,0x7F,0xFF);
    I2C_write(0x4F,0x2C,0x62);
    I2C_write(0x53,0x00,0x94);
    I2C_write(0x55,0x00,0x81);
    I2C_write(0x56,0x0B,0x22);
    I2C_write(0x57,0x1C,0x00);
    I2C_write(0x5A,0x0E,0xDB);
    I2C_write(0x60,0x10,0x1E);
    I2C_write(0x63,0x16,0xAD);
    I2C_write(0x30,0x40,0xA4);
    delay_ms(500);
    I2C_write(0x30,0x40,0xA6);
    delay_ms(500);
    I2C_write(0x30,0x40,0x06);
    }
    
     
     /*********校准***************/
     I2C_write(0x27,0x74,0xA4);
      delay_ms(10);
     I2C_write(0x27,0x7C,0xA4);
     I2C_write(0x27,0x6C,0xA4);
       
   
   /********窄带设置**********/  
    
    /*    I2C_write(0x15,0x11,0x00);
        I2C_write(0x32,0x44,0x95);
         I2C_write(0x3A,0x40,0xC3);
          I2C_write(0x3C,0x04,0x07);
          I2C_write(0x3F,0x28,0xD0);
           I2C_write(0x48,0x20,0x3E);
            I2C_write(0x60,0x1B,0xB7);
             I2C_write(0x62,0x14,0x25);
              I2C_write(0x65,0x24,0x94);
               I2C_write(0x66,0xEB,0x2E);
      I2C_write(0x7F,0x00,0x01);
       I2C_write(0x06,0x00,0x14);
        I2C_write(0x07,0x02,0x0C);
         I2C_write(0x08,0x02,0x14);
          I2C_write(0x09,0x03,0x0C);
           I2C_write(0x0A,0x03,0x14);
            I2C_write(0x0B,0x03,0x24);
             I2C_write(0x0C,0x03,0x44);
              I2C_write(0x0D,0x13,0x44);
               I2C_write(0x0E,0x1B,0x44);
                I2C_write(0x0F,0x3F,0x44);
                 I2C_write(0x12,0xE0,0xEB);
                  I2C_write(0x7F,0x00,0x00);*/
     
      /********宽带设置**********/  
    
        I2C_write(0x15,0x1F,0x00);
        I2C_write(0x32,0x75,0x64);
         I2C_write(0x3A,0x44,0xC3);
          I2C_write(0x3C,0x17,0x2C);
          I2C_write(0x3F,0x29,0xD2);
           I2C_write(0x48,0x21,0x41);
           I2C_write(0x59,0x0A,0x50);
             I2C_write(0x62,0x37,0x67);
              I2C_write(0x65,0x24,0x8A);
               I2C_write(0x66,0xFF,0x2E);
      I2C_write(0x7F,0x00,0x01);
       I2C_write(0x06,0x00,0x24);
        I2C_write(0x07,0x02,0x14);
         I2C_write(0x08,0x02,0x24);
          I2C_write(0x09,0x03,0x14);
           I2C_write(0x0A,0x03,0x24);
            I2C_write(0x0B,0x03,0x44);
             I2C_write(0x0C,0x03,0x84);
              I2C_write(0x0D,0x13,0x84);
               I2C_write(0x0E,0x1B,0x84);
                I2C_write(0x0F,0x3F,0x84);
                 I2C_write(0x12,0xE0,0xEB);
                  I2C_write(0x7F,0x00,0x00);
   
      if(TX_RX_Flag==0)     
      {  
          PaOff();
         LNA_En();        
         LNA_Trans_En();
        
        I2C_write(0x30,0x00,0x00); //shut down TX or Rx     
        I2C_write(0x05,0x87,0x63);
        I2C_write(0x29,0x00,0x69);
       I2C_write(0x2A,0x1A,0x40);//430.5M
   //  I2C_write(0x41,0x47,0x1F);
      /*********接收**********需设置WM8974 DEC50=0x02***/
        I2C_write(0x49,0x0E,0xEF);     //sq level
        I2C_write(0x30,0x70,0x06);   //RX  
         I2C_write(0x30,0x70,0x2E);   //RX      I2C_write(0x30,0x70,0x2E); //SQ       
         I2C_write(0x44,0x0D,0xAA);//volume
         I2C_write(0x58,0xFF,0x40);  //filter
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
      }
      
     else if(TX_RX_Flag==1)
       {  
          
         
          PA_En();        
          PA_Trans_En();
         I2C_write(0x30,0x00,0x00); //shut down TX or Rx     
        I2C_write(0x05,0x87,0x63);
        I2C_write(0x29,0x00,0x69);
       I2C_write(0x2A,0x1A,0xB0);//0x2A,0x1A,0xA0 430.501M
   //  I2C_write(0x41,0x47,0x1F);
     
    /*********发射******/ 
      I2C_write(0x0A,0x7B,0x7F);//  0x7B=7dBm 3.3V  
      I2C_write(0x30,0x70,0x06);   //TX
       I2C_write(0x30,0x70,0x46);   //TX
          GPIO_ResetBits(GPIOB, GPIO_Pin_0);
      }
    
 /************************读取寄存器值****************************/ 
 /*   I2C_GenerateSTOP(I2C3, ENABLE);
     while(I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY)); 
     I2C_GenerateSTART(I2C3, ENABLE);
     while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT)); 
  I2C_Send7bitAddress(I2C3,0xE2, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )); 
     I2C_SendData(I2C3, 0x3A);        
    while(!I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_BYTE_TRANSMITTED ));
      I2C_GenerateSTART(I2C3, ENABLE);
       while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));
        I2C_Send7bitAddress(I2C3, 0xE3, I2C_Direction_Receiver);
           while(!(I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED)));
             test1846_0 = I2C_ReceiveData(I2C3);
             while(!(I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED)));
             test1846_1 = I2C_ReceiveData(I2C3);
    */  
    
     I2C_GenerateSTOP(I2C3, ENABLE);
     // I2C_AcknowledgeConfig(I2C1,DISABLE);

  }
 }
}

float SX1276ReadRssi(int modem)
{
	float rssi; u8 temp;
        float  PacketRssi=113;

	switch( modem )
	{
	case MODEM_FSK:
		rssi = -(SX1276Read( LR_RegPktRssiValue,&temp ) >> 1 );
		break;
	case MODEM_LORA:		
		{
		  rssi = -164 + SX1276Read( LR_RegPktRssiValue ,&temp);
		}
		break;
	default:
		rssi = -1;
		break;
	}
       // rssi = -164+16/15 * PacketRssi;        
	return rssi;
}


int main(int argc, char *argv[]) {

         
	 int i;
     //    usart_configure();
      //   while(1);

	PA_LNA_configure();
        EXTIX_Init(); 
        LED_Init();
        I2C3_Congiguration();
        delay_ms(2000);
        I2S_GPIO_Config();
		  // TIM3_Int_Init(51,200) ;  //8KHz 		    
		   TIM8_PWM_Init(5,13,3); 
        SPI1_GPIO_Init();   
       // TX();
      // TIM3_Int_Init(2000,20000); //(658,20000)
        
    
     /* I2S3_Init(I2S_Standard_Phillips,I2S_Mode_MasterTx,I2S_CPOL_Low, I2S_DataFormat_16b);
        I2S2ext_Init(I2S_Standard_Phillips,I2S_Mode_SlaveRx,I2S_CPOL_Low, I2S_DataFormat_16b);//飞利浦标准,从机接收,时钟低电平有效,16 位帧长度  
        I2S3_SampleRate_Set(8000); //设置采样率  */
       /**********I2S 主模式 DMA中断为啥加倍？**************/ 
        
 /*xx:   DMA_Cmd(I2Sxext_RX_DMA_STREAM,DISABLE);//关闭DMA RX传输,关闭录音  
           DMA_Cmd(I2Sx_TX_DMA_STREAM,DISABLE);//关闭DMA TX传输,关闭播放   
           delay_ms(10);
        
     I2S3_Mode_Config(I2S_Standard_Phillips  ,I2S_DataFormat_16b,I2S_AudioFreq_8k); 
    I2S3ext_Mode_Config(I2S_Standard_Phillips ,I2S_DataFormat_16b,I2S_AudioFreq_8k);        
     I2Sxext_RX_DMA_Init(buffer0,buffer1,2560) ;
    I2S3_TX_DMA_Init(buffer1,buffer0,2560)  ;
     DMA_Cmd(I2Sxext_RX_DMA_STREAM,ENABLE);//开启DMA RX传输,开始录音  
     DMA_Cmd(I2Sx_TX_DMA_STREAM,ENABLE);//开启DMA TX传输,开始播放  
     delay_ms(10);
     
    //  DMA_Cmd(I2Sxext_RX_DMA_STREAM,DISABLE);//开启DMA RX传输,开始录音  
    // DMA_Cmd(I2Sx_TX_DMA_STREAM,DISABLE);//开启DMA TX传输,开始播放  
    //  delay_ms(10);
      goto xx ; */
     
     
      /* LNA_En();        
       LNA_Trans_En();
      sx1276_LoRaEntryRx();  //配置成接收*/
   
        WM8974();
     if(FM==1)
     { RF_switch1846();
       AT1846S();
     }
      else  
      
      {   RF_switch1278();
         sx1276_Config();  
         SX1276Read(0x42,&test);//test=0x12  
         SX1276Read(0x44,&test);//test=0x2D
         

	machdep_profile_init ();
        Voice_Realtime_Transfer();
      }
 
}

#pragma optimize=speed  medium
void Voice_Realtime_Transfer(void)
{
   struct CODEC2 *codec2;
   unsigned char   *bits;
int   nsam, nbit;
codec2 = codec2_create(CODEC2_MODE_1200);
 nsam = codec2_samples_per_frame(codec2);
nbit = codec2_bits_per_frame(codec2);
bits = (unsigned char*)malloc(nbit*sizeof(char));

    static unsigned char char_wireless_send,char_wireless_receive;
      
 wireless_transport_en=1;
 char wireless_send_pointer,wireless_receive_pointer;
 int xwl_point;
 int xwl_delay;
 
    start=0;     
TX_RX_Flag=TX_start=0,RX_start=1;
 while(1)
  {   
     if(TX_RX_Flag==1)
     {   
  TX:  if( TX_start==1)
       {  
           DMA_Cmd(I2Sxext_RX_DMA_STREAM,DISABLE);//关闭DMA RX传输,关闭录音  
           DMA_Cmd(I2Sx_TX_DMA_STREAM,DISABLE);//关闭DMA TX传输,关闭播放   
           delay_ms(10);        
           I2S3_Mode_Config(I2S_Standard_Phillips  ,I2S_DataFormat_16b,I2S_AudioFreq_8k); 
           I2S3ext_Mode_Config(I2S_Standard_Phillips ,I2S_DataFormat_16b,I2S_AudioFreq_8k);        
           I2Sxext_RX_DMA_Init(buffer0,buffer1,2560) ;
           I2S3_TX_DMA_Init(buffer3,buffer2,2560)  ;
           DMA_Cmd(I2Sxext_RX_DMA_STREAM,ENABLE);//开启DMA RX传输,开始录音  
           DMA_Cmd(I2Sx_TX_DMA_STREAM,ENABLE);//开启DMA TX传输,开始播放          
           TX_start=0;
        }
    
          {           
             PA_En();        
             PA_Trans_En();
             wireless_transport_en=0;             	           
             sx1276_LoRaEntryTx();  //单向02ELS接收时注释掉
             sx1276_LoRaTxPacket();
             send_over=1;
             receive_over=0;
            }
       
       while(en_code==0);
             en_code =0; 
           xwl_point=0;
          
          
        if( buffer_select0==0)      
        {    
            GPIO_SetBits(GPIOB, GPIO_Pin_0);
            for(int AAA=0;AAA<4;AAA++) 
          { 
               codec2_encode(codec2, bits, &buffer_encode_1[xwl_point]);
            for(char i=0;i<6;i++)
              {   wireless_send[i+wireless_send_pointer]=*(bits+i);
                  
              } 
             wireless_send_pointer+=6;   
            xwl_point+=320;
           }  
          }
          
        
         else  if( buffer_select0==1)      
         {      GPIO_ResetBits(GPIOB, GPIO_Pin_0);
           for(int AAA=0;AAA<4;AAA++) 
          {
              codec2_encode(codec2, bits, &buffer_encode_0[xwl_point]); 
              for(char i=0;i<6;i++)
               { 
                   wireless_send[i+wireless_send_pointer]=*(bits+i);  
               } 
               wireless_send_pointer+=6;
                xwl_point+=320;
            }            
          }   
        
          //  if( wireless_send_pointer>=loadLength)//24个字节打包
              {  
                           
                  wireless_send_pointer=0;                 
              }
      }   
     
      else if(TX_RX_Flag==0)  //receive
      { 
      if(RX_start==1)    
      {   PaOff();  
          LNA_En();        
           LNA_Trans_En();
           sx1276_LoRaEntryRx();  //配置成接收*/    
      }
             xwl_point=0;
              send_over=0;   
	       receive=1;   //允许启动接收
               
             while(start==0){
               if(TX_RX_Flag==1){
                 TX_start=1; goto TX ;}  }               
              start=0;
        
         if(RX_start==1)     
         {
           DMA_Cmd(I2Sxext_RX_DMA_STREAM,DISABLE);//关闭DMA RX传输,关闭录音  
           DMA_Cmd(I2Sx_TX_DMA_STREAM,DISABLE);//关闭DMA TX传输,关闭播放   
           delay_ms(10);        
           I2S3_Mode_Config(I2S_Standard_Phillips  ,I2S_DataFormat_16b,I2S_AudioFreq_8k); 
           I2S3ext_Mode_Config(I2S_Standard_Phillips ,I2S_DataFormat_16b,I2S_AudioFreq_8k);        
           I2Sxext_RX_DMA_Init(buffer0,buffer1,2560) ;
           I2S3_TX_DMA_Init(buffer3,buffer2,2560)  ;
           DMA_Cmd(I2Sxext_RX_DMA_STREAM,ENABLE);//开启DMA RX传输,开始录音  
           DMA_Cmd(I2Sx_TX_DMA_STREAM,ENABLE);//开启DMA TX传输,开始播放      
           RX_start=0;
          }   
              
           
           { 
               SX1276ReadBuffer( 0x00,wireless_receive, loadLength);             
             
            }
             
        if( buffer_select1==0)      
        {    
            GPIO_SetBits(GPIOB, GPIO_Pin_0);
            for(int AAA=0;AAA<4;AAA++) 
          { 
            for(char i=0;i<6;i++)
              {   
                  *(bits+i)=wireless_receive[i+wireless_send_pointer];
                // wireless_send[i+wireless_send_pointer]=wireless_receive[i+wireless_send_pointer];
              }    
             // codec2_encode(codec2, bits, &buffer_encode_0[xwl_point]);             
               codec2_decode(codec2, &buffer_decode_1[xwl_point], bits);   
              wireless_send_pointer+=6;   
                xwl_point+=320;
          }
          
        }
         else  if( buffer_select1==1)      
         {      GPIO_ResetBits(GPIOB, GPIO_Pin_0);
           for(int AAA=0;AAA<4;AAA++) 
          {
              for(char i=0;i<6;i++)
               {                  
                   *(bits+i)=wireless_receive[i+wireless_send_pointer]; 
                 // wireless_send[i+wireless_send_pointer]=wireless_receive[i+wireless_send_pointer];
               } 
              codec2_decode(codec2, &buffer_decode_0[xwl_point], bits);  
              wireless_send_pointer+=6;
                xwl_point+=320;
                       
            }   
         }          
                  wireless_send_pointer=0;   
      }
    }      
  }
