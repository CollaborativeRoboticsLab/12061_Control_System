#include "sys.h"
#include "usart_cmd.h"   /* ==== 改动 1/3 (Minibalance.c):串口命令层 ==== */
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Flag_Stop=1,delay_50,delay_flag;         //停止标志位 50ms精准演示标志位
u8 system_start=0;                          //检测调节函数标志位
u8 tips_flag = 0;                           //OLED提示函数标志位
int Encoder,Position_Zero=10000;            //编码器的脉冲计数
int Moto;                                   //电机PWM变量 应是Motor的 向Moto致敬	
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance;                        //角位移传感器数据
float Balance_KP=400,Balance_KD=400,Position_KP=20,Position_KD=300;  //PID系数
/* R4-5: the angle loop was pure PD. Lab 4 asks students to add integral
   action, so Ki exists now and defaults to 0 -- i.e. the rig behaves
   exactly as before until someone sends GAIN,BKI,<value>. */
float Balance_KI=0;
float Balance_Integral=0;
//float Balance_KP=10,Balance_KD=0,Position_KP=0,Position_KD=0;  //PID系数
float Menu=1,Amplitude1=5,Amplitude2=20,Amplitude3=1,Amplitude4=10; //PID调试相关参数                                               
extern float D_Angle_Balance; //摆杆角度变化率
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====系统时钟设置
	delay_init(72);                 //=====延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	delay_ms(1000);                 //=====延时启动，等待系统稳定
	delay_ms(1000);                 //=====延时启动，等待系统稳定 共2s
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	EXTI_Init();                    //=====按键初始化(外部中断的形式)
	OLED_Init();                    //=====OLED初始化
	uart_init(72,128000);           //=====初始化串口1
	// uart_init(72,112000);           //=====初始化串口1
	UartCmd_Init();                 //=====改动 2/3:打开串口接收中断(必须在 uart_init 之后)
  MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机 
	Encoder_Init_TIM4();            //=====初始化编码器（TIM2的编码器接口模式） 
	Adc_Init();                     //=====角位移传感器模拟量采集初始化
	Timer1_Init(49,7199);           //=====定时中断初始化 
	
	

//	OLED_ShowString(0,0,"UART TEST");
//	OLED_Refresh_Gram();
	
	
//	while(1)
//	{
////		DataScope();
//		Tips();
//		// USART1 ??,???????? A
//    while((USART1->SR & 0x40) == 0);
//    USART1->DR = 'A';
//		
//		delay_flag = 1;
//		while(delay_flag);
//	}
//	}
	/* ==== 改动 3/3 (Minibalance.c) ====
	   两处改动:
	   1) DataScope() 加了开关。开机时 stream_enable=1,行为和原来完全一样;
	      MATLAB 一发命令就自动关掉,免得二进制波形和文字回复混在一起。
	   2) 原来那 50ms 是干等,现在顺便处理串口命令。
	      放在这里而不是中断里,是因为发回复要等硬件标志位,
	      在 5ms 的控制中断里等会把控制环拖死。
	   ================================== */
	while(1)
		{      
				if(stream_enable) DataScope();   //===上位机(收到第一条 ASCII 命令后自动关闭)
		   	Tips();                          //===OLED显示与提示
				delay_flag=1;	                   //===50ms中断精准延时标志位
				while(delay_flag) UartCmd_Poll();//===等 50ms 的同时处理串口命令
		} 
}
