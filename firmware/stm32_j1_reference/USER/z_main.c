/***************************************************************
	*	@笔者	：	tacbo
	*	@日期	：	2020年05月23日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-robot.com
	*	@功能	：	ZL-KPZ51控制板
	
	实现的功能：
	1、手柄按钮控制0-5号舵机，摇杆控制6-7号电机；
	2、zide图形化控制舵机
	3、可脱机存储控制
	
	传感器引脚:
		循迹（S1-PA0 PA1） 
		超声波(S3-PB0 PA2) 
		声音(S4-PB1) 
		颜色识别(S6-PA7 PA5)
	舵机引脚：
		DJ0-PB3
		DJ1-PB8
		DJ2-PB9
		DJ3-PB6
		DJ4-PB7
		DJ5-PB4
	蜂鸣器引脚：
		BEEP-PB5
	LED引脚：
		NLED-PB13
  PS2手柄引脚：	
	  PS1-DAT-PA15
	  PS2-CMD-PA14
	  PS6-ATT-PA13
	  PS7-CLK-PA12
	按键引脚：
	  KEY1-PA8 KEY2-PA11
	
	统一总线口： TX3 RX3
	
	主频：72M
	单片机型号：STM32F103C8T6
	
***************************************************************/

#include "z_rcc.h"		//配置时钟文件
#include "z_gpio.h"		//配置IO口文件
#include "z_global.h"	//存放全局变量
#include "z_delay.h"	//存放延时函数
#include "z_type.h"		//存放类型定义
#include "z_usart.h"	//存放串口功能文件
#include "z_timer.h"	//存放定时器功能文件
#include "z_ps2.h"		//存放索尼手柄
#include "z_w25q64.h"	//存储芯片的操作
#include "z_adc.h"		//ADC初始化
#include <stdio.h>		//标准库文件
#include <string.h>		//标准库文件
#include <math.h>		//标准库文件
#include "z_kinematics.h"	//逆运动学算法
#include "z_action.h" //动作组执行文件
#include "stm32f10x_iwdg.h"


#define MODULE "Jibot1-32"

/*
	全局变量定义
*/
//int i;								    //常用的一个临时变量
u8 ps_mode = 0;
u8 psx_buf[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //存储手柄的数据 	

kinematics_t kinematics;

const char *pre_cmd_set_red[PSX_BUTTON_NUM] = {//红灯模式下按键的配置			
	"<PS2_RED01:#005P0600T2000!^#005PDST!>",	//L2						  
	"<PS2_RED02:#005P2400T2000!^#005PDST!>",	//R2						  
	"<PS2_RED03:#004P0600T2000!^#004PDST!>",	//L1						  
	"<PS2_RED04:#004P2400T2000!^#004PDST!>",	//R1			
	"<PS2_RED05:#002P2400T2000!^#002PDST!>",	//RU						  
	"<PS2_RED06:#003P2400T2000!^#003PDST!>",	//RR						  
	"<PS2_RED07:#002P0600T2000!^#002PDST!>",	//RD						  
	"<PS2_RED08:#003P0600T2000!^#003PDST!>",	//RL				
	"<PS2_RED09:$MODE!>",					            //SE    				  
	"<PS2_RED10:>",					                  //AL						   
	"<PS2_RED11:>",					                  //AR						  
	"<PS2_RED12:$DJR!>",					            //ST		
	"<PS2_RED13:#001P0600T2000!^#001PDST!>",	//LU						  
	"<PS2_RED14:#000P0600T2000!^#000PDST!>",	//LR								  
	"<PS2_RED15:#001P2400T2000!^#001PDST!>",	//LD						  
	"<PS2_RED16:#000P2400T2000!^#000PDST!>",	//LL								
};

const char *ai_pre_cmd_set_red[PSX_BUTTON_NUM] = {//红灯模式下按键的配置			
	"<PS2_RED01:#005P0600T2000!^#005PDST!>",	//L2						  
	"<PS2_RED02:#005P2400T2000!^#005PDST!>",	//R2						  
	"<PS2_RED03:#004P0600T2000!^#004PDST!>",	//L1						  
	"<PS2_RED04:#004P2400T2000!^#004PDST!>",	//R1			
	"<PS2_RED05:$DGT:18-21,1!>",	            //RU 前放	 					  
	"<PS2_RED06:$DGT:27-31,1!>",	            //RR 右放					  
	"<PS2_RED07:$DGT:1-1,1!>",	              //RD 蜷缩					  
	"<PS2_RED08:$DGT:22-26,1!>",	            //RL 左放			
	"<PS2_RED09:$MODE!>",					            //SE    				  
	"<PS2_RED10:>",					                  //AL						   
	"<PS2_RED11:>",					                  //AR						  
	"<PS2_RED12:$DJR!>",					            //ST		
	"<PS2_RED13:$DGT:3-7,1!>",	              //LU	前抓					  
	"<PS2_RED14:$DGT:13-17,1!>",	            //LR	右抓						  
	"<PS2_RED15:$DGT:1-1,1!>",	              //LD	蜷缩					  
	"<PS2_RED16:$DGT:8-12,1!>",	              //LL	左抓						
};

//绿灯模式下，摇杆会映射到按键，防止影响，去掉绿灯模式功能
/*
const char *pre_cmd_set_grn[PSX_BUTTON_NUM] = {//绿灯模式下按键的配置			
	"<PS2_GRN01:#005P0600T2000!^#005PDST!>",	 //L2						  
	"<PS2_GRN02:#005P2400T2000!^#005PDST!>",	 //R2						  
	"<PS2_GRN03:#004P0600T2000!^#004PDST!>",	 //L1						  
	"<PS2_GRN04:#004P2400T2000!^#004PDST!>",	 //R1			
	"<PS2_GRN05:#002P2400T2000!^#002PDST!>",	 //RU						  
	"<PS2_GRN06:#003P2400T2000!^#003PDST!>",	 //RR						  
	"<PS2_GRN07:#002P0600T2000!^#002PDST!>",	 //RD						  
	"<PS2_GRN08:#003P0600T2000!^#003PDST!>",	 //RL				
	"<PS2_GRN09:$DJR!>",					             //SE    				  
	"<PS2_GRN10:>",					                   //AL						   
	"<PS2_GRN11:>",					                   //AR						  
	"<PS2_GRN12:$DJR!>",					             //ST		
	"<PS2_GRN13:#001P0600T2000!^#001PDST!>",	 //LU						  
	"<PS2_GRN14:#000P0600T2000!^#000PDST!>",	 //LR								  
	"<PS2_GRN15:#001P2400T2000!^#001PDST!>",	 //LD						  
	"<PS2_GRN16:#000P2400T2000!^#000PDST!>",	 //LL								
};
*/

/*-------------------------------------------------------------------------------------------------------
*  程序从这里执行				
*  这个启动代码 完成时钟配置 使用外部晶振作为STM32的运行时钟 并倍频到72M
-------------------------------------------------------------------------------------------------------*/

int main(void) {	
	setup_rcc();		  //初始化时钟
	setup_global();		//初始化全局变量
	setup_gpio();		  //初始化IO口
	setup_nled();		  //初始化工作指示灯
	setup_beep();		  //初始化定时器
	setup_djio();		  //初始化舵机IO口
	setup_w25q64();		//初始化存储器W25Q64
	setup_ps2();		  //初始化PS2手柄
	setup_uart1();		//初始化串口1 用于下载动作组
	setup_uart3();		//初始化串口3 用于底板总线、蓝牙、lora
	setup_systick();	//初始化滴答时钟，1S增加一次millis()的值
	setup_dj_timer();	//初始化定时器2 处理舵机PWM输出	
	setup_interrupt();//初始化总中断		
	setup_kinematics(110, 105, 75, 190, &kinematics); //kinematics 90mm 105mm 98mm 150mm
	setup_servo_bias();  //初始化舵机，将偏差代入初始值
	IWDG_Init();       //初始化独立看门狗
	setup_start();		//初始化启动信号
	setup_do_group(); //开机动作
	
	while(1) {
		loop_nled();		//循环执行工作指示灯，500ms跳动一次 和声音公用一个IO口 这里在声音功能启用的时候就关闭nled
		loop_uart();		//串口数据接收处理
		loop_action();	//动作组批量执行
		loop_ps2_data();//循环读取PS2手柄数据
		loop_ps2_button();//处理手柄上的按钮
		//loop_monitor();   //定时保存一些变量
		loop_pwm_monitor();	//定时去观测PWM舵机的状态
				
	}
}

//--------------------------------------------------------------------------------
/*
	初始化函数实现
*/

void setup_rcc(void) {   //初始化时钟
	tb_rcc_init();	  	   //时钟初始化
}

void setup_global(void) {//初始化全局变量
	tb_global_init();	
}

void setup_gpio(void) {  //初始化IO口
	tb_gpio_init();		    
}

void setup_nled(void) {  //初始化工作指示灯
	nled_init();		
	nled_off();		         //工作指示灯关闭
}

void setup_beep(void) {  //初始化定时器蜂鸣器
	beep_init();		
	beep_off();			       //关闭蜂鸣器
}			
void setup_w25q64(void) {//初始化存储器W25Q64
	u8 i;
	spiFlahsOn(1);
	w25x_init();				   //动作组存储芯片初始化
	w25x_read((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info));//读取全局变量
	
	if(eeprom_info.version != VERSION) {//判断版本是否是当前版本
		eeprom_info.version = VERSION;		//复制当前版本
		eeprom_info.dj_record_num = 0;		//学习动作组变量赋值0
	}
	
	if(eeprom_info.dj_bias_pwm[DJ_NUM] != FLAG_VERIFY) {
		for(i=0;i<DJ_NUM;i++) {
			eeprom_info.dj_bias_pwm[i] = 0;
		}
		eeprom_info.dj_bias_pwm[DJ_NUM] = FLAG_VERIFY;
	}
	
	for(i=0;i<DJ_NUM;i++) {
		duoji_doing[i].aim = 1500 + eeprom_info.dj_bias_pwm[i];
		duoji_doing[i].cur = 1500 + eeprom_info.dj_bias_pwm[i];
		duoji_doing[i].inc = 0;
	}
	spiFlahsOn(0);
}	

void setup_adc(void) {//初始化ADC采集 使用DMA初始化
	ADC_init();
}

void setup_ps2(void) {//初始化PS2手柄
	PSX_init();	
}

void setup_uart1(void) {
  //串口1初始化
	tb_usart1_init(115200);
	//串口1打开
	uart1_open();
	//串口发送测试字符
	uart1_send_str((u8 *)"uart1 check ok!");
}
//初始化串口2
void setup_uart2(void) {
	//串口2初始化
	tb_usart2_init(115200);
	//串口2打开
	uart2_open();
	//串口发送测试字符
	uart2_send_str((u8 *)"uart2 check ok!");
}	
//初始化串口3
void setup_uart3(void) {
	//串口3初始化
	tb_usart3_init(115200);
	//串口3打开
	uart3_open();
	//串口发送测试字符
	uart3_send_str((u8 *)"uart3 check ok!");
	//总线输出 复位总线舵机 串口3即为总线串口
	zx_uart_send_str((u8 *)"#255P1500T2000!");
}	
//初始化滴答时钟，1S增加一次millis()的值
void setup_systick(void) {
	//系统滴答时钟初始化	
	SysTick_Int_Init();
}	
//初始化启动信号
void setup_start(void) {
	//蜂鸣器LED 名叫闪烁 示意系统启动
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
}	


//初始化总中断
void setup_interrupt(void) {
	//总中断打开
	tb_interrupt_open();
}	
//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------
/*
	主循环函数实现
*/
//循环执行工作指示灯，500ms跳动一次
void loop_nled(void) {
	static u32 time_count=0;
	static u8 flag = 0;
	if(millis()-time_count > 1000)  {
		time_count = millis();
		if(flag) {
			nled_on();
		} else {
			nled_off();
		}
		flag= ~flag;
	}
}		
//串口数据接收处理
void loop_uart(void) {
	if(uart1_get_ok) {
		if(uart1_mode == 1) {					    //命令模式
			parse_group_cmd(uart_receive_buf);
			parse_cmd(uart_receive_buf);			
		} else if(uart1_mode == 2) {			//单个舵机调试
			parse_action(uart_receive_buf);
		} else if(uart1_mode == 3) {		  //多路舵机调试
			parse_action(uart_receive_buf);
		} else if(uart1_mode == 4) {		  //存储模式
			save_action(uart_receive_buf);
		} 
		uart1_mode = 0;
		uart1_get_ok = 0;
		uart1_open();
	}
	return;
}	

//循环读取PS2手柄数据
void loop_ps2_data(void) {
	static u32 systick_ms_bak = 0;
	//每50ms处理1次
	if(millis() - systick_ms_bak < 50) {
		return;
	}
	systick_ms_bak = millis();
	//读写手柄数据
	psx_write_read(psx_buf);
	
#if 0
	//测试手柄数据，1为打开 0为关闭
	sprintf((char *)cmd_return, "0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\r\n", 
	(int)psx_buf[0], (int)psx_buf[1], (int)psx_buf[2], (int)psx_buf[3],
	(int)psx_buf[4], (int)psx_buf[5], (int)psx_buf[6], (int)psx_buf[7], (int)psx_buf[8]);
	uart1_send_str(cmd_return);
#endif 	
	
	return;
}	
//处理手柄上的按钮
void loop_ps2_button(void) {
	static unsigned char psx_button_bak[2] = {0};

	//对比两次获取的按键值是否相同 ，相同就不处理，不相同则处理
	if((psx_button_bak[0] == psx_buf[3])
	&& (psx_button_bak[1] == psx_buf[4])) {				
	} else {		
		//处理buf3和buf4两个字节，这两个字节存储这手柄16个按键的状态
		parse_psx_buf(psx_buf+3, psx_buf[1]);
		psx_button_bak[0] = psx_buf[3];
		psx_button_bak[1] = psx_buf[4];
	}
	return;
}	

//--------------------------------------------------------------------------------

//软件复位函数，调用后单片机自动复位
void soft_reset(void) {
	__set_FAULTMASK(1);     
	NVIC_SystemReset();
}

//处理手柄按键字符，buf为字符数组，mode是指模式 主要是红灯和绿灯模式
void parse_psx_buf(unsigned char *buf, unsigned char mode) {
	u8 i, pos = 0;
	static u16 bak=0xffff, temp, temp2;
	temp = (buf[0]<<8) + buf[1];
	
	if(bak != temp) {
		temp2 = temp;
		temp &= bak;
		for(i=0;i<16;i++) {     //16个按键一次轮询
			if((1<<i) & temp) {
			} else {
				if((1<<i) & bak) {	//press 表示按键按下了
															
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED){
						if(ps_mode == 0){							
							memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
						}else if(ps_mode == 1){
							memcpy((char *)uart_receive_buf, (char *)ai_pre_cmd_set_red[i], strlen(ai_pre_cmd_set_red[i]));
						}
					}
															
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) uart_receive_buf[pos-1] = '\0';
					if(str_contain_str(uart_receive_buf, (u8 *)"$")) {
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf, (char *)cmd_return);
						uart1_get_ok = 1;
						uart1_open();
						uart1_mode = 1;
					} else if(str_contain_str(uart_receive_buf, (u8 *)"#")) {
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf,(char *) cmd_return);
						uart1_get_ok = 1;
						uart1_open();
						uart1_mode = 2;
					}
					bak = 0xffff;
				} else {//release 表示按键松开了
										
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED){
						if(ps_mode == 0){
							memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
						}else if(ps_mode == 1){
							memcpy((char *)uart_receive_buf, (char *)ai_pre_cmd_set_red[i], strlen(ai_pre_cmd_set_red[i]));
						}					
					}										
											
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) {
						if(str_contain_str(uart_receive_buf+pos, (u8 *)"$")) {
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							parse_cmd(uart_receive_buf);
							parse_group_cmd(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 1;
						} else if(str_contain_str(uart_receive_buf+pos, (u8 *)"#")) {
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							parse_action(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 2;
						}
						//uart1_send_str(uart_receive_buf);
					}	
				}
			}
		}
		bak = temp2;
		beep_on_times(1,100);
	}	
	return;
}

//命令解析函数
void parse_cmd(u8 *cmd) {
	int pos, i, index, int1, int2, int3, int4;
	//uart1_send_str(cmd);
	//逆运动学相关指令
	if(pos = str_contain_str(cmd, (u8 *)"$KMS:"), pos) {		
		if(sscanf((char *)cmd, "$KMS:%d,%d,%d,%d!", &int1, &int2, &int3, &int4)) {
			uart1_send_str((u8 *)"Try to find best pos:\r\n");
			if(kinematics_move(int1, int2, int3, int4)) {
				//beep_on_times(1,100);
			} else {
				//beep_on_times(2,100);
				uart1_send_str((u8 *)"Can't find best pos!!!");
			}					
		}
		//智能控制相关指令，
	}else if(pos = str_contain_str(cmd, (u8 *)"$QC!"), pos){
			beep_on_times(1,100);
			parse_group_cmd((u8 *)"$DGT:3-7,1!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$ZC!"), pos){
			beep_on_times(1,100);
			parse_group_cmd((u8 *)"$DGT:8-12,1!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$YC!"), pos){
			beep_on_times(1,100);
			parse_group_cmd((u8 *)"$DGT:13-17,1!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$QP!"), pos){
			beep_on_times(1,100);
			parse_group_cmd((u8 *)"$DGT:18-21,1!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$ZP!"), pos){
			beep_on_times(1,100);
			parse_group_cmd((u8 *)"$DGT:22-26,1!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$YP!"), pos){
			beep_on_times(1,100);
			parse_group_cmd((u8 *)"$DGT:27-31,1!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$QS!"), pos){
			beep_on_times(1,100);
			parse_group_cmd((u8 *)"$DGT:1-1,1!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$JQ!"), pos){
			beep_on_times(1,100);
			parse_action((u8 *)"#005P1700T1500!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$FX!"), pos){
			beep_on_times(1,100);
			parse_action((u8 *)"#005P1200T1500!");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$DJR!"), pos){
			beep_on_times(1,100);
			zx_uart_send_str((u8 *)"#255P1500T2000!\r\n");		
	}else if(pos = str_contain_str(cmd, (u8 *)"$MODE!"), pos){
			if(ps_mode == 0){
				ps_mode = 1;
			}else if(ps_mode == 1){
				ps_mode = 0;
			}		
	}
}


int kinematics_move(float x, float y, float z, int time) {
	int i,j, min = 0, flag = 0;
	
	if(y < 0)return 0;
	
	//寻找最佳角度
	flag = 0;
	for(i=0;i>=-135;i--) {
		if(0 == kinematics_analysis(x,y,z,i,&kinematics)){
			if(i<min)min = i;
			flag = 1;
		}
	}
	
	//用3号舵机与水平最大的夹角作为最佳值
	if(flag) {
		kinematics_analysis(x,y,z,min,&kinematics);
		for(j=0;j<4;j++) {
			set_servo(j, kinematics.servo_pwm[j], time);
		}
		return 1;
	}
	
	return 0;
}

void loop_pwm_monitor(void) {
	static u32 systick_ms_bak=0;
	int i;
	float sum;
	sum = 0;
	for(i=0;i<6;i++) {
		sum += duoji_doing[i].inc;
	}
	
	if(sum) {
		DjTimer_ON();
		systick_ms_bak = millis();
	}else {
		if(millis() - systick_ms_bak>500) {
			DjTimer_OFF();
		}
	}
}



