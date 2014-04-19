/*------------------------
DC Motor Speed Control
Close loop P control
With UART
Author: Xinyou Han, PMC, Wuhan University
Wuhan, Hubei Province, China
------------------------*/

/*------------------------
This module contains a DC motor, an optocoupler to measure speed
Timers should be used in PWM output, timing, pulse counting, and UART
However, 89C52 has just 3 timers
As PWM cycle is fixed, it can be also used in timing
At first it's designed to communicate with KingView
Somehow the communicate cannot be established (namely failed LOL)
Though communicate with PC via UART succeeded (with some soft in PC instead of KingView)
It's not completed yet

The actual speed is up to about 160n/s
P control is excellent
The motor driving ic is L9110
------------------------*/

//Crystal Frequency 12MHz

//TIMER0: PWM output and timing
//TIMER1: Serial port control
//TIMER2: Pulse count

//A phase of the motor connects to VCC, when B phase to PWM out(P1.2)



#include <reg52.h>
#include <math.h>
#define UINT unsigned int
#define UCHAR unsigned char

//------------------------
sbit SEG0 = P2^2;
sbit SEG1 = P2^3;
sbit SEG2 = P2^4;

UCHAR code table[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
//------------------------

#define N 4				//coefficient for duty cycle
#define DZ 5			//deadzone

#define START 0x40
#define END   0x0d
#define ADDR  0x01
#define READ  0x00
#define WRITE 0x01


//limits of duty cycle, 1% and 99%
#define Tmax 9900
#define Tmin 100

sbit PWM  = P1^2;
sfr T2MOD = 0xc9;

int N_d;				//actual speed
int N_s;				//expected speed
UCHAR FLAG;				//PWM duty cycle modulate, FLAG=0 keep, =1 decrease(speed up), =2 increase(speed down)

//UCHAR LIM;				//duty cycle limitation, LIM=0 normal, =1 minimun(fatest), =2 maximun(lowest)

//PWM cycle, unit:us, initial duty cycle is 50%, cycle:10ms
UINT T_H = 5000;
UINT T_L = 5000;

//define flag bits
bit OFW;				//counter overflow
bit TIMER;				//time up
bit RCV;				//MCU receive data success

//communication
UCHAR CNT = 0;
UCHAR ACK[10]   = {0x40,0x30,0x31,0x30,0x31,0x31,0x31,0x30,0x32,0x0d};
UCHAR ACK_WR[8] = {0x40,0x30,0x31,0x23,0x23,0x30,0x31,0x0d};
UCHAR BUFF[20]  = {0};	//receive buff
//UCHAR CRC = 0;
UCHAR DAT_ADDR  = 0;	//data addr
UCHAR RCV_FLAG  = 0;	//receive 1st byte flag

/*------------------------
FUNC:   delay
DESCR:  time delay
IN:     delaying time(ms)
OUT:    N/A
------------------------*/
void delay(UCHAR t)
{
	UCHAR i,j;
	for(i=t;i>0;i--)
		for(j=106;j>0;j--);
}

/*------------------------
FUNC:   TH0_cfg
DESCR:  set reg value of TIMER0, TH0
IN:     time
OUT:    TH0 value
------------------------*/
UCHAR TH0_cfg(UINT t)
{
	return (65536-t)/256;
}

/*------------------------
FUNC:   TL0_cfg
DESCR:  set reg value of TIMER0, TL0
IN:     time
OUT:    TL0 value
------------------------*/
UCHAR TL0_cfg(UINT t)
{
	return (65536-t)%256;
}

/*------------------------
FUNC:   asc2hex
DESCR:  transfer 2 ascii to hex
IN:     ascii high and low
OUT:    hex
------------------------*/
UCHAR asc2hex(UCHAR asc)
{
	UCHAR hex;
	if((asc>=0x30)&&(asc<=0x39))
		hex = asc-0x30;
	else if((asc>=0x41)&&(asc<=0x46))
		hex = asc-0x37;
	else
		hex = 0;
	return hex;
}

/*------------------------
FUNC:   hex2asc
DESCR:  transfer hex to ascii
IN:     hex
OUT:    ascii
------------------------*/
UCHAR hex2asc(UCHAR hex)
{
	UCHAR asc;
	if((hex>=0x00)&&(hex<=0x09))
		asc = hex+0x30;
	else if((hex>=0x0a)&&(hex<=0x0f))
		asc = hex+0x37;
	else
		asc = 0;
	return asc;
}

/*------------------------
FUNC:   UART_init
DESCR:  UART initialize, baud rate: 4800
IN:     N/A
OUT:    N/A
------------------------*/
void UART_init()
{
	TMOD |= 0x20;
	SCON  = 0x50;
	PCON  = 0x80;
	TH1   = 0xf3;
	TL1   = 0xf3;
	TR1   = 1;
	EA    = 1;
	ES    = 1;
	PS    = 1;
}

/*------------------------
FUNC:   init
DESCR:  main initialize
IN:     N/A
OUT:    N/A
------------------------*/
void init()
{
	TMOD  |= 0x01;	//TIMER0:mode 1 timer
	
	//TIMER2: 16-bit auto-reload counter
	RCLK   = 0;
	TCLK   = 0;
	EXEN2  = 0;
	TR2    = 1;
	C_T2   = 1;
	CP_RL2 = 0;
	T2MOD  = 0x00;
	
	//set initial value for timers
	TH0    = TH0_cfg(T_L);
	TL0    = TL0_cfg(T_L);
	TH2    = 0x00;
	TL2    = 0x00;
	RCAP2H = 0x00;
	RCAP2L = 0x00;
	
	//enable all interrupts, and timers on, timers interrupts on
	EA     = 1;
	ET0    = 1;
	TR0    = 1;
	ET2    = 1;
	EX0    = 0;		//enable external interrupt 1,disble EXIT0
	//EX1    = 1;
	
	//global variable initialize
	N_d    = 0;
	N_s    = 0;
	FLAG   = 0;
	//LIM    = 0;
	
	//flag bit initialize
	OFW    = 0;
	TIMER  = 0;
	RCV    = 0;
	
	PWM    = 0;		//PWM initial low
}

/*------------------------
FUNC:   display
DESCR:  digital tube display, for test
IN:     expected speed, actual speed
OUT:    N/A
------------------------*/
void display(int n, int s)
{
	int n1,n2,n3;		//n1: actual speed's ones, n2: tens, n3: hundreds
	int s1,s2,s3;		//s1: expected speed's ones, s2: tens, s3: hundreds
	n1 = n%10;
	n2 = n/10%10;
	n3 = n/100;
	s1 = s%10;
	s2 = s/10%10;
	s3 = s/100;
	P0 = 0x00;
	P2 = 0x00;
	
	SEG0 = 0;
	SEG1 = 0;
	SEG2 = 0;
	P0   = table[n1];
	delay(5);
	SEG0 = 1;
	SEG1 = 0;
	SEG2 = 0;
	P0   = table[n2];
	delay(5);
	SEG0 = 0;
	SEG1 = 1;
	SEG2 = 0;
	P0   = table[n3];
	delay(5);
	SEG0 = 0;
	SEG1 = 0;
	SEG2 = 1;
	P0   = table[s1];
	delay(5);
	SEG0 = 1;
	SEG1 = 0;
	SEG2 = 1;
	P0   = table[s2];
	delay(5);
	SEG0 = 0;
	SEG1 = 1;
	SEG2 = 1;
	P0   = table[s3];
	delay(5);
	//SEG0 = 1;
	//SEG1 = 1;
	//SEG2 = 1;
	//P0   = table[LIM];
	//delay(5);
}

/*------------------------
FUNC:   UART_send
DESCR:  send data to host
IN:     data, length
OUT:    N/A
------------------------*/
void UART_send(UCHAR *dat,UCHAR length)
{
	UCHAR i;
	ES = 0;
	for(i=0;i<length;i++)
	{
		SBUF = *(dat+i);
		while(!TI);
		TI   = 0;
	}
	ES = 1;
}

/*------------------------
FUNC:   dat_ack
DESCR:  process data from host, and acknowledge
IN:     N/A
OUT:    N/A
------------------------*/
void dat_ack()
{
	UCHAR temp[2] = {0};
	temp[0] = BUFF[11];
	temp[1] = BUFF[12];
	N_s     = asc2hex(temp[0])*16+asc2hex(temp[1]);
	if((BUFF[7]==0x30)&&(BUFF[8]==0x31))
		UART_send(ACK_WR,8);
}

/*------------------------
FUNC:   dat_wr
DESCR:  send actual speed to host
IN:     N/A
OUT:    N/A
------------------------*/
void dat_wr()
{
	UCHAR dat_xor,i,tmp0,tmp1,wr_buff;
	dat_xor = 0;
	ACK[5]  = hex2asc(N_d/16);
	ACK[6]  = hex2asc(N_d%16);
	for(i=1;i<7;i++)
		dat_xor ^= ACK[i];
	wr_buff = dat_xor;
	tmp0    = (wr_buff>>4)&0x0f;
	ACK[7]  = hex2asc(tmp0);
	tmp1    = wr_buff&0x0f;
	ACK[8]  = hex2asc(tmp1);
	UART_send(ACK,10);
}

/*------------------------
FUNC:   main
DESCR:  main func
IN:     N/A
OUT:    N/A
------------------------*/
void main()
{
//	UCHAR i;
	UART_init();
	init();
	while(1)
	{
		if(OFW)
		{
			//overflow, invalid data, N_d = 0
			OFW = 0;
			N_d = 0;
		}
		if(TIMER)
		{
			TIMER = 0;
			N_d   = TL2+TH2*256;
			N_d   = N_d*10;
			N_d   = N_d/4;
			TH2   = 0x00;
			TL2   = 0x00;
			TR0   = 1;
			TR2   = 1;
			dat_wr();
		}
		if((N_s-N_d)>DZ)			//expected > actual, speed up
			FLAG  = 1;
		else if((N_s-N_d)<(0-DZ))	//expected < actual, speed down
			FLAG  = 2;
		else
			FLAG  = 0;
		display(N_d,N_s);
		if(RCV)						//receive data from host
		{
			dat_ack();
			RCV = 0;				//reset receive flag
		}
//		UART_send();
	}
}

/*------------------------
FUNC:   T2_int
DESCR:  interrupt service of TIMER2
IN:     N/A
OUT:    N/A
------------------------*/
void T2_int() interrupt 5
{
	OFW = 1;
	TH2 = 0x00;
	TL2 = 0x00;
}

/*------------------------
FUNC:   UART_get
DESCR:  read data from host via UART
IN:     N/A
OUT:    N/A
------------------------*/
void UART_get() interrupt 4
{
	UCHAR temp;
	temp = SBUF;
	if(RCV_FLAG==0)
	{
		if(temp==START)
		{
			RCV_FLAG  = 1;
			BUFF[CNT] = temp;
			CNT++;
		}
	}
	else
	{
		BUFF[CNT] = temp;
		CNT++;
		if(temp==END)
		{
			RCV      = 1;
			CNT      = 0;
			RCV_FLAG = 0;
		}
	}
	RI = 0;	
}

/*------------------------
FUNC:   PWM_out
DESCR:  PWM output, as well as timing
IN:     N/A
OUT:    N/A
------------------------*/
void PWM_out() interrupt 1
{
	static UCHAR i;
	UCHAR delta;				//delta=|N_s-N_d|/4
	i++;						//i+1 when PWM reverses
	TR0 = 0;
	if(i==20)					//if PWM reverses for 20 times, it's 10 cycle, namely 0.1s
	{
		TR0   = 0;
		TR2   = 0;
		TIMER = 1;
		i     = 0;
	}
	switch(FLAG)
	{
		case 0:					//speed keep, PWM keep
			if(PWM)				//currently PWM is high, then next time is low, hereinafter
			{
				TH0 = TH0_cfg(T_L);
				TL0 = TL0_cfg(T_L);
				TR0 = 1;
				PWM = 0;
			}
			else				//currently PWM is low, then next time is high, hereinafter
			{
				TH0 = TH0_cfg(T_H);
				TL0 = TL0_cfg(T_H);
				TR0 = 1;
				PWM = 1;
			}
			break;
		case 1:					//speed up, duty cycle decreases
			delta    = abs(N_s-N_d)/N;
			if(T_H>(Tmin+delta))
			{
				T_H -= delta;
				T_L += delta;
				//LIM  = 0;
			}
			else				//duty cycle minimum
			{
				T_H = Tmin;
				T_L = Tmax;
				//LIM = 1;
			}
			if(PWM)
			{
				TH0 = TH0_cfg(T_L);
				TL0 = TL0_cfg(T_L);
				TR0 = 1;
				PWM = 0;
			}
			else
			{
				TH0 = TH0_cfg(T_H);
				TL0 = TL0_cfg(T_L);
				TR0 = 1;
				PWM = 1;
			}
			break;
		case 2:					//speed down, duty cycle increases
			delta    = abs(N_s-N_d)/N;
			if(T_H<(Tmax-delta))
			{
				T_H += delta;
				T_L -= delta;
				//LIM  = 0;
			}
			else				//duty cycle maximum
			{
				T_H = Tmax;
				T_L = Tmin;
				//LIM = 2;
			}
			if(PWM)
			{
				TH0 = TH0_cfg(T_L);
				TL0 = TL0_cfg(T_L);
				TR0 = 1;
				PWM = 0;
			}
			else
			{
				TH0 = TH0_cfg(T_H);
				TL0 = TL0_cfg(T_H);
				TR0 = 1;
				PWM = 1;
			}
			break;
		default:
			break;
	}
}

	
	
	
	
	
	
