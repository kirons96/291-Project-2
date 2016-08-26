#include <C8051f38x.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#define  SYSCLK         48000000L // System clock frequency in Hz
#define  BAUDRATE       115200L
#define  SMB_FREQUENCY  100000L   // I2C SCL clock rate (10kHz to 100kHz)

#define  LED        P2_2
#define  LED_ON     0
#define  LED_OFF    1

#define OUT0 P2_0
#define OUT1 P2_1
#define OUT2 P2_2
#define OUT3 P2_3

volatile unsigned char pwm_count=0;
volatile int PWM1 = 0;
volatile int PWM2 = 0;
volatile int PWM3 = 0;
volatile int PWM4 = 0;

char _c51_external_startup (void)
{
	PCA0MD&=(~0x40) ;  // DISABLE WDT: clear Watchdog Enable bit
	VDM0CN=0x80;       // Enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == 12000000L)
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4
	#elif (SYSCLK == 24000000L)
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == 48000000L)
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12000000L, 24000000L, or 48000000L
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency

	#if (SYSCLK/BAUDRATE/2L/256L < 1)
		TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
		CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
		CKCON |=  0x08;
	#elif (SYSCLK/BAUDRATE/2L/256L < 4)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 01
		CKCON |=  0x01;
	#elif (SYSCLK/BAUDRATE/2L/256L < 12)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 00
	#else
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 10
		CKCON |=  0x02;
	#endif

	TL1 = TH1;     // Init timer 1
	TMOD &= 0x0f;  // Mask out timer 1 bits
	TMOD |= 0x20;  // TMOD: timer 1 in 8-bit autoreload
	TR1 = 1;       // Start timer1
	SCON = 0x52;

	// Initialize Crossbar and GPIO
	P0MDOUT = 0x10;           // Enable Uart TX as push-pull output
	P2MDOUT |= 0b0000_0110;   // Make the LED (P2.2) a push-pull output.  P2.1 used for debuging.
	XBR0 = 0b0000_0101;       // Enable SMBus pins and UART pins P0.4(TX) and P0.5(RX)
	XBR1 = 0x40;              // Enable crossbar and weak pull-ups
	XBR2 = 0b0000_0001;       // Enable UART1

	// Configure Timer 0 as the I2C clock source
	CKCON |= 0x04; // Timer0 clock source = SYSCLK
	TMOD &= 0xf0;  // Mask out timer 1 bits
	TMOD |= 0x02;  // Timer0 in 8-bit auto-reload mode
	// Timer 0 configured to overflow at 1/3 the rate defined by SMB_FREQUENCY
	TL0 = TH0 = 256-(SYSCLK/SMB_FREQUENCY/3);
	TR0 = 1; // Enable timer 0

	// Configure and enable SMBus
	SMB0CF = INH | EXTHOLD | SMBTOE | SMBFTE ;
	SMB0CF |= ENSMB;  // Enable SMBus

	LED = LED_OFF;

	// Initialize timer 2 for periodic interrupts
	TMR2CN=0x00;   // Stop Timer2; Clear TF2;
	CKCON|=0b_0001_0000;
	TMR2RL=(-(SYSCLK/(2*48))/(100L)); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2

	EA=1; // Enable interrupts

	return 0;
}

void Timer2_ISR (void) interrupt 5
{
	unsigned char temp_fix;

	temp_fix = SFRPAGE;

	SFRPAGE = 0;

	TF2H = 0; // Clear Timer2 interrupt flag

	pwm_count++;
	if(pwm_count>100) pwm_count=0;

	OUT0=pwm_count>PWM1?0:1;
	OUT1=pwm_count>PWM2?0:1;
	OUT2=pwm_count>PWM3?0:1;
	OUT3=pwm_count>PWM4?0:1;

	SFRPAGE = temp_fix;
}

// Uses Timer4 to delay <ms> mili-seconds.
void Timer4ms(unsigned char ms)
{
	unsigned char i;// usec counter
	unsigned char k;

	k=SFRPAGE;
	SFRPAGE=0xf;
	// The input for Timer 4 is selected as SYSCLK by setting bit 0 of CKCON1:
	CKCON1|=0b_0000_0001;

	TMR4RL = 65536-(SYSCLK/1000L); // Set Timer4 to overflow in 1 ms.
	TMR4 = TMR4RL;                 // Initialize Timer4 for first overflow

	TMR4CN = 0x04;                 // Start Timer4 and clear overflow flag
	for (i = 0; i < ms; i++)       // Count <ms> overflows
	{
		while (!(TMR4CN & 0x80));  // Wait for overflow
		TMR4CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR4CN = 0 ;                   // Stop Timer4 and clear overflow flag
	SFRPAGE=k;
}

void UART1_Init (unsigned long baudrate)
{
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	if (((SYSCLK/baudrate)/2L)/0xFFFFL < 1){
		SBRL1 = 0x10000L-((SYSCLK/baudrate)/2L);
		SBCON1 |= 0x03; // set prescaler to 1
	}
	else if (((SYSCLK/baudrate)/2L)/0xFFFFL < 4){
		SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/4L);
		SBCON1 &= ~0x03;
		SBCON1 |= 0x01; // set prescaler to 4
	}
	else if (((SYSCLK/baudrate)/2L)/0xFFFFL < 12){
		SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/12L);
		SBCON1 &= ~0x03; // set prescaler to 12
	}
	else{
		SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/48L);
		SBCON1 &= ~0x03;
		SBCON1 |= 0x02; // set prescaler to ?
	}
	SCON1 |= 0x02; // indicate ready for TX
	SBCON1 |= 0x40; // enable baud rate generator

	XBR2=0x01; // Enable UART1 on P0.0(TX1) and P0.1(RX1)
	XBR1=0x40; // Enable crossbar and weak pull-ups
}

void putchar1 (char c)
{
	if (c == '\n' )
	{
		while (!(SCON1 & 0x02));
		SCON1 &= ~0x02;
		SBUF1 = '\r' ;
	}
	while (!(SCON1 & 0x02));
	SCON1 &= ~0x02;
	SBUF1 = c;
}

char getchar1 (void)
{
	char c;
	while (!(SCON1 & 0x01));
	SCON1 &= ~0x01;
	c = SBUF1;
	return (c);
}
void InitADC (void)
{
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN = 0b_0000_1000; //Select VDD as the voltage reference for the converter
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;

	mask=1<<pinno;

	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 3:
			P3MDIN &= (~mask); // Set pin as analog input
			P3SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	AMX0P = pin;             // Select positive input from pin
	AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
	// Dummy conversion first to select new pin
	AD0BUSY=1;
	while (AD0BUSY); // Wait for dummy conversion to finish
	// Convert voltage at the pin
	AD0BUSY = 1;
	while (AD0BUSY); // Wait for conversion to complete
	return (ADC0L+(ADC0H*0x100));
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*3.30)/1024.0);
}

// Uses Timer3 to delay <us> micro-seconds.
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter

	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON:
	CKCON|=0b_0100_0000;

	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow

	TMR3CN = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN & 0x80));  // Wait for overflow
		TMR3CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

void stopMoving(){
	PWM2 = 0;
	PWM3 = 0;
	PWM1 = 0;
	PWM4 = 0;
}

void moveUp(){
	PWM2 = 60;
	PWM4 = 60;
	PWM1 = 0;
	PWM3 = 0;
}

void moveDown(){
	PWM1 = 60;
	PWM3 = 60;
	PWM2 = 0;
	PWM4 = 0;
}

void moveLeft(){
	PWM2 = 60;
	PWM3 = 60;
	PWM1 = 0;
	PWM4 = 0;
}

void moveRight(){
	PWM1 = 60;
	PWM4 = 60;
	PWM2 = 0;
	PWM3 = 0;
}

void moveFastUp(){
	PWM2 = 100;
	PWM4 = 100;
	PWM1 = 0;
	PWM3 = 0;
}

void moveFastDown(){
	PWM1 = 100;
	PWM3 = 100;
	PWM2 = 0;
	PWM4 = 0;
}

void moveFastLeft(){
	PWM2 = 100;
	PWM3 = 100;
	PWM1 = 0;
	PWM4 = 0;
}

void moveFastRight(){
	PWM1 = 100;
	PWM4 = 100;
	PWM2 = 0;
	PWM3 = 0;
}

void main (void)
{
	volatile float coil_left = 0.0, ten_percent_coil_left = 0.0;
	volatile float coil_right = 0.0, ten_percent_coil_right = 0.0;
	volatile float d1 = 0, d2 = 0;
	int initialized = 0;
	int mode = 0;
	char test = 's';
	//volatile float coil_right;

	printf("%c\n",test);

	InitPinADC(1, 5); // Configure P1.5 as analog input
	InitPinADC(2, 6); // Configure P2.6 as analog input
		// Initialize the ADC
	InitADC();

	while(1){

		char direction = 's';
		UART1_Init(110);
		
		if (P0_1==0){
			mode = !mode;
			printf("switched");
		}
		
		if (mode == 0){
			direction = getchar1();
			SCON1 |= 0x01;
			
			if(direction=='u'){
				moveFastUp();
				initialized = 0; //prepare to switch modes back
			}
			else if(direction=='d'){
				moveFastDown();
				initialized = 0; //prepare to switch modes back
			}
			else if (direction=='r'){
				moveFastRight();
				initialized = 0; //prepare to switch modes back
			}
			else if(direction=='l'){
				moveFastLeft();
				initialized = 0; //prepare to switch modes back
			}
			else if(direction=='s'){
				stopMoving();
				initialized = 0; //prepare to switch modes back
			}
		}
		else if (mode == 1){
			stopMoving();

			if (initialized == 0){ //initialize the two distances
				d1 = Volts_at_Pin(LQFP32_MUX_P1_5);
				d2 = Volts_at_Pin(LQFP32_MUX_P2_6);
				initialized = 1;
			}

			coil_left = Volts_at_Pin(LQFP32_MUX_P1_5); //get ADC voltages

			ten_percent_coil_left = 0.2*coil_left;

			coil_right = Volts_at_Pin(LQFP32_MUX_P2_6);

			ten_percent_coil_right = 0.2*coil_right;

			//left/right

			if (coil_left > (coil_right+ten_percent_coil_right)){
				moveFastLeft();
				test = 'r';
			}
			else if (coil_left < (coil_right-ten_percent_coil_right)){
				moveFastRight();
				test = 'l';
			}

			//forwards/backwards

			else if (coil_left < (coil_right+ten_percent_coil_right) && coil_left > (coil_right-ten_percent_coil_right)){ //robot is facing forwards

				if (coil_left < (d1-ten_percent_coil_left) && coil_right < (d2-ten_percent_coil_right)){
					moveFastUp();
					test = 'u';
				}
				else if (coil_left > (d1+ten_percent_coil_left) && coil_right > (d2+ten_percent_coil_right)){
					moveFastDown();
					test = 'd';
				}
				else{
					stopMoving();
					test = 's';
				}
			}

			Timer4ms(100);
		}


		printf("left=%5.3f right=%5.3f direction=%c mode=%d\r", coil_left, coil_right, direction, mode); //print data


		//printf("%d %d %d %d\r", PWM1, PWM2, PWM3, PWM4);
	}
}

