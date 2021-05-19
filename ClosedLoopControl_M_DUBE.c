//Mayibongwe Dube 216055670
//CLOSED LOOP Brushless Dc motor control
// MPLAB IDE v8.91 was used
//compiler used was the C30
//p30f2010.gld	-- Linker script file

#include "p30F2010.h"

#define FCY  10000000			// xtal = 5.0Mhz; PLLx8
#define MILLISEC FCY/10000			// 1 mSec delay constant
#define FPWM 39000
#define POLEPAIRS	5		// number of pole pairs of the motor
#define INDEX	1			// Hall sensor position index

#define S2	!PORTCbits.RC14
#define S3	!PORTCbits.RC13
#define SPEEDMULT	2343750		// factor used to calculate speed
#define POTMULT 4				// potetiometer to speed ratio

#define Kps	750					// Kp is the proportional constant
#define Kis	20					// Ki is the integral constant


void InitADC10(void);
void AverageADC(void);
void DelayNmSec(unsigned int N);
void InitMCPWM(void);
void InitTMR3(void);
void CalculateDC(void);



struct {
			unsigned RunMotor : 	1;
			unsigned CalSpeed :		1;
			unsigned unused 	:	12;
		}	Flags;

unsigned int HallValue;
unsigned int timer3value;
unsigned int timer3avg;
unsigned char polecount;
int DesiredSpeed;
int ActualSpeed;
int SpeedError;
int DutyCycle;
int SpeedIntegral;

unsigned int StateTable[] = {0x0000, 0x0210, 0x2004, 0x0204,
									0x0801, 0x0810, 0x2001, 0x0000}; 

	//Low side driver table is as below.  In the StateLoTableClk,	the Low side driver is a PWM while the high side driver is	either on or off.  


unsigned int StateLoTableClk[] = {0x0000, 0x0210, 0x2004, 0x0204,0x0801, 0x0810, 0x2001, 0x0000};
unsigned int StateLoTableAntiClk[] = {0x0000, 0x2001, 0x0810, 0x0801,0x0204, 0x2004, 0x0210, 0x0000};


//Interrupt vector for Change Notification CN5, 6 and 7 is as below.When a Hall sensor changes states, an interrupt will be 
//caused which will vector to the routine below.The program then reads PORTB, mask bits 3, 4 and 5,
//shift and adjust the value to read as 1, 2 ... 6.  This  value is then used as an offset in the lookup table StateLoTableClk 
//or StateLoTableAntiClk to determine the value loaded in the OCDCON 
//register.  This routine also reads the Hall sensors and counts
//up to the POLEPAIRS to determine the time for one mechanical
//revolution using the fact that 1 mech rev = POLEPAIR*(1 elect. rev)

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt (void)
{
	IFS0bits.CNIF = 0;				// clear flag
	HallValue = PORTB & 0x0038;	// mask RB3,4 & 5
	HallValue = HallValue >> 3;	// shift right 3 times
	OVDCON = StateLoTableAntiClk[HallValue];
// The code below is uses TMR3 to calculate the speed of the rotor
	if (HallValue == INDEX)  // has the same position been sensed?
		if (polecount++ == POLEPAIRS)  //has one mech rev elasped?
		{								// yes then read timer 3
		timer3value = TMR3;
		TMR3 = 0;
		timer3avg = ((timer3avg + timer3value) >> 1);
		polecount = 1;
		} 
}


//The ADC interrupt reads the demand pot value. 

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt (void)
{
	IFS0bits.ADIF = 0;	
	DesiredSpeed = ADCBUF0;
	Flags.CalSpeed = 1;		// after every adc read,  do a PI calculation
}

int main(void)
{
	LATE = 0x0000;
	TRISE = 0xFFC0;		// PWMs are outputs
	CNEN1 = 0x00E0;		// CN5,6 and 7 enabled
	CNPU1 = 0x00E0;		// enable internal pullups
	IFS0bits.CNIF = 0;	// clear CNIF
	IEC0bits.CNIE = 1;	// enable CN interrupt
	InitMCPWM();
	InitADC10();
	InitTMR3();
	timer3avg = 0;
	while(1)
	{		
		while (!S2);				// wait for start key hit
		while (S2)					// wait till key is released
			DelayNmSec(10);
		// read hall position sensors on PORTB
		HallValue = PORTB & 0x0038;			// mask RB3,4 & 5
		HallValue = HallValue >> 3;			// shift right to get value = 1, 2 ... 6
		OVDCON = StateLoTableAntiClk[HallValue];	// Load the overide control register
		PWMCON1 = 0x0777;					// enable PWM outputs
		Flags.RunMotor = 1;					// set running flag
		T3CONbits.TON = 1;					// start tmr3
		polecount = 1;
		DelayNmSec(100);
		while (Flags.RunMotor)				// while motor is running
			{
			if (S2)							// if S2 is pressed, stop
				{
				PWMCON1 = 0x0700;			// disable PWM outputs
  				OVDCON = 0x0000;			// overide PWM low.
				Flags.RunMotor = 0;			// reset run flag
				while (S2)					// wait for key release
					DelayNmSec(10);
				}
			if (Flags.CalSpeed)		// if calculate flag set then 
				{
				CalculateDC();		// calculate new dutycycle using PI
				Flags.CalSpeed = 0;		// clear flag 
				}
			}
	}	// end of while (1)
	

}	// end of main
	
/*
	Below is the setup code ADC registers for :
        1.channel conversion (in this case RB2/AN2)
    	2. PWM trigger starts conversion
		3. Pot is connected to CH0 and RB2
		4. AD interrupt is set and buffer is read in the interrupt 
																*/
void InitADC10(void)
{

 ADPCFG = 0xFFF8;				// all PORTB = Digital;RB0 to RB2 = analog
 ADCON1 = 0x0064;				// PWM starts conversion
 ADCON2 =  0x0200;				// simulataneous sample 4 channels									
 ADCHS = 0x0002;				// Connect RB2/AN2 as CH0 = pot  
 ADCON3 = 0x0080;				// Tad = internal RC (4uS)
 IFS0bits.ADIF = 0;
 IEC0bits.ADIE = 1;

 ADCON1bits.ADON = 1;			// turn ADC ON
}

/*
InitMCPWM, intializes the Motor PWM as follows:
1. FPWM = 39000 hz at 10Mips
2. Independant PWMs
3. Control outputs using OVDCON
4. Init Duty Cycle with a value = 100
5. Set ADC to be triggered by PWM special trigger
*/

void InitMCPWM(void)
{
	PTPER = FCY/FPWM - 1;		// set the pwm period register

	PWMCON1 = 0x0700;			// disable PWMs
	OVDCON = 0x0000;			// allow control using OVD register
	PDC1 = 100;					// init PWM DC 1, 2 and 3 to 100
	PDC2 = 100;
	PDC3 = 100;
	SEVTCMP = PTPER;		// set ADC to trigeer at ...
	PWMCON2 = 0x0F00;		// 16 PWM values
	PTCON = 0x8000;			// start PWM however output ...
							// is enabled by OVDCON which is OFF
	 
}

/*
Tmr3 is used to determine the rotor speed so it is set to count using Tcy/256

*/

void InitTMR3(void)
{
	T3CON = 0x0030;			// internal Tcy/256 clock
	TMR3 = 0;
	PR3 = 0xFFFF;
}

/*
CalculateDC, uses the PI algorithm to calculate the new DutyCycle value which
will get loaded into the PDCx registers.
*/

void CalculateDC(void)
{
	ActualSpeed = SPEEDMULT/timer3avg;
	DesiredSpeed = DesiredSpeed*POTMULT;
	SpeedError = DesiredSpeed - ActualSpeed;
			SpeedIntegral += SpeedError;
	DutyCycle = (((long)Kps*(long)SpeedError + (long)Kis*(long)SpeedIntegral) >> 16);
	PDC1 = PDC1 + DutyCycle;	
	if (PDC1 < 50)
		{PDC1 = 50;SpeedIntegral = 0;}
	if (PDC1 > 512)
		{PDC1 = 512;SpeedIntegral = 0;} 
	PDC2 = PDC1;
	PDC3 = PDC1;

}
// This is a generic 1ms delay routine to give a 1mS to 65.5 Seconds delay

void DelayNmSec(unsigned int N)
{
unsigned int j;
while(N--)
 	for(j=0;j < MILLISEC;j++);
}