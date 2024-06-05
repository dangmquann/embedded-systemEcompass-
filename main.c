#include "MKL46Z4.h"
#include "lcd.h"
#include "mag.h"
#include "i2c.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"



// mac dinh cac chan thanh ghi
#define PORTC_D_IRQ_NBR (IRQn_Type) 31 // Thanh ghi ngat o Port D va C la 31
#define Sw1_pin (3)
#define Sw2_pin (12)
#define green_led_pin				 (1<<5)
#define red_led_pin 				 (1<<29)

// cac ham dieu khien LED
#define GREEN_LED_TOGGLE()   PTD->PTOR |= green_led_pin;
#define RED_LED_TOGGLE()   	 PTE->PTOR |= red_led_pin;
#define GREEN_LED_OFF()      PTD->PSOR |= green_led_pin;
#define RED_LED_OFF()        PTE->PSOR |= red_led_pin;
#define GREEN_LED_ON()       PTD->PCOR |= green_led_pin;
#define RED_LED_ON()         PTE->PCOR |= red_led_pin;

unsigned char DATA_READ[6];
short int MAG_DATA_READ_AXIS[3];
short int MAG_DATA_MAX_AXIS[3];
short int MAG_DATA_MIN_AXIS[3];
short int MAG_DATA_AVERAGE_AXIS[3];
short int MAG_DATA_HI_CALIBRATED[3];
unsigned char DR_STATUS_DATA;
short int ANGLE;

// khoi tao cac chan
void initLed(void);
void initSwitch(void);
void init_systick_interupt(void);
void SetSystemClock(void);

// khoi tao cac ham
void PORTC_PORTD_IRQHandler(void);
void reset_system_state(void);

// khai bao cac bien toan cuc
int system_active = 0;
int32_t volatile msTicks = 0;	

void SetGPIOPriority();
void SetSysTickPriority();

typedef enum {
	STOP,
	RUN,
	MAG_ACQ,
	MAG_CAL,
	ACC_CAL
} enumECompassOperationState;

enumECompassOperationState enumECompassState = STOP;

bool bSW3Pressed = false;
bool bSW1Pressed = false;

bool bIsTimerExpired = false;

unsigned char    ucSecond = 0;
unsigned char    ucHundredsMilliSecond = 0;
unsigned char    ucMinute = 0;
unsigned short   usTimeElapsed = 0;

unsigned char    ucaryLCDMsg[5] = "";
/*========================================================*/
/* Define and initialise variables*/
/* =======================================================*/
// --------------------------------------------------------------------
// Defining variables to be used for calculation of TPM0 MOD Count for Timer Overflow every 0.1 second
int prescalar;
int prescalar_factor;//to be
int resolution;
int original_clock_frequency; //in kHz
int new_clock_frequency ;
int timer_overflow_count;
int reset_count;

// Defining variables to be used for acquiring and processing Accelerometer Data
unsigned char DR_STATUS_DATA_ACC;

// Defining variables to be used as flags
int flag_first_data = 0;
int flag_initialisation = 0;
int flag_first_data_acquire =0;
int flag_first_data_run =0;
int flag_first_data1 =0;

void TIMER_Init(void)
{
// --------------------------------------------------------------------
	/*===========================================================================================================================*/
	/*Calculation of the MOD Value or upper limit for the TPM0 Counter to trigger a Timer Overflow Interrupt:*/
	/*User can change the prescalar factor and resolution if needed and the upper count limit will be calculated automatically*/
	//TPM counter increments on every TPM counter clock
	//Prescale Factor set to 6, It divides clock frequency by 2^6 = 64.New Frequency = 8MHz/64 = 0.125 MHz = 125kHz
    //Time period = 1/New Frequency.
	//timer_overflow_count = resolution/time period = resolution * original_clock_frequency / prescalar = 12500
	/*===========================================================================================================================*/
	prescalar_factor = 6;//user specified prescalar factor
	prescalar = 1<<(prescalar_factor);//Left shift to find the prescalar
	resolution = 100; //The lowest count in milliseconds (Can be modified by user)
	original_clock_frequency = 8000; //The Frequency of the onboard crystal oscillator in kHz
	timer_overflow_count = resolution * original_clock_frequency / prescalar;//Calculating the MOD value after which the Timer Overflow occurs and an interrupt is triggered
	reset_count = 0;//Reset value for TPM0
	/*===========================================================================================================================*/
	/*Connect Clock Source to the TPM0 */
	/*===========================================================================================================================*/
	OSC0->CR |= (1<<7); //Enabling the external reference clock through ERCLKEN bit in OSC0 Control Register
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; //Enable TPM0 clock using System Clock Gating Control Register 6
	SIM->SOPT2 |= (1<<25); //Selecting the OSCERCLK as source of TPM0 by setting TPMSRC to 10 in System Options Register 2
	/*===========================================================================================================================*/
	/*Resetting TPM0 Count Values and Specifying the upper limit(MOD value) for the TPM0 Counter */
	/*===========================================================================================================================*/
	TPM0->CNT = reset_count; //The CNT register of TPM0 is reset to 0
	TPM0->MOD = timer_overflow_count; //Setting the maximum count value. TPM0 counter reaches this modulo value and increments, the overflow flag (TOF) is set.
	/*===========================================================================================================================*/
	/*Status and Control register configuration for the TPM0 */
	/*===========================================================================================================================*/
	TPM0->SC &=(~(TPM_SC_CPWMS_MASK));//CPWMS is set to 0 so that TPM0 Counter operates in upcounting mode
	TPM0->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(prescalar_factor);//CMOD is set to 01 so that TPM counter increments on every TPM counter clock and PS is set to user defined prescalar value
	TPM0->SC |= TPM_SC_TOIE_MASK; //Enabling the timer to raise an interrupt on overflow every hundred-millisecond
	TPM0->CONF |= (1<<7) | (1<<6); //Setting DBGMODE to be 11  in TPM0_CONF so that TPM counter continues in debug mode
// --------------------------------------------------------------------
}

/*========================================================*/
/* Function for Timer Interrupt Handler*/
/* =======================================================*/
void TPM0_IRQHandler(void)
{
// --------------------------------------------------------------------
/*===========================================================================================================================*/
/*Update the Timer Expired state variable and Clearing the Timer Overflow Flag for the TPM0 */
/*===========================================================================================================================*/
	bIsTimerExpired = true;//This indicates that 1 unit lowest count/resolution(hundred-millisecond) has reached
	TPM0->SC |= TPM_SC_TOF_MASK; //Clears the overflow flag TOF so that clock can start counting the next lowest count or resolution (hundred-millisecond)
// --------------------------------------------------------------------
}
/*========================================================*/
/* Function for PORT C and PORT D Interrupt Handler*/
/* =======================================================*/
void PORTC_PORTD_IRQHandler(void)
{
// --------------------------------------------------------------------
	int SW1_Status, SW2_Status; //Indicates the status of SW1 and SW2 (if they are pressed or not)
	/*===========================================================================================================================*/
	/*Read Interrupt Status Flag for Switch 1 and Switch 2 and storing the status after right shifting to indicate a 0 or 1 value*/
	/*===========================================================================================================================*/
	SW1_Status=(PORTC->PCR[Sw1_pin] & (1<<24))>>24;
	SW2_Status=(PORTC->PCR[Sw2_pin] & (1<<24))>>24;
	/*===========================================================================================================================*/
	/*Update the corresponding State variables and Clearing Interrupt Status Flags for the corresponding switches for detecting future interrupts */
	/*===========================================================================================================================*/
	if (SW1_Status==1)
	{
		bSW1Pressed=1;      //Update the state variable for Reset Switch (Switch 1)
		PORTC->PCR[Sw1_pin] |= (1<<24); //Clearing the Interrupt Status Flag for Switch 1
	}
	if (SW2_Status==1)
	{
		bSW3Pressed=1;  //Update the state variable for Start/Stop Toggle Switch (Switch 2)
		PORTC->PCR[Sw2_pin] |= (1<<24); //Clearing the Interrupt Status Flag for Switch 2
	}
  if (PORTC->ISFR & (1U << Sw1_pin)) {
        system_active = !system_active;
        PORTC->ISFR = (1U << Sw1_pin); // xoa co
    }
    if (PORTC->ISFR & (1U << Sw2_pin)) {
        reset_system_state(); // ham reset trang thai
        PORTC->ISFR = (1U << Sw2_pin); // xoa co
    }
// --------------------------------------------------------------------
}





int main()
{
	__disable_irq();
	
	SetSystemClock();
	initLed();
	initSwitch();
	init_systick_interupt();
	SetSysTickPriority(1);
	SetGPIOPriority(0);
	LCD_Init();
	BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
            	    TIMER_Init();
            	    initI2C0();
            	    /* Enable individual interrupt */
            	    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
            	    NVIC_EnableIRQ(TPM0_IRQn);

            	    /* Enable global interrupt */
            	    __enable_irq();
	
	while(1){
		if(bSW3Pressed == true){
          // Clear the flag
            bSW3Pressed = false;
            /*if(enumECompassState == STOP){
                
            }else if(enumECompassState == MAG_ACQ){
                
            }else if(enumECompassState == MAG_CAL){
                
            }*/
						flag_initialisation = 0;
						initMagnetometer();
            flag_initialisation = 1;
		
        // Check if SW1 is pressed
        }else if(bSW1Pressed == true){
          // Clear the flag
            bSW1Pressed = false;
            if(enumECompassState == STOP){
                // Nothing to be done
								//RED_LED_OFF();
                //GREEN_LED_OFF();
                enumECompassState = MAG_ACQ;
            }else if(enumECompassState == MAG_ACQ){
                // Nothing to be done
							enumECompassState = MAG_CAL;
            }else if(enumECompassState == MAG_CAL){
								//RED_LED_OFF();
                //GREEN_LED_OFF();
                enumECompassState = RUN;
                // Nothing to be done
            }else if(enumECompassState == RUN){
				//Stop if SW1 is pressed
                enumECompassState = STOP;
            }
        }
        /* Carry out the given tasks defined in the current state */
        if(enumECompassState == STOP){

        		//Initialising all flags and data variables
        	    //RED_LED_ON();
        	    //GREEN_LED_OFF();
        	    flag_first_data = 0;
        	    flag_initialisation = 0;
        	    flag_first_data_acquire =0;
        	    flag_first_data_run =0;
        	    for(int i =0;i<3;i++)
        	    	{MAG_DATA_MAX_AXIS[i]=0;
        	    	 MAG_DATA_MIN_AXIS[i]=0;
        	    	 MAG_DATA_READ_AXIS[i]=0;}
        	    //Display STOP Message
        	    LCD_Display((unsigned char *)"STOP");
							GREEN_LED_OFF();
							system_active = 0;
							
        }
				else if(enumECompassState == RUN)
				{
			//The e-compass heading is updated every 100ms using the timer IRQ

				//Polling the DR_Status register data to read Magnetometer whenever new data is available 
        		DR_STATUS_DATA_ACC = readI2C_single(MAG_DEVICE_ADDRESS,MAG_DR_STATUS);
            	if((DR_STATUS_DATA_ACC!=0)||(flag_first_data_run ==0))
					//Update Magnetometer Data
            		{
									Magnetometer_Run();
									flag_first_data_run =1;
            		}
            		// Displaying current magnetometer heading angle wrt magnetic north on LCD in degrees
				while(bIsTimerExpired == true)
					//clear bIsTimerExpired Flag
					{
								bIsTimerExpired = false;
            		snprintf(ucaryLCDMsg,5,"%4d",ANGLE);
            		LCD_Display(ucaryLCDMsg);

					/* The green LED should be turned on between 345° to 15° (± 15° tolerance) as the e-compass is heading towards
					north magnetic pole (Otherwise, the green LED should be turned off).*/
            		
            			//GREEN_LED_ON();
            		system_active = 1;
						

            	}//endwhile
        }
			else if(enumECompassState == MAG_ACQ)
				{		
						RED_LED_OFF();
            GREEN_LED_OFF();
            if(flag_initialisation == 0)
            {   //initialise the Magnetometer once
            	initMagnetometer();
            	flag_initialisation = 1;
            }
			//Polling the DR_Status register data to read Magnetometer whenever new data is available 
            DR_STATUS_DATA = readI2C_single(MAG_DEVICE_ADDRESS,MAG_DR_STATUS);
            if((DR_STATUS_DATA!=0)||((flag_first_data_acquire ==0)))
               	{//Acquire Magnetometer data for calibration purpose
            	 Magnetometer_Acq();
            	 flag_first_data_acquire =1;
            	 //clear bIsTimerExpired Flag
            	 //bIsTimerExpired = false;
            	 }
			//Display MACQ Message
        	LCD_Display((unsigned char *)"MACQ");

       }
			else if(enumECompassState == MAG_CAL)
			{		
					RED_LED_OFF();
          GREEN_LED_OFF();
			//Calibrate Magnetometer
        	Magnetometer_Avg();
			//Display MCAL Message
        	LCD_Display((unsigned char *)"MCAL");

      }
    }
	return 0;
}

// thiet lap systemclock
void SetSystemClock(void) {
    // ghi vào thanh ghi MCG_C1
    MCG->C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(0);
		// Thiet lap tan so
    MCG->C4 = (MCG->C4 & ~MCG_C4_DMX32_MASK) | MCG_C4_DRST_DRS(0x5);
		// chon nguon module
    SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_PLLFLLSEL_MASK) | SIM_SOPT2_PLLFLLSEL(0x3);
    while (!(MCG->S & MCG_S_IREFST_MASK));
		// thiet lap tan so ghi vao thanh ghi MCG_C4
    MCG->C4 = (MCG->C4 & ~MCG_C4_DRST_DRS_MASK) | MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x5);
}


// khoi tao cac chan led do va xanh
void initLed(void)
{
		// Thiet lap led xanh
		SIM->SCGC5 |= (1 << 12);   // Cap clock cho Port D
		PORTD->PCR[5] = (1 << 8);  // Thiet lap chan 5 la GPIO
		PTD->PDDR |= (1 << 5);		 // Thiet lap la output
	
		// Thiet lap led do
		SIM->SCGC5 |= (1 << 13);   // Cap clock cho Port E
		PORTE->PCR[29] = (1 << 8);  // Thiet lap chan 29 la GPIO
		PTE->PDDR |= (1 << 29);		 // Thiet lap la output
}

// khoi tao 2 switch
void initSwitch(void)
{
	// PortC pin3, input pullup - khoi tao sw1 va 2
	SIM->SCGC5 |= (1 << 11); // Cap clock cho sw1
	PORTC->PCR[Sw1_pin] = (1 << 8) | (1 << 1) | (1 << 0);  // MUX(1) | PE | PS - kich hoat theo suon xung xuong sw1
	PTC->PDDR &= ~((uint32_t)(1u<< Sw1_pin)); //  - sw1 output
	PORTC->PCR[Sw2_pin] = (1 << 8) | (1 << 1) | (1 << 0);  // MUX(1) | PE | PS - kich hoat theo suon xung xuong sw2
	PTC->PDDR &= ~((uint32_t)(1u<< Sw2_pin)); //  - sw2 output
	
	// cho phep interupt cua switch
	PORTC->PCR[Sw2_pin] |= PORT_PCR_IRQC(0xA); // thiet lap ngat suon xuong cho PORTC pin3
	PORTC->PCR[Sw1_pin] |= PORT_PCR_IRQC(0xA); // thiet lap ngat suon xuong cho PORTC pin3
	NVIC_ClearPendingIRQ(PORTC_D_IRQ_NBR); // xoa cac interupt request dang o C va D
	NVIC_EnableIRQ(PORTC_D_IRQ_NBR); // cho phep interupt request o C va D
}
//init Systick
void init_systick_interupt(void) {
    SysTick->LOAD = 24000000 - 1;  // Set reload register
    SysTick->VAL = 0;                      // Reset the SysTick counter value
    SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0);  // Enable SysTick, enable interrupt, use processor clock
}

// SysTick Handler - cho den chay luan phien
void SysTick_Handler(void) {
	msTicks++;
  if (system_active == 1) {
			RED_LED_OFF();
			GREEN_LED_TOGGLE(); // Toggle green LED
			for (int i = 0; i < 24000; i++);
  } else {
			GREEN_LED_OFF();
      RED_LED_TOGGLE();   // Toggle red LED
  }
}

// Hàm reset trang thai
void reset_system_state(void) {
    system_active = 1; // dat system dang hoat dong
    GREEN_LED_OFF(); // Tt dèn LED xanh
    RED_LED_OFF(); // tat den LED do
	
    // Kh?i t?o l?i các bi?n, thi?t b? ngo?i vi, v.v. tùy theo yêu c?u
	
}
	
// PORTC interrupt handler for switches
/*void PORTC_PORTD_IRQHandler(void) {
	for(int i = 0; i < 1000; i++){};
  if (PORTC->ISFR & (1U << Sw1_pin)) {
        system_active = !system_active;
        PORTC->ISFR = (1U << Sw1_pin); // xoa co
    }
    if (PORTC->ISFR & (1U << Sw2_pin)) {
        reset_system_state(); // ham reset trang thai
        PORTC->ISFR = (1U << Sw2_pin); // xoa co
    }
}*/

void SetSysTickPriority(uint32_t priority) {
    NVIC_SetPriority(SysTick_IRQn, priority);
}

void SetGPIOPriority(uint32_t priority) {
    NVIC_SetPriority(PORTC_PORTD_IRQn, priority);
}