//Amaiam Ul Haque
//March 31, 2025
//Spatial Scanner - main file



//HEADER FILES --------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"



//GLOBAL CONSTANTS --------------------------------------------------------------------------------------------------------
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

const int32_t STEP = 4; //the increment of steps (out of 512) taken after each measurement


//GLOBAL VARIABLES -----------------------------------------------------------------------------------------------------------
uint32_t currentPos = 0;	//current position in steps
uint32_t direction = 1;		//direction toggle
//initial states of status outputs
uint32_t state0 = 0;

volatile unsigned long FallingEdges = 0;

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

uint8_t byteData, byteData1, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
uint16_t wordData;
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;
uint8_t dataReady;




//FUNCTION PROTOTYPES -------------------------------------------------------------------------------------------------------
void PortN_Init(); //on board LEDs
void PortF_Init(); //on board LEDs
void PortJ_Init(); //on board push button
void PortH_Init(); //stepper motor
void PortG_Init(); //ToF sensor

void EnableInt();
void DisableInt();
void WaitForInt();

void ToF_Init();
void I2C_Init(void);
void VL53L1X_XSHUT(void);

void spinCW();
void spinCCW();
void rotate(uint32_t steps, uint32_t dir);

void updateCurrPos(uint32_t stepsTaken);
void returnHome();

void statusOutput0 (uint32_t state); //PN1 <-- measurement status
void statusOutput1 (uint32_t state); //PN0
void statusOutput2 (uint32_t state); //PF4 <-- UART Tx
void statusOutput3 (uint32_t state); //PF0
void clearAllStatusOutputs();

void toggleDirection();

void getData();





//FUNCTION DEFINITIONS ------------------------------------------------------------------------------------------------------
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           												// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          											// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																	// ready?
	GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
	GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only
	GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
	//GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB
																																					// 6) configure PB2,3 as I2C
	//GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
	I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
	I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
	//I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
}
 
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
	//Use PortG0
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;             // activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
	GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0
  return;
}
 
//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;           // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;		//PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;          // make PG0 input (HiZ)
}
 


void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};				// Allow time for clock to stabilize
		
	GPIO_PORTN_DIR_R=0b00000011;															// Enable PN0 and PN1 as outputs													
	GPIO_PORTN_DEN_R=0b00000011;															// Enable PN0 and PN1 as digital pins
	return;
}

//initialise port F for status output 2 & 3 (D3 = PF4 & D4 = PF0) as output
void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 	// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};					// Allow time for clock to stabilize
		
	GPIO_PORTF_DIR_R=0b00010011;															// Enable PF0-1 and PF4 as outputs
	GPIO_PORTF_DEN_R=0b00010011;															// Enable PF0-1 and PF4 as digital pins
	return;
}
	
//initialise port H for stepper motor (PH0-3) as output
void PortH_Init(void){
	//Use PortH pins (PH0-3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;						// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};		// allow time for clock to stabilize --> two clock cycles (same as x2 NOP)
	
	GPIO_PORTH_DIR_R |= 0x0F;        										// configure Port H pins (PH0-3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     										// disable alt funct on Port H pins (PH0-3)
  GPIO_PORTH_DEN_R |= 0x0F;        										// enable digital I/O on Port H pins (PH0-3)
																											// configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;     										// disable analog functionality on Port H pins (PH0-3)
	return;
}

//initialise port J for buttons 0 & 1 (B0 = PJ0 & B3 = PJ1) as input
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;  // Activate clock for Port J
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {}; // Allow time for clock to stabilize
	GPIO_PORTJ_DIR_R &= ~0x03;                 // Make PJ0 input 
	GPIO_PORTJ_DEN_R |= 0x03;                  // Enable digital I/O on PJ0
	GPIO_PORTJ_PCTL_R &= ~0x0000000F;          // Configure PJ0 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;               // Disable analog functionality on PJ0        
	GPIO_PORTJ_PUR_R |= 0x03;                  // Enable weak pull-up resistor on PJ0
	return;
}

void EnableInt(void) {
    __asm("    cpsie   i\n");
}
 
// Disable interrupts
void DisableInt(void) {
    __asm("    cpsid   i\n");
}
 
// Low power wait
void WaitForInt(void) {
    __asm("    wfi\n");
}



//motor spins CW taking 4 steps at a time
void spinCW(){
	uint32_t delay = 1;
	
	GPIO_PORTH_DATA_R = 0b00000011;
	SysTick_WaitMinimum(delay);
	
	GPIO_PORTH_DATA_R = 0b00000110; 
	SysTick_WaitMinimum(delay);
	
	GPIO_PORTH_DATA_R = 0b00001100; 
	SysTick_WaitMinimum(delay);
	
	GPIO_PORTH_DATA_R = 0b00001001; 
	SysTick_WaitMinimum(delay);
	
	updateCurrPos(4);
	return;
}



//motor spins CCW taking 4 steps at a time
void spinCCW(){
	uint32_t delay = 1;
	
	GPIO_PORTH_DATA_R = 0b00001001;
	SysTick_WaitMinimum(delay);
	
	GPIO_PORTH_DATA_R = 0b00001100; 
	SysTick_WaitMinimum(delay);
	
	GPIO_PORTH_DATA_R = 0b00000110; 
	SysTick_WaitMinimum(delay);
	
	GPIO_PORTH_DATA_R = 0b00000011;
	SysTick_WaitMinimum(delay);
	
	updateCurrPos(-4);
	return;
}



//actual STEP = 4*steps
//if dir == 1 --> CW
//if dir == 0 --> CCw
void rotate(uint32_t steps, uint32_t dir){ 
	//updates currentPos automatically
	if (dir == 1){
		for (uint32_t i=0; i<steps; i++){
			spinCW();
		}
		statusOutput3(1);
		SysTick_Wait10ms(1);
		statusOutput3(0);
	}
	else if(dir == 0){
		for (uint32_t i=0; i<steps; i++){
			spinCCW();
		}
		statusOutput3(1);
		SysTick_Wait10ms(1);
		statusOutput3(0);
	}
}



//FOR ALL STATUS OUTPUT FUNCTION CALLS 
// if state == 1 --> status output set
// else --> status output clear
void statusOutput0 (uint32_t state) {
	if (state == 1)
		GPIO_PORTN_DATA_R |= 0b00000010; //D1 on 
	else
		GPIO_PORTN_DATA_R &= 0b11111101; //D1 off
	return;
}

void statusOutput1 (uint32_t state) {
	if (state == 1)
		GPIO_PORTN_DATA_R |= 0b00000001; //D2 on
	else 
		GPIO_PORTN_DATA_R &= 0b11111110; //D2 off
	return;
}

void statusOutput2 (uint32_t state) {
	if (state == 1)
		GPIO_PORTF_DATA_R |= 0b00010000; //D3 on
	else
		GPIO_PORTF_DATA_R &= 0b11101111; //D3 off
	return;
}

void statusOutput3 (uint32_t state) {
	if (state == 1)
		GPIO_PORTF_DATA_R |= 0b00000001; //D4
	else
		GPIO_PORTF_DATA_R &= 0b11111110; //D3
	
	return;
}

void clearAllStatusOutputs(){
	GPIO_PORTF_DATA_R = 0b00000000;
	GPIO_PORTN_DATA_R = 0b00000000;
	return;
}

//updates currentPos based on stepsTaken
void updateCurrPos(uint32_t stepsTaken){
	currentPos = (currentPos + stepsTaken) % 2048; //mod operator for if currentPos goes over 2048 CW direction
	
	if (currentPos < 0){ //for if currentPos goes below 0 from CCW direction
		currentPos += 2048;
	}
	return;
}

void returnHome(){
	if (currentPos == 0) 
		currentPos = 2047;
	rotate((currentPos)/4, 0);
	SysTick_Wait10ms(100);
	//currentPos automatically reset back to 0 within spinCW and spinCCW functions
	return;
}


void ToF_Init(){
		UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);
 
/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);
 
	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }

	FlashAllLEDs();

	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
  /* This function must to be called to initialize the sensor with the default setting  */

  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
 

  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */

	status = VL53L1X_SetDistanceMode(dev, 2); // 1=short, 2=long
	
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */
 
  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
 
}

void getData(){
	//GET DATA ---------------------------------------------------------------------------------------------
	status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	for(int i = 0; i < 512; i++) { //run 1 revolution
	
		if (i%STEP == 0){ //checks if increment of angle*
		
			//wait until the ToF sensor's data is ready
			while (dataReady == 0){ //wait until the ToF sensor's data is ready
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
				VL53L1_WaitMs(dev, 5);
			}
			
			statusOutput0(1); //start of measurement
			
			dataReady = 0;
			//read the data values from ToF sensor
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus); 	//0 = valid, 1=signal low, 2=singal high, 3 = out of range
			status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
			status = VL53L1X_GetSignalRate(dev, &SignalRate); 		//strength of relfected signal
			status = VL53L1X_GetAmbientRate(dev, &AmbientRate); 	//background light interference
			status = VL53L1X_GetSpadNb(dev, &SpadNum); 						//the spatial photon avalanche number

			statusOutput0(0); //end of measurement
			
			statusOutput2(1); //start of UART Tx
			status = VL53L1X_ClearInterrupt(dev); //clear interrupt has to be called to enable next interrupt
			sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum); //print the resulted readings to UART
			UART_printf(printf_buffer);
			SysTick_Wait10ms(5);
			statusOutput2(0); //end of UART Tx
			}
			

		//run stepper motor rotation outside of if statement
		spinCW();
			
	}
	//full revolution completed --> stop ranging and undo spin to keep stepper motor wires from tangling
	VL53L1X_StopRanging(dev);
	returnHome();
	currentPos = 0;
	return;
}
// ---------------------------------------------------------------------------------------------------------------------------------- 
// MAIN -----------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------- 


int main(void) {
 
	//initialisations
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	ToF_Init();
	
	PortN_Init(); //on board LEDs
  PortF_Init(); //on board LEDs
  PortJ_Init(); //on board push button
  PortH_Init(); //stepper motor
  PortG_Init(); //ToF sensor
	
	

  while(1) {
		
  	//ACTUAL MAIN PROGRAM DOING THE SCANNING
		//initally off
		while (state0 == 0){
			clearAllStatusOutputs();
			//B0 - PJ0 - TOGGLE POWER
			if ((GPIO_PORTJ_DATA_R &= 0b00000001) == 0){ //once B0 pressed, turn ON
				state0 = 1;
				SysTick_Wait10ms(25); //wait 0.25s
			}
		}
		
		//B0 - PJ0 - TOGGLE POWER
		if ((GPIO_PORTJ_DATA_R &= 0b00000001) == 0){ //once PJ0 pressed, TOGGLE POWER
			state0^=1;
		}
		
		if (state0 == 1){
			statusOutput1(1); // additional status start
			getData();
			
			//THIS IS NECESSARY FOR THE IN PERSON DEMO
			//must scan three times with no disaplacement
			//getData();
			//getData();
			
			statusOutput1(0); // additional status end
			state0 = 0;
		}
	}
}