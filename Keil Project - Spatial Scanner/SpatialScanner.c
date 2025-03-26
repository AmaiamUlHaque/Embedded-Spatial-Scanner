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

const int32_t STEP = 4; //the increment of steps (out of 512) taken after each measurement


//GLOBAL VARIABLES -----------------------------------------------------------------------------------------------------------
uint32_t currentPos = 0;	//current position in steps
uint32_t direction = 1;		//direction toggle
//initial states of status outputs
uint32_t state0 = 0;

//FUNCTION PROTOTYPES -------------------------------------------------------------------------------------------------------
void PortN_Init(); //on board LEDs
void PortF_Init(); //on board LEDs
void PortJ_Init(); //on board push button
void PortH_Init(); //stepper motor

void spinCW();
void spinCCW();
void rotate(uint32_t steps, uint32_t dir);
void toggleDirection();

void updateCurrPos(uint32_t stepsTaken);
void returnHome();


void statusOutput0 (uint32_t state); //PN1 <-- measurement status
void statusOutput1 (uint32_t state); //PN0
void statusOutput2 (uint32_t state); //PF4 <-- UART Tx
void statusOutput3 (uint32_t state); //PF0
void clearAllStatusOutputs();


//FUNCTION DEFINITIONS ------------------------------------------------------------------------------------------------------
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


// ---------------------------------------------------------------------------------------------------------------------------------- 
// MAIN -----------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------- 


int main(void) {
 
	//initialisations
	onboardLEDs_Init();

	PortN_Init(); //on board LEDs
  PortF_Init(); //on board LEDs
	PortH_Init(); //stepper motor
	PortJ_Init(); //on board push button
	
	
	
	//initally off
		while (state0 == 0){
			for (int i = 0; i < 1024; i++)
			spinCW();
			for (int i = 0; i < 1024; i++)
			spinCCW();
			
			rotate(1024, 1);
			toggleDirection();
		}
}