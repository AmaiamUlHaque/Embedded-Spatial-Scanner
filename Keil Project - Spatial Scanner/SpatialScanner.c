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


//GLOBAL VARIABLES -----------------------------------------------------------------------------------------------------------
//initial states of status outputs
uint32_t state0 = 0;

//FUNCTION PROTOTYPES -------------------------------------------------------------------------------------------------------
void PortN_Init(); //on board LEDs
void PortF_Init(); //on board LEDs
void PortJ_Init(); //on board push button

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
	PortJ_Init(); //on board push button
	
	
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
			
			statusOutput1(0); // additional status end
			state0 = 0;
		}
}