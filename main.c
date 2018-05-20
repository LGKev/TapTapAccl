#include "msp.h"
#include "BMA280.h"


/**
 * main.c
 */
void initHBLed(void){
    P9SEL0 &= ~BIT3;
    P9SEL1 &= ~BIT3;
    P9DIR |= BIT3;
    P9OUT |= BIT3;
}

void beatHeart(uint32_t duration){

    P9OUT|=BIT3;
    uint32_t duration_mS = duration*300; // .1 seconds, like 1 is .1 seconds, 10 is 1 second
    uint32_t i = 0;
    for(i=0; i<duration_mS; i++);
    P9OUT &= ~BIT3;
    for(i=0; i<duration_mS; i++);

}

/* really useful for checking for an ACK */
uint8_t SingleWrite(uint8_t address, uint8_t registerToWrite, uint8_t valueToWrite){
    beginTransmission(address);
    UCB3CTL0 |= UCTR;
    UCB3TXBUF = valueToWrite;
    stopTransmission();
//interrupt will trigger and then we will do a stop condition.
    return 1;
}


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    __enable_interrupt(); //globals

	initBMADriver();
	initHBLed();

	uint8_t firstRead =0;
	while(1){
	    writeRegister(BMA_DEAULT_ADDR, 0x3E, 0x00); //default value write to Fifo mode
	    //lower level ones like ACCL_X is Read Only (Ro) I just want an ACK.

    readRegister(BMA_DEAULT_ADDR, 0x00,  &firstRead);
	    beatHeart(100);
	}
}
