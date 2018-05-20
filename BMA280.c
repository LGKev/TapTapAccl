/*
 * BMA280.c
 *
 *  Created on: May 20, 2018
 *      Author: Kevin Kuwata
 *		
 */


#include "msp.h"
#include "bma280.h"

void initBMADriver(void){
    //Configure the GPIO and Interrupt Pins
    P1SEL0 &= ~BIT1; //Interrupt pin 2
    P1SEL1 &= ~BIT1;
    P10SEL0 &= ~BIT1; //Interrupt pin 1
    P10SEL1 &= ~BIT1;

    P1DIR &= ~BIT1; //input
    P10DIR &= ~BIT1; //input

    P1REN |= BIT1;
    P10REN |= BIT1;

    P1OUT |= BIT1;
    P10OUT |= BIT1; //PULLUPS

    //I2C UCB3 DRIVER
    UCB3CTLW0 = UCSWRST; //unlock
    // Master,    i2c,    smclk (3mhz),   WR
    UCB3CTLW0 |= UCMST | UCMODE_3 | UCSSEL__SMCLK | UCTR | UCSYNC; //uscyn always 1 bc spi or i2c only no uart.


      //need to divide 3Mhz down to 100 khz...
    UCB3BR0 |= 30;


    //i2c pins 10.2 (SDA) 10.3 (SCL) //Primary Mode
    P10SEL0 |= BIT2 | BIT3;
    P10SEL1 &= ~(BIT2 | BIT3);

    UCB3CTL0 &= ~UCSWRST; //lock

    /* Polling implementation, Interrupts could be used in the future */
    UCB3IFG = 0; //clear existing interrupts
    // UCB3IE |=  UCNACKIE | UCRXIE0 | UCTXIE; //INTERRUPTS FOR TIMOUT, NACK, RX (BYTE RECIEVED), BYTE COMPLETE TRANSMIT
}


void writeRegister(uint8_t address, uint8_t reg, uint8_t value){

}


void readRegister(uint8_t address, uint8_t reg, uint8_t* result){

}

void beginTransmission(uint8_t address){

}


void stopTransmission(void){

}
