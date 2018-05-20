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

    NVIC_EnableIRQ(PORT1_IRQn); // interrupt pin 2

    /* NOTE: p10.1 is not supported for interrupts. You will have to do polling. which defeats the
     * entire purpose.
    */
}


void writeRegister(uint8_t address, uint8_t reg, uint8_t value){
    //while(UCB3STATW & UCBBUSY); // wait while busy
    while(UCB3CTLW0 & UCTXSTT); //soon as this is no longer true send the next thing
    UCB3I2CSA = address;

    UCB3CTL0 |= UCTR | UCTXSTT;

    while(UCB3CTLW0 & UCTXSTT); //soon as this is no longer true send the next thing
    UCB3TXBUF = reg;
    while(!(UCB3IFG & UCTXIFG0)); //when empty send next thing

    UCB3TXBUF = value;
    while(!(UCB3IFG & UCTXIFG0)); //when empty send next thing
    UCB3CTL0 |= UCTXSTP;
}


void readSingle(uint8_t address, uint8_t reg, uint8_t* result){
    while(UCB3STAT & UCBBUSY); //wait if busy
       UCB3I2CSA = address;
       UCB3CTL0 |= UCTR | UCTXSTT;

    while(UCB3CTLW0 & UCTXSTT);
       UCB3TXBUF = reg;
    while(!(UCB3IFG & UCTXIFG0)); //wait until buf is transmitted

       /* read and repeat start */
       UCB3CTL0 &= ~UCTR;
       UCB3CTL0 |= UCTXSTT;

       while(UCB3CTLW0 & UCTXSTT);
     //  while(!(UCB3IFG & UCRXIFG0));
       UCB3CTL0 |= UCTXSTP;
       /* collect the bytes requested, stuff into array, then send stop*/
       uint8_t rxValue = UCB3RXBUF;
       while(UCB3CTLW0 & UCTXSTP); //clear after stop been set and come out.

        *result = rxValue; //remember pass an address, deref and store value.
}

void readMultiple(uint8_t address, uint8_t reg, uint8_t numBytes,uint8_t* result){
    while(UCB3STAT & UCBBUSY); //wait if busy
    UCB3I2CSA = address;

}


void beginTransmission(uint8_t address){
    UCB3I2CSA = address;
    UCB3CTL0 |= UCTXSTT;
}


void stopTransmission(void){
    EUSCI_B3->CTLW0 |= UCTXSTP; //send stop command;
}
