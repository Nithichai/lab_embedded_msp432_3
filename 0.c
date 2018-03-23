#include "msp.h"
#include <stdio.h>

void delay_ms (uint32_t delay);
void UART0_init(void);
unsigned char UART0Rx(void);
int UART0Tx(unsigned char c);
int Analog=0;

int main (void) {
    UART0_init();
    //////////configure ADC////////
    ADC14->CTL0 = 0x00000010;       //power on and disabled during configuration
    ADC14->CTL0 |= 0x04180300;      //S/H mode, mclk, 32 sample clock 0x04180300
    ADC14->CTL1 = 0x00000020;       //12bits resolution
    ADC14->MCTL[5]=6;               //A6 input, sigle-ended, Vref =AVCC
    P4-> SEL1 &= 0x80;              //configure P4.7 for A6
    P4-> SEL0 &= 0x80;
    ADC14->CTL1 |= 0x00050000;      //convert for mem reg 5
    ADC14->CTL0 |=2;                //enable ADC after configuration
    while (1) {
        ADC14->CTL0 |=1;                //start aconversion
        while (!(ADC14->IFGR0));        //wait till conversion complete
        Analog = ADC14-> MEM[5];    //read ADC result
        printf("Input voltage : %d\r\n",Analog );
        delay_ms (1000);
    }

}

//////////////////////UART0_init//////////////////////
void UART0_init(void){
    EUSCI_A0->CTLW0 |= 1;       //put in reset mode for config
    EUSCI_A0->MCTLW = 0;        //disable oversampling
    EUSCI_A0->CTLW0 = 0x0081;   //1 stop bit, parity non, SMCK, 8 bits data
    EUSCI_A0->BRW = 26;         //baud rate 115200 (3000000/115200 = 26)
    P1->SEL0 |= 0x0C;           //P1.3 and P1.2 for UART
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1;      //take UART out of reset mode
}

////////////////////// UART0Rx //////////////////////
unsigned char UART0Rx(void) {
    char c;
    while (!(EUSCI_A0 -> IFG & 0x01));
        c = EUSCI_A0 -> RXBUF;
        return c;
    }
}

////////////////////// UART0Tx //////////////////////
int UART0Tx (unsigned char c) {
    while (!(EUSCI_A0 -> IFG & 0x02));
    EUSCI_A0 -> TXBUF = c;
    // return c;
}

//////////////////////delay_ms//////////////////////
void delay_ms(uint32_t delay){
    uint32_t i;
    SysTick->LOAD = 3000-1;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x00000005;
    for(i=0; i<delay; i++){
        while((SysTick -> CTRL & 0x00010000) == 0){}
    }
    SysTick->CTRL=0;
}