#include "msp.h"
#include <stdio.h>

void delay_ms (uint32_t delay);
void UART0_init(void);
unsigned char UART0Rx(void);
int UART0Tx(unsigned char c);
int Analog[] = {0, 0, 0};

int main (void) {
    UART0_init();
    //////////configure ADC14////////
    ADC14->CTL0 = 0x00000010;       //power on and disabled during configuration
    ADC14->CTL0 |= 0x04180300;      //S/H mode, mclk, 32 sample clock 0x04180300
        ADC14->CTL0 |= 2;               //enable ADC after configuration
    ADC14->CTL1 = 0x00000020;       //12bits resolution
        ADC14->CTL1 |= 0x00050000;      //convert for mem reg 5
    while (1) {
                ADC14->CTL0 = 0x00000010;       //power on and disabled during configuration
                ADC14->CTL0 |= 0x04180300;      //S/H mode, mclk, 32 sample clock 0x04180300
                ADC14->CTL1 = 0x00000020;       //12bits resolution
        ADC14->MCTL[5] = 13;            //A6 input, sigle-ended, Vref =AVCC
        P4->SEL1 &= 0x01;               //configure P4.7 for A6
        P4->SEL0 &= 0x01;
        ADC14->CTL1 |= 0x00050000;    //convert for mem reg 5
        ADC14->CTL0 |= 2;             //enable ADC after configuration
        ADC14->CTL0 |= 1;             //start aconversion
        while (!(ADC14->IFGR0));        //wait till conversion complete
        Analog[0] = ADC14-> MEM[5];     //read ADC result
                
                ADC14->CTL0 = 0x00000010;       //power on and disabled during configuration
                ADC14->CTL0 |= 0x04180300;      //S/H mode, mclk, 32 sample clock 0x04180300
                ADC14->CTL1 = 0x00000020;       //12bits resolution
        ADC14->MCTL[5] = 14;            //A6 input, sigle-ended, Vref =AVCC
        P6->SEL1 &= 0x02;               //configure P4.7 for A6
        P6->SEL0 &= 0x02;
        ADC14->CTL1 |= 0x00050000;      //convert for mem reg 5
        ADC14->CTL0 |= 2;               //enable ADC after configuration
        ADC14->CTL0 |= 1;               //start aconversion
        while (!(ADC14->IFGR0));        //wait till conversion complete
        Analog[1] = ADC14-> MEM[5];     //read ADC result
                
                ADC14->CTL0 = 0x00000010;       //power on and disabled during configuration
                ADC14->CTL0 |= 0x04180300;      //S/H mode, mclk, 32 sample clock 0x04180300
                ADC14->CTL1 = 0x00000020;       //12bits resolution
        ADC14->MCTL[5] = 15;            //A6 input, sigle-ended, Vref =AVCC
        P6->SEL1 &= 0x01;               //configure P4.7 for A6
        P6->SEL0 &= 0x01;
        ADC14->CTL1 |= 0x00050000;      //convert for mem reg 5
                ADC14->CTL0 |= 2;               //enable ADC after configuration
        ADC14->CTL0 |= 1;               //start aconversion
        while (!(ADC14->IFGR0));        //wait till conversion complete
        Analog[2] = ADC14-> MEM[5];     //read ADC result
                ADC14->CTL0 &= ~1;              //disable aconversion
        
                for (int i = 0; i < 3; i++) {
            printf("Input voltage %d : %d\r\n", i+1, Analog[i] * 330 / 4096);    
        }
        printf("-----------------\n");
        delay_ms(1000);
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

/////////////////////////////UART0Rx///////////////////////////////////
/* read a charater from UART */
unsigned char UART0Rx(void) {
    char c;
    while (!(EUSCI_A0 -> IFG & 0x01));
    c = EUSCI_A0 -> RXBUF;
    return c;}
    /////////////////////////////UART0Tx///////////////////////////////////
    /* write a charater to UART */
    int UART0Tx (unsigned char c){
    while (!(EUSCI_A0 -> IFG & 0x02));
    EUSCI_A0 -> TXBUF = c;
    return c;
}

//////////////////////////////////////////////////////////////////////
/* The code below is the interface to the C standard I/O library
All the I/O are directed to the console, which is UART0.*/
struct __FILE { int handle;};
    FILE __stdin = {0};
    FILE __stdout = {1};
    FILE __stderr = {2};
    /*Called by by C library console/file input This function echoes the character received.
    If the character is '\r', it is substituted by '\n'. */
    int fgetc(FILE *f) {
    int c;
    c = UART0Rx(); // read the character from console
    if (c == '\r') { //if '\r' replace with '\n'
    UART0Tx(c); // echo
    c = '\n'; }
    UART0Tx(c); // echo
    return c;
}

//Called by C library console/file output
int fputc (int c, FILE *f) {
    return UART0Tx(c);
}//write the character to console

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
