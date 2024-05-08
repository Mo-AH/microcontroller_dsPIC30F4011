/*
 * File:   main.c
 * Author: eppa1
 *
 * Created on 28 novembre 2022, 19.59
 */


// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "timer_functions.h"
#include "spi_functions.h"

#define MAX_TASKS 4

typedef struct{
    int n;
    int N;
    void *param;
    // void (*fptr)(void* p);
    void (*fptr)();
} heartbeat;

typedef struct{
    int current_pos;
    int cursor_pos;
    int start_pos;
    int writing_flag;
} state;

heartbeat schedInfo[MAX_TASKS];
state spi_state;
// char* string = "This is a very long string";
char* string = "Amy Bellitto sei bellissima";
int str_len;
int sliding_motion = 1;

void task0(){ // write on the SPI
    if (spi_state.writing_flag == 1) { // if the end of the LCD row has not been reached yet
        
        place_cursor(FIRST_ROW,spi_state.cursor_pos);
        
        if (spi_state.current_pos <= str_len){ // if the end of the string has not been reached yet
            while(SPI1STATbits.SPITBF == 1);
            SPI1BUF = string[spi_state.current_pos]; // print the current character of the string
            spi_state.current_pos++;
            spi_state.cursor_pos++;
        }
        else { // if the end of the string has been reached instead
            while(SPI1STATbits.SPITBF == 1);
            SPI1BUF = ' '; // print a space
            spi_state.current_pos = (spi_state.current_pos+1) %(str_len+15); // reset the current_pos when str_len + 15 spaces is reached
            spi_state.cursor_pos++;
        }
        
        if (spi_state.cursor_pos >=16 ){ // if the end of the LCD row has been reached/ if I wrote the last position of the row
            spi_state.writing_flag = 0;
        }
    }
}

void task1(){ // update the sliding
    spi_state.cursor_pos = 0;
    // place_cursor(FIRST_ROW,0);
    if (sliding_motion == 1){
        spi_state.start_pos = (spi_state.start_pos+1)%(str_len+15); // consider the subsequent character wrt the one cosnidered at the previous iteration, so as to start writing from there
    }
    spi_state.current_pos = spi_state.start_pos; // start writing from the updated start_pos
    spi_state.writing_flag = 1;
}

void task2(){ // turn on/off the LED
    LATBbits.LATB0 = !LATBbits.LATB0; // change the LED state
}

void task3(){ // convert the signal
    
    int ADCValue;
    double VoltValue;
    unsigned char VoltValueChar;
    
    ADCON1bits.SAMP = 0; // clearing this bit ends sampling and starts converting
    while (ADCON1bits.DONE == 0); // while the conversion is not finished wait
    ADCValue = ADCBUF0; // get ADC value

    VoltValue=(ADCValue*5)/1023.0;
    VoltValueChar = (unsigned char)VoltValue;
    clear_row(SECOND_ROW,3,15);
    print_char_num(SECOND_ROW,3,VoltValueChar);
    
    schedInfo[1].N = 200 - VoltValueChar*30; // the period goes from 1second to 250ms with bounces of 150ms

    ADCON1bits.SAMP = 1; // start sampling
}

void scheduler(){
    // int executed = 0;
    for(int i=0; i<MAX_TASKS;i++){
        schedInfo[i].n++;
        if(schedInfo[i].n >= schedInfo[i].N){
            schedInfo[i].fptr(); // call the function corresponding to the i-th task
            schedInfo[i].n = 0; // reset the loop counter 
        }
    }

}

// INTERRUPT HANDLERS

void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
    IFS0bits.INT0IF = 0; //reset interrupt flag

    // restart the timer
    IFS0bits.T2IF = 0;       // reset the timer2 flag
    TMR2 = 0;                // empty the timer2 'counter' register 
    
    IEC0bits.T2IE = 1;       // enable T2 interrupt (TIMER2)

}

void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(){ // Interrupt handler -> event: Timer2 elapsed
 
    IFS0bits.T2IF = 0;                         //reset Timer2 interrupt flag

    if (PORTEbits.RE8 == 1){
        sliding_motion = !sliding_motion; // change the value of the sliding_motion flag to enable/diable the motion on the LCD
    }
    
    IEC0bits.T2IE = 0;                        // disable T2 interrupt (TIMER2)    
}

int main(void) {
    
    tmr_wait_ms(TIMER1,1000); // wait for the LCD to set-up
    
    // SPI SETUP
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE =6; // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI peripheral
    
    // LED SETUP
    TRISBbits.TRISB0 = 0; // D3 LED as output
    
    // BUTTON SETUP
    TRISEbits.TRISE8 = 1; // S5 button as input
    
    IEC0bits.INT0IE = 1;    // enable INT0 interrupt (S5)
    IFS0bits.INT0IF = 0;    //reset interrupt flag (S5)
    
    // TIMER SETUP
    IEC0bits.T2IE = 0;      // disable T2 interrupt (TIMER2)
    IFS0bits.T2IF = 0;      //reset interrupt flag (TIMER2)

    // ADC SETUP (manual sampling - manual conversion)
    ADCON3bits.ADCS = 8; // selects how long is one Tad: in this case it is 8*Tcy
    // Conversion time duration (Tconv): fixed to 12 Tad
    ADCON1bits.ASAM = 0; // set how the sampling begins as manual
    ADCON1bits.SSRC = 0; // set how the sampling ends and conversion begins as manual
    ADCON2bits.CHPS = 1; // set the number of channels to 1
    ADCHSbits.CH0SA = 2; // choose the positive input to channel 0 - in particular select pin AN2 (0010) where the potentiometer is connected 
    ADPCFG = 0xFFFF; // first set all the bits to 1 
    ADPCFGbits.PCFG2 = 0;  // then set the one corresponding to AN2 to 0 
    // selects which pin should be used for A/D (if the bit is 0 the corresponding pin will be used for ADC)
    ADCON1bits.ADON = 1; // turn on the ADC
    
    // INITIALIZE TASKS
    schedInfo[0].n = 0;
    schedInfo[0].N = 1; // T = 5 ms
    schedInfo[0].fptr = task0;
    
    schedInfo[1].n = 0;
    schedInfo[1].N = 50; // T = 250 ms
    schedInfo[1].fptr = task1;
    
    schedInfo[2].n = 0;
    schedInfo[2].N = 100; // T = 500 ms
    schedInfo[2].fptr = task2;
    
    schedInfo[3].n = 0;
    schedInfo[3].N = 25; // T = 125 ms
    schedInfo[3].fptr = task3;
    
    spi_state.current_pos = 0;
    spi_state.cursor_pos = 0;
    spi_state.start_pos = 0;
    spi_state.writing_flag = 1;
    
    str_len = strlen(string)-1;
    
    // PRINT "V= " IN THE FIRST AND SECOND ROW OF THE LCD
    char *str_v = "V= ";
    place_cursor(SECOND_ROW,0);
    print_string(str_v);
    place_cursor(FIRST_ROW,0);
    
    // START TIMERS
    tmr_setup_period(TIMER2,50); // start the timer2 to avoid S5 bounces
    tmr_setup_period(TIMER1, 5);
    
    ADCON1bits.SAMP = 1; // start sampling
    
    while(1){        
        scheduler();
        tmr_wait_period(TIMER1);
    }
             
    return 0;
}
