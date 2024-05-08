/* 
 * File:   main.c
 * Author: moame
 *
 * Created on 20 ottobre 2022, 14.56
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
#include <string.h>
#include <stdio.h>

#define FOSC 7372800L   //Hz
#define FCY FOSC/4      //1 843 200 = 1,8432 MHz 
#define PR_MAX 65535
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define TIMER5 5

//The first function setups the timer to count for the specified amount of milliseconds.
void tmr_setup_period(int timer, int ms){
    
    // FCY = 7372800/4 = 1843200/secondo --> diviso 2 -> 921600/500ms
    // per contare 500 ms devo contare 921600 cicli di clock, quindi 912600 deve essere inserito nel PRX Register ma
    // 921600 non ci sta nel 16bit register (max 65535), quindi bisogna rallentare il clock settando il prescaler
    // 921600 / 65535 = 14,06 -> dividiamo per 64 perchè 14,06 non c'è -> 921600/64 = 14259,375 

    double PR_no_prescale = FCY * (ms/1000.0); // [1/s * s = adim.] 1000.0 because I have to cast the division to double
    // double PR_no_prescale = 921600;
    double prescale_needed = PR_no_prescale / PR_MAX;
    
    switch (timer) {
        case TIMER1:
            if (prescale_needed < 1) {
                T1CONbits.TCKPS = 0;
                PR1 = (int) (PR_no_prescale / 1);  // registro che contiene il numero a cui arrivare prima di riazzerare
            } else if (prescale_needed < 8) {
                T1CONbits.TCKPS = 1;
                PR1 = (int) (PR_no_prescale / 8);
            } else if (prescale_needed < 64) {
                T1CONbits.TCKPS = 2;
                PR1 = (int) (PR_no_prescale / 64);
            } else if (prescale_needed < 256) {
                T1CONbits.TCKPS = 3;
                PR1 = (int) (PR_no_prescale / 256);
            }
            TMR1 = 0;           // reset the timer
            T1CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER2:
            if (prescale_needed < 1) {
                T2CONbits.TCKPS = 0;
                PR2 = (int) (PR_no_prescale / 1);  // registro che contiene il numero a cui arrivare prima di riazzerare
            } else if (prescale_needed < 8) {
                T2CONbits.TCKPS = 1;
                PR2 = (int) (PR_no_prescale / 8);
            } else if (prescale_needed < 64) {
                T2CONbits.TCKPS = 2;
                PR2 = (int) (PR_no_prescale / 64);
            } else if (prescale_needed < 256) {
                T2CONbits.TCKPS = 3;
                PR2 = (int) (PR_no_prescale / 256);
            }
            TMR2 = 0; // reset the timer
            T2CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER3:
            if (prescale_needed < 1) {
                T3CONbits.TCKPS = 0;
                PR3 = (int) (PR_no_prescale / 1);  // registro che contiene il numero a cui arrivare prima di riazzerare
            } else if (prescale_needed < 8) {
                T3CONbits.TCKPS = 1;
                PR3 = (int) (PR_no_prescale / 8);
            } else if (prescale_needed < 64) {
                T3CONbits.TCKPS = 2;
                PR3 = (int) (PR_no_prescale / 64);
            } else if (prescale_needed < 256) {
                T3CONbits.TCKPS = 3;
                PR3 = (int) (PR_no_prescale / 256);
            }
            TMR3 = 0; // reset the timer
            T3CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER4:
            if (prescale_needed < 1) {
                T4CONbits.TCKPS = 0;
                PR4 = (int) (PR_no_prescale / 1);  // registro che contiene il numero a cui arrivare prima di riazzerare
            } else if (prescale_needed < 8) {
                T4CONbits.TCKPS = 1;
                PR4 = (int) (PR_no_prescale / 8);
            } else if (prescale_needed < 64) {
                T4CONbits.TCKPS = 2;
                PR4 = (int) (PR_no_prescale / 64);
            } else if (prescale_needed < 256) {
                T4CONbits.TCKPS = 3;
                PR4 = (int) (PR_no_prescale / 256);
            }
            TMR4 = 0; // reset the timer
            T4CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER5:
            if (prescale_needed < 1) {
                T5CONbits.TCKPS = 0;
                PR5 = (int) (PR_no_prescale / 1);  // registro che contiene il numero a cui arrivare prima di riazzerare
            } else if (prescale_needed < 8) {
                T5CONbits.TCKPS = 1;
                PR5 = (int) (PR_no_prescale / 8);
            } else if (prescale_needed < 64) {
                T5CONbits.TCKPS = 2;
                PR5 = (int) (PR_no_prescale / 64);
            } else if (prescale_needed < 256) {
                T5CONbits.TCKPS = 3;
                PR5 = (int) (PR_no_prescale / 256);
            }            
            TMR5 = 0; // reset the timer
            T5CONbits.TON = 1;  // starts the timer!
            break;
            
        default:
            // error
            break;
    }
}

//The second function should use the timer flag to wait until it has expired.
void tmr_wait_period(int timer){
    
    switch (timer) {
        case TIMER1:
            while (IFS0bits.T1IF == 0){
                // Do nothing
            }
            IFS0bits.T1IF = 0; //RESETTO IL FLAG
            break;
            
        case TIMER2:
            while (IFS0bits.T2IF == 0){
                // Do nothing
            }
            IFS0bits.T2IF = 0;
            break;
            
        case TIMER3:
            while (IFS0bits.T3IF == 0){
                // Do nothing
            }
            IFS0bits.T3IF = 0;
            break;
            
        case TIMER4:
            while (IFS1bits.T4IF == 0){
                // Do nothing
            }
            IFS1bits.T4IF = 0;
            break;
            
        case TIMER5:
            while (IFS1bits.T5IF == 0){
                // Do nothing
            }
            IFS1bits.T5IF = 0;
            break;

        default:
            // error
            break;
    }
}

//The third function should realize a simple delay when called
void tmr_wait_ms(int timer, int ms){
    
    tmr_setup_period(timer,ms);
    tmr_wait_period(timer); 

}

int choice = 0;
char lgbtq_string[20];
int seconds_elapsed = 0;

void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
    IFS0bits.INT0IF = 0; //reset interrupt flag
    seconds_elapsed = 0; //reset the seconds counter
    
    // restart the timer
    IFS0bits.T1IF = 0;       // reset the timer1 flag
    TMR1 = 0;                // empty the timer1 'counter' register  
    
    choice++;
    if (choice%4 == 0)
        lgbtq_string = "FraFerraz Gay";
    else if (choice%4 == 1 )
        lgbtq_string = "EmaCulo Laziale";
    else if (choice%4 == 2 )
        lgbtq_string = "SalvoZio Satrapo";
    else if (choice%4 == 3 )
        lgbtq_string = "MoHHammad Trans";
    
    if (choice == 4)
        choice = 0;

}

int main(int argc, char** argv) {

    TRISEbits.TRISE8 = 1;   // S5 button as input
    IEC0bits.INT0IE = 1;    // enable INT0 interrupt (S5)
    
    tmr_wait_ms(TIMER1, 1000); //after any reset operation, wait 1000ms
    
    SPI1CONbits.MSTEN = 1;  // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3;   // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6;   // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI

    // 1.   Write ?HELLO WORLD? on the LCD display
//    char* string = "Hello world!";
//    for (int i=0; i<12; i++){
//        while(SPI1STATbits.SPITBF == 1); // wait until not full
//        SPI1BUF = string[i]; // send the byte containing the value 22
//    }
    
    //    2. Set a timer to expire every second; write the seconds elapsed on
    //the LCD (use sprintf(buffer, ?%d?, value) to convert an integer to
    //a string to be displayed
    
    char timer_string[100];
    tmr_setup_period(TIMER1,1000);
    
    lgbtq_string = "FraFerraz Gay";
    
    while(1){
        
        //FIRST ROW
        sprintf(timer_string, "Time = %d", seconds_elapsed);
        while(SPI1STATbits.SPITBF == 1); // wait until not full (this bits indicate if the register is not empty)
        SPI1BUF = 0x80;
        for (int i=0; i<16; i++){
            while(SPI1STATbits.SPITBF == 1); // wait until fully sent
            SPI1BUF = ' '; // send the char byte
        }
        for (int i=0; i<strlen(timer_string); i++){
            while(SPI1STATbits.SPITBF == 1); // wait until fully sent
            SPI1BUF = timer_string[i]; // send the char byte
        }
        
        //SECOND ROW
        while(SPI1STATbits.SPITBF == 1); // wait until not full (this bits indicate if the register is not empty)
        SPI1BUF = 0xC0;
        for (int i=0; i<16; i++){
            while(SPI1STATbits.SPITBF == 1); // wait until fully sent
            SPI1BUF = ' '; // send the char byte
        }
        for (int i=0; i<strlen(lgbtq_string); i++){
            while(SPI1STATbits.SPITBF == 1); // wait until fully sent
            SPI1BUF = lgbtq_string[i]; // send the char byte
        }
        seconds_elapsed++;
        tmr_wait_period(TIMER1);
    }
    
    while(1);
    return (0);
}

