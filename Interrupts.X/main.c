/*
 * File:   main.c
 * Author: eppa1
 *
 * Created on 11 ottobre 2022, 11.18
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

#define FOSC 7372800L //Hz
#define FCY FOSC/4
#define PR_MAX 65535
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define TIMER5 5


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
            TMR1 = 0; // reset the timer
            IFS0bits.T1IF = 0; // lower the flag
            
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

            T1CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER2:
            TMR2 = 0; // reset the timer
            IFS0bits.T2IF = 0; // lower the flag
            
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

            T2CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER3:
            TMR3 = 0; // reset the timer
            IFS0bits.T3IF = 0; // lower the flag
            
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

            T3CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER4:
            TMR4 = 0; // reset the timer
            IFS1bits.T4IF = 0; // lower the flag
            
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

            T4CONbits.TON = 1;  // starts the timer!
            break;
            
        case TIMER5:
            TMR5 = 0; // reset the timer
            IFS1bits.T5IF = 0; // lower the flag
            
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
            
            T5CONbits.TON = 1;  // starts the timer!
            break;
        default:
            // error
            break;
    }
   
}

void tmr_wait_period(int timer){
    //The second function should use the timer flag to wait until it has expired.
    
    switch (timer) {
        case TIMER1:
            while (IFS0bits.T1IF == 0){
                // Do nothing
            }
    
            IFS0bits.T1IF = 0;
            //TMR1 = 0; // if the flag is raised, the TMR is automatically reset
            break;
        case TIMER2:
            while (IFS0bits.T2IF == 0){
                // Do nothing
            }
    
            IFS0bits.T2IF = 0;
            //TMR2 = 0;
            break;
        case TIMER3:
            while (IFS0bits.T3IF == 0){
                // Do nothing
            }
    
            IFS0bits.T3IF = 0;
            //TMR3 = 0;
            break;
        case TIMER4:
            while (IFS1bits.T4IF == 0){
                // Do nothing
            }
    
            IFS1bits.T4IF = 0;
            //TMR4 = 0;
            break;
        case TIMER5:
            while (IFS1bits.T5IF == 0){
                // Do nothing
            }
    
            IFS1bits.T5IF = 0;
            //TMR5 = 0;
            break;
        default:
            // error
            break;
    }
}

void tmr_wait_ms(int timer,int ms){
    tmr_setup_period(timer,ms);
    tmr_wait_period(timer);
}

//  1st EXERCISE
//void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(){ // Interrupt handler -> event: Timer2 elapsing
//    IFS0bits.T2IF = 0; //reset interrupt flag
//    LATBbits.LATB1 = !LATBbits.LATB1;     //change the state of the D4 led
//}

//  2nd EXERCISE
//void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
//    IFS0bits.INT0IF = 0; //reset interrupt flag
//    LATBbits.LATB1 = !LATBbits.LATB1;     //change the state of the D4 led
//}

//  3rd EXERCISE
//void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
//    IFS0bits.INT0IF = 0; //reset interrupt flag
//    IEC0bits.INT0IE = 0;
//    LATBbits.LATB1 = !LATBbits.LATB1;     //change the state of the D4 led
//}

//  3rd EXERCISE bis
//void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
//    IFS0bits.INT0IF = 0; //reset interrupt flag
//
//    if (IFS0bits.T2IF == 1){
//        LATBbits.LATB1 = !LATBbits.LATB1;     //change the state of the D4 led
//        IFS0bits.T2IF = 0;  // reset the timer2 flag
//        TMR2 = 0;   // empty the timer2 'counter' register
//    }
//}

//  3rd EXERCISE tris

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
        LATBbits.LATB1 = !LATBbits.LATB1;     //change the state of the D4 led
    }
    
    IEC0bits.T2IE = 0;                        // disable T2 interrupt (TIMER2)    
}

int main(void) {
    
//    1st EXERCISE
    
//    TRISBbits.TRISB0 = 0;   //led D3 as output
//    TRISBbits.TRISB1 = 0;   //led D4 as output
//    IEC0bits.T2IE = 1;   // enable T2 interrupt
//    
//    tmr_setup_period(TIMER1,500);
//    tmr_setup_period(TIMER2,250);
//    
//    while (1){
//        LATBbits.LATB0 = !LATBbits.LATB0;     //change the state of the D3 led
//        tmr_wait_period(TIMER1);
//    }
    
//    2nd EXERCISE
    
//    TRISBbits.TRISB0 = 0;   //led D3 as output
//    TRISBbits.TRISB1 = 0;   //led D4 as output
//    TRISEbits.TRISE8 = 1; // S5 button as input
//    IEC0bits.INT0IE = 1;   // enable INT0 interrupt
//    
//    tmr_setup_period(TIMER1,10);
//    
//    while (1){
//        LATBbits.LATB0 = !LATBbits.LATB0;     //change the state of the D3 led
//        tmr_wait_period(TIMER1);
//    }
    // gestione bouncing del LED
    
//    3rd EXERCISE 
//    wrong solution because if the main spins at 500ms I cannot reduce the period. 
//    As a matter of fact the code of the main may last around 450 ms and therefore if I reduce the 
//    period I also have to framment the code in order to comply with the reduced period
    
//    int count_ISR = 0; // counter to count the loops to wait after the ISR to avoid bounce
//    int count_loops = 0; // counter to count the loops to turn on the D4 led after 500 ms
//    TRISBbits.TRISB0 = 0;   //led D3 as output
//    TRISBbits.TRISB1 = 0;   //led D4 as output
//    TRISEbits.TRISE8 = 1; // S5 button as input
//    IEC0bits.INT0IE = 1;   // enable INT0 interrupt
//    IFS0bits.INT0IF = 0; //reset interrupt flag
//    
//    tmr_setup_period(TIMER1,10);
//    
//    while (1){
//        count_loops++;
//        if(IEC0bits.INT0IE == 0){
//            count_ISR++;
//        }
//        if(count_ISR == 15){
//            IFS0bits.INT0IF = 0; //reset interrupt flag because even if I don't consider the transitions while the interrupt is disabled, the interrupt flag still goes to 1
//            IEC0bits.INT0IE = 1;   // enable INT0 interrupt
//            count_ISR = 0;
//        }
//        if(count_loops == 50){ // if 500 ms have passed
//         LATBbits.LATB0 = !LATBbits.LATB0;     //change the state of the D3 led
//         count_loops = 0;
//        }
//        tmr_wait_period(TIMER1);
//    }
    
//     3rd EXERCISE bis 
//     wrong solution because if I keep pressed the button more than 150 ms and at the beginning
//     an edge occurred, making the state change, I will necessarily change state again because 
//     the timer will be elapsed after I release the buttom
    
//    TRISBbits.TRISB0 = 0;   //led D3 as output
//    TRISBbits.TRISB1 = 0;   //led D4 as output
//    TRISEbits.TRISE8 = 1; // S5 button as input
//    IEC0bits.INT0IE = 1;   // enable INT0 interrupt
//    IFS0bits.INT0IF = 0; //reset interrupt flag
//    IFS0bits.T2IF = 1; //set the timer2 flag to 1 to catch the first press
//    
//    tmr_setup_period(TIMER1,500);
//    tmr_setup_period(TIMER2,150);
//    
//    while (1){
//        
//        LATBbits.LATB0 = !LATBbits.LATB0;     //change the state of the D3 led
//        
//        tmr_wait_period(TIMER1);
//    }
    
    
//     3rd EXERCISE tris
    
    TRISBbits.TRISB0 = 0;   //led D3 as output
    TRISBbits.TRISB1 = 0;   //led D4 as output
    TRISEbits.TRISE8 = 1;   // S5 button as input
    
    IEC0bits.INT0IE = 1;    // enable INT0 interrupt (S5)
    IEC0bits.T2IE = 0;      // disable T2 interrupt (TIMER2)
    
    IFS0bits.INT0IF = 0;    //reset interrupt flag (S5)
    IFS0bits.T2IF = 0;      //reset interrupt flag (TIMER2)
    
    tmr_setup_period(TIMER1,500);
    tmr_setup_period(TIMER2,50);
    
    while (1){
        LATBbits.LATB0 = !LATBbits.LATB0;     //change the state of the D3 led
        tmr_wait_period(TIMER1);
    }
}
