/*
 * File:   timer_functions.c
 * Author: Salvatore D'ippolito, Mohammad Al Horany, Francesco Ferrazzi, Emanuele Rambaldi
 *
 * Created on 15 novembre 2022, 11.15
 */

#include "timer_functions.h"

//This function is used to setup a timer's period
void tmr_setup_period(int timer, int ms){
    
    // FCY = 7372800/4 = 1843200 Hz  921600/500ms
    // In 0.5 second there would be 921600 clocks steps
    // This is too high to be put in a 16 bit register (max 65535)
    // If we set a prescaler of 1:64 we have 921600/64 = 14259,375 clock steps
      
    

    double PR_no_prescale = FCY * (ms/1000.0); 
    // [1/s * s = adim.] 1000.0 because I have to cast the division to double

    double prescale_needed = PR_no_prescale / PR_MAX;
    
    switch (timer) {
        case TIMER1:
            TMR1 = 0; // reset the timer
            IFS0bits.T1IF = 0; // lower the flag
            
            if (prescale_needed < 1) {
                T1CONbits.TCKPS = 0;
                PR1 = (int) (PR_no_prescale / 1);  
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

            T1CONbits.TON = 1;  // turn on the timer!
            break;
            
        case TIMER2:
            TMR2 = 0; // reset the timer
            IFS0bits.T2IF = 0; // lower the flag
            
            if (prescale_needed < 1) {
                T2CONbits.TCKPS = 0;
                PR2 = (int) (PR_no_prescale / 1);  
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

            T2CONbits.TON = 1;  // turn on the timer!
            break;
            
        case TIMER3:
            TMR3 = 0; // reset the timer
            IFS0bits.T3IF = 0; // lower the flag
            
            if (prescale_needed < 1) {
                T3CONbits.TCKPS = 0;
                PR3 = (int) (PR_no_prescale / 1); 
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

            T3CONbits.TON = 1;  // turn on the timer!
            break;
            
        case TIMER4:
            TMR4 = 0; // reset the timer
            IFS1bits.T4IF = 0; // lower the flag
            
            if (prescale_needed < 1) {
                T4CONbits.TCKPS = 0;
                PR4 = (int) (PR_no_prescale / 1);
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

            T4CONbits.TON = 1;  // turn on the timer!
            break;
            
        case TIMER5:
            TMR5 = 0; // reset the timer
            IFS1bits.T5IF = 0; // lower the flag
            
            if (prescale_needed < 1) {
                T5CONbits.TCKPS = 0;
                PR5 = (int) (PR_no_prescale / 1); 
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
            
            T5CONbits.TON = 1;  // turn on the timer!
            break;
        default:
            // error
            break;
    }
   
}

//This function waits for a timer's period to expire
void tmr_wait_period(int timer){
        
    switch (timer) {
        case TIMER1:
            while (IFS0bits.T1IF == 0);    
            IFS0bits.T1IF = 0;
            break;
            
        case TIMER2:
            while (IFS0bits.T2IF == 0);
            IFS0bits.T2IF = 0;
            break;
    
        case TIMER3:
            while (IFS0bits.T3IF == 0);
            IFS0bits.T3IF = 0;
            break;
            
        case TIMER4:
            while (IFS1bits.T4IF == 0);
            IFS1bits.T4IF = 0;
            break;
            
        case TIMER5:
            while (IFS1bits.T5IF == 0);
            IFS1bits.T5IF = 0;
            break;
       
        default:
            // error
            break;
    }
}

//This function sets a delay of a given number of milliseconds
void tmr_wait_ms(int timer,int ms){
    tmr_setup_period(timer,ms);
    tmr_wait_period(timer);
    switch (timer) {
        case TIMER1:
            T1CONbits.TON = 0;  // turn off the timer!
            break;
            
        case TIMER2:
            T2CONbits.TON = 0;  // turn off the timer!
            break;
            
        case TIMER3:
            T3CONbits.TON = 0;  // turn off the timer!
            break;
            
        case TIMER4:
            T4CONbits.TON = 0;  // turn off the timer!
            break;
            
        case TIMER5:
            T5CONbits.TON = 0;  // turn off the timer!
            break;
            
        default:
            // error
            break;
    }
}
