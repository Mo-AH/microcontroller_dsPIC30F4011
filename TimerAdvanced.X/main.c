
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

int main(int argc, char** argv) {
    
    TRISBbits.TRISB0 = 0;   //led D3 as output
    TRISEbits.TRISE8 = 1;   // set the button s5 as input
    PORTEbits.RE8 == 0; //pressed
    int n_pulse = 0;
    int state = 0;
    int n_period_pressed = 0;
    tmr_setup_period(TIMER1,100);
    
    while(1){
        if ((n_pulse==0 && state>0) || (n_pulse==2 && state>1) || (n_pulse==4 && state==3))
            LATBbits.LATB0 = 1;
        else
            LATBbits.LATB0 = 0;

        if (PORTEbits.RE8 == 0)     //se il bottone è premuto
            n_period_pressed++;
        else {
            if (n_period_pressed > 29)
                state = 0;
            else if (n_period_pressed > 0)
                state = (state%3)+1;
            n_period_pressed = 0;
        }

        n_pulse++;
        if (n_pulse == 10)
            n_pulse = 0;
        tmr_wait_period(TIMER1);
    }
    return 0;
}

