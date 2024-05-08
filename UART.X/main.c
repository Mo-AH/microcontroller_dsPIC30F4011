/*
 * File:   main.c
 * Author: eppa1
 *
 * Created on 4 novembre 2022, 21.31
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

// DEFINES =====================================================================
#define FOSC 7372800L //Hz
#define FCY FOSC/4
#define PR_MAX 65535
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define TIMER5 5
#define SIZE_CB 32
#define FIRST_ROW 1
#define SECOND_ROW 2

// GLOBAL VARIABLES ============================================================
int char_counter = 0;
int current_position = 0;
short filled_cells_t = 0; // number of filled cells in the transmitter FIFO (short integer (16 bits) to make operations composed by less instructions (after all it reaches a value of 4 at maximum))
char circular_buffer[SIZE_CB];
short filled_cells_cb = 0; // number of filled cells in the circular buffer (short integer (16 bits) to make operations composed by less instructions (after all it reaches a value of 4 at maximum))
int kw = 0; // index to span the circular buffer during the writing
int kr = 0; // index to span the circular buffer during the reading

// TIMER FUNCTIONS =============================================================
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

            T1CONbits.TON = 1;  // turn on the timer!
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

            T2CONbits.TON = 1;  // turn on the timer!
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

            T3CONbits.TON = 1;  // turn on the timer!
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

            T4CONbits.TON = 1;  // turn on the timer!
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
            
            T5CONbits.TON = 1;  // turn on the timer!
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

// REGULAR FUNCTIONS ===========================================================


void place_cursor(int row,int position){
    while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    if(row == FIRST_ROW){SPI1BUF = 128 + position;}
    else if (row == SECOND_ROW){SPI1BUF = 192 + position;}
}

void clear_row(int row, int start){
    
    place_cursor(row,start);
    
    // CLEAN THE FIRST LINE OF THE LCD (16 spots available)
    for(int i = start; i<16; i++){
        while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
        SPI1BUF = ' ';
    }
}

void fill_second_row(){
    char str2[64]; // string that will contain the number of character received (in order to update the LCD)
    // WRITE THE NUMBER OF RECEIVED CHARACTERS ON THE LCD (here or in the ISR of the receiver every time that a character is received?)
    sprintf(str2, "%d", char_counter);
    place_cursor(SECOND_ROW,11); //place the cursor after the "Char Recv: " string
    for(int i = 0; i< strlen(str2); i++){
        while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
        SPI1BUF = str2[i];
    }
}

// INTERRUPT HANDLERS ==========================================================
// PRIORITY 2 (higher)
void __attribute__((__interrupt__,__auto_psv__))  _U2RXInterrupt(){ // Interrupt handler -> event: UART2 Receiver FIFO gets one of the cells filled up
    IFS1bits.U2RXIF = 0; // reset interrupt flag
    if (filled_cells_cb < SIZE_CB){ // if the circular buffer is not full
        while(U2STAbits.URXDA == 0); // loop until there is a character to be read sent by the sender (in priciple there is no need. Since the ISR has been called, I'm pretty sure that there is)
        circular_buffer[kw] = U2RXREG; // read the character and store it in the current cell of the circular buffer
        kw = (kw+1)%SIZE_CB; // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        filled_cells_cb++; // increase the number of filled cells since one of them has just been written
        char_counter++; //update the received characters counter (even if the received character is \n or \r, it is still a received character)
    }
}

void __attribute__((__interrupt__,__auto_psv__))  _U2TXInterrupt(){ // Interrupt handler -> event: UART2 Transmitter FIFO gets one of the cells freed up
    IFS1bits.U2TXIF = 0; // reset interrupt flag
    filled_cells_t--; // decrease the counter that takes into account the number of filled cells in the Transmitter FIFO
}

// S5 BUTTON INTERRUPT HANDLER
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
    IEC0bits.INT0IE = 0; // disable INT0 interrupt (S5) for the whole duration of the ISR
    IFS0bits.INT0IF = 0; // reset interrupt flag
    
    char str3[64];
    int i = 0;
    
    // SEND THE NUMBER OF CHARACTERS RECEIVED SO FAR TO THE UART2 FIFO (by keeping an eye on the filled cells of the FIFO to avoid overflow)
    sprintf(str3, "%d", char_counter);
    while(i< strlen(str3)){
        if(filled_cells_t < 4){ // if the number of filled spots in the Transmitter FIFO is 3 or less (to avoid overflow)
            U2TXREG = str3[i]; // fill one spot with the i-th character of the string
            filled_cells_t++;
            i++;
        }
    }
    
    IEC0bits.INT0IE = 1; // re-enable INT0 interrupt (S5)
}

// S6 BUTTON INTERRUPT HANDLER
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(){ // Interrupt handler -> event: S6 button pushed
    // Here I don't care about bounces, since if the row gets cleared multiple times in a matter of few milliseconds it's not a big problem
    IFS1bits.INT1IF = 0; // reset interrupt flag
    
    char_counter = 0; // reset the received characters counter    
    clear_row(FIRST_ROW,0);
    clear_row(SECOND_ROW,11);
    
    place_cursor(SECOND_ROW, 11);
    //while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    //SPI1BUF = '0';
    fill_second_row();
    
    place_cursor(FIRST_ROW,0);
    current_position = 0; // update the current position of the cursor
}

// MAIN FUNCTION ===============================================================

// Insert a dummy algorithm that lasts 7 ms (wait_period)
// write on the SPI only on main (modify the INT1 ISR)
// Evaluate the maximum size of the circular buffer

// PRIORITY 0
int main(void) {
    
    // UART2 SETUP
    U2BRG = 11; // [FCY/(16*desired baud rate)]-1 with desired baud rate = 9600
    U2MODEbits.UARTEN = 1; // enable the UART2 peripheral
    U2STAbits.UTXEN = 1; // enable the transmission (on the board side)
    U2STAbits.URXISEL = 1; // specify that the Interrupt Flag of the receiver is set to 1 whenever it receives a character
    U2STAbits.UTXISEL = 0; // specify that the Interrupt Flag of the transmitter is set to 1 when a byte of the transmitter FIFO is transmitted
    
    // S5 IF SETUP
    TRISEbits.TRISE8 = 1;   // S5 button as input
    IEC0bits.INT0IE = 1;    // enable INT0 interrupt (S5)
    IFS0bits.INT0IF = 0;    //reset interrupt flag (S5)
    
    // S6 IF SETUP
    TRISDbits.TRISD0 = 1;   // S6 button as input
    IEC1bits.INT1IE = 1;    // enable INT1 interrupt (S6)
    IFS1bits.INT1IF = 0;    //reset interrupt flag (S6)
    
    // UART2 IF SETUP
    IEC1bits.U2RXIE = 1; // enable UART2 Receiver interrupt
    IEC1bits.U2TXIE = 1; // enable UART2 Transmitter interrupt
    IPC6bits.U2RXIP = 2; // set the priority of the UART2 Receiver interrupt to 2 (main() priority: 0, other ISR priority: 1?) so that it is able to block the execution of other ISR
    IPC6bits.U2TXIP = 2; // set the priority of the UART2 Transmitter interrupt to 2 (main() priority: 0, other ISR priority: 1?) so that it is able to block the execution of other ISR
    IFS1bits.U2RXIF = 0; //reset interrupt flag (UART2 Receiver)
    IFS1bits.U2TXIF = 0; //reset interrupt flag (UART2 Transmitter)

    tmr_wait_ms(TIMER1,1000); // wait for the LCD to set-up
    
    // SPI SETUP
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE =6; // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI peripheral
    
    int i; // general purpose index    
    
    // PRINT "Char Recv: " IN THE SECOND ROW OF THE LCD
    char *str1 = "Char Recv: 0";
    place_cursor(SECOND_ROW,0);
    for(i = 0; i< strlen(str1); i++){
        while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
        SPI1BUF = str1[i];
    }
    
    // START THE TIMERS
    tmr_setup_period(TIMER1,10);
    
    while(1){
        
        tmr_wait_ms(TIMER2, 7);
        TMR1 = 0; // empty the Timer1 'counter' register (AKA start here the counting))
        
        while(filled_cells_cb > 0){
            
            // PROCESS THE RECEIVED CHARACTERS STORED IN THE CIRCULAR BUFFER
            // if a either a new line or a carriage return is read from the circular buffer
            // or the end of the row has been reached
            if (circular_buffer[kr] == '\n' || circular_buffer[kr] == '\r'){ 
                clear_row(FIRST_ROW,0);
                place_cursor(FIRST_ROW,0);
                current_position = 0; // update the current position of the cursor
            }
            else if(current_position == 16){ 
                clear_row(FIRST_ROW,0);
                place_cursor(FIRST_ROW,0);
                current_position = 0; // update the current position of the cursor
                while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
                SPI1BUF = circular_buffer[kr]; // write the read character on the LCD
                current_position++; // update the current position of the cursor
            }
            else {
                // PLACE THE CURSOR AFTER THE LAST-WRITTEN CHARACTER IN THE FIRST LINE
                place_cursor(FIRST_ROW,current_position);
                while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
                SPI1BUF = circular_buffer[kr]; // write the read character on the LCD
                current_position++; // update the current position of the cursor
            }

            kr = (kr+1)%SIZE_CB; // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
            filled_cells_cb--; // decrease the number of filled cells since one of them has just been read
        
            fill_second_row();
        }
        
        tmr_wait_period(TIMER1);
    }
    
    
    return 0;
}
