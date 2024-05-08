/*
 * File:   main.c
 * Author: Salvatore D'ippolito, Mohammad Al Horany, Francesco Ferrazzi, Emanuele Rambaldi
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
#include "timer_functions.h"
#include "spi_functions.h"

// ASSIGNMENT 1 EMBEDDED SYSTEMS
// DEFINES =====================================================================
#define SIZE_CB 16
// The size of the Circular Buffer is determined by considering both the frequency at which it is filled up and the frequency at which it is freed up.
// Since the Circular Buffer is filled up by the characters coming from the UART2, the former corresponds to the frequency at which the UART2 exchanges data. 
// In particular, the baud frequency has been set to 9600: this means that 9600 bauds are exchanged every second. 
// Based on the settings, each flow of information consists of 10 bauds: 1 start bit, 8 data bit, 1 stop bit. Therefore 10 bauds correspond to 1 data byte. The byte rate is thus 9600/10 = 960 bytes/s = 0.960 bytes/ms. 
// During one iteration of the main function (10 ms, since it loops at 100 Hz) the maximum number of received characters stored in the Circular Buffer is then 10ms*0.960 bytes/ms = 9.60 bytes.
// As regards the process of freeing up the cells of the Circular Buffer, since each character is written on the LCD, it is executed at the SPI serial clock frequency.
// The SPI serial clock frequency has been set, via the primary and secondary prescaler, to FCY/2 = 1843200/2 = 921600 Hz. This means that 921600 bits are exchanged every second.
// The byte rate is therefore 921600/8 = 115200 bytes/s. The number of exchanged bytes is instead 115.2 bytes/ms.
// However the portion of an iteration of the main function whereby a character can be taken from the Circular Buffer and written in the LCD lasts 3 ms. 
// The number of bytes exchanged with the SPI in 3 ms evaluates to 345.6.
// To sum up, the maximum number of characters written in the Circular Buffer during one iteration of the main function is 9.60 bytes; whereas the maximum number of read characters in the same iteration is 345.6 bytes.
// Worst Case Scenario: at the beginning of the i-th iteration the Circular Buffer is empty. Furthermore no characters are received from the UART2 during the 7 ms execution of the simulated algorithm.
// As a consequence, immediately after these 7 ms,the process doesn't enter in the while loop used to read the Circular Buffer. Therefore the process sleeps 3 ms waiting for the 10 ms (Timer2) to elapse.
// During this period of time at maximum 3*0.960 = 2.88 bytes can be received. At the subsequent iteration (i+1) the Circular Buffer can then have around 3 bytes occupied from the previous iteration.
// Now, if the maximum number of bytes arrives on the UART2 during the 10 ms execution of the iteration, the worst case scenario is that 2.88 + 9.60 = 12.48 bytes accumulate in the circular buffer.
// The Circular Buffer should be then big enough to contain at least these 12.48 bytes, approximated to 13 for simplicity. Let's consider SIZE_CB = 16 to leave an allowance.

// GLOBAL VARIABLES ============================================================
// Some of these variables are defined as characters (8 bits), instead of integers, in order to save memory
unsigned char char_counter = 0; // number of received characters; since it is defined as an unsigned character the maximum number that can contain is 255
char current_position = 0;
char send_UART = 0;
char filled_cells_t = 0; // number of filled cells in the transmitter FIFO (character (8 bits) to save memory)
char circular_buffer[SIZE_CB];
char filled_cells_cb = 0; // number of filled cells in the circular buffer (character (8 bits) to save memory)
int kw = 0; // index to span the circular buffer during the writing
int kr = 0; // index to span the circular buffer during the reading

// INTERRUPT HANDLERS ==========================================================
// PRIORITY 5
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

// PRIORITY 5
void __attribute__((__interrupt__,__auto_psv__))  _U2TXInterrupt(){ // Interrupt handler -> event: UART2 Transmitter FIFO gets one of the cells freed up
    IFS1bits.U2TXIF = 0; // reset interrupt flag
    filled_cells_t--; // decrease the counter that takes into account the number of filled cells in the Transmitter FIFO
}

// DEFAULT PRIORITY = 4
// S5 BUTTON INTERRUPT HANDLER 
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
    IFS0bits.INT0IF = 0; // reset interrupt flag
    
    IFS0bits.T3IF = 0;  // reset Timer3 interrupt flag
    TMR3 = 0; // empty the Timer3 'counter' register
    
    IEC0bits.T3IE = 1;  // enable Timer3 interrupt
    
}

// DEFAULT PRIORITY = 4
// TIMER3 INTERRUPT HANDLER (to avoid button S5 bounces)
void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(){ // Interrupt handler -> event: Timer2 elapsed
    IFS0bits.T3IF = 0; //reset Timer3 interrupt flag

    if (PORTEbits.RE8 == 1){ // if button S5 has been released
        send_UART = 1; // set a flag to 1
    }
    
    IEC0bits.T3IE = 0; // disable Timer3 interrupt    
}

// MAIN FUNCTION ===============================================================

// PRIORITY 0
int main(void) {
    
    // UART2 SETUP
    U2BRG = 11; // [FCY/(16*desired baud rate)]-1 with desired baud rate = 9600 baud al secondo, 10 baud (1 start-bit, 8 bit di dato, 1 stop-bit) = 1 byte di dato -> baud rate = 9600, bit rate = 9600/10 = 960 hz
    U2MODEbits.UARTEN = 1; // enable the UART2 peripheral
    U2STAbits.UTXEN = 1; // enable the transmission (on the board side)
    U2STAbits.URXISEL = 1; // specify that the Interrupt Flag of the receiver is set to 1 whenever it receives a character
    U2STAbits.UTXISEL = 0; // specify that the Interrupt Flag of the transmitter is set to 1 when a byte of the transmitter FIFO is transmitted
    
    // TIMER 3 SETUP (to avoid button S5 bounces)
    IEC0bits.T3IE = 0; // disable T3 interrupt (TIMER3)
    IFS0bits.T3IF = 0; //reset interrupt flag (TIMER3)
    
    // S5 INTERRUPT SETUP
    TRISEbits.TRISE8 = 1; // S5 button as input
    IEC0bits.INT0IE = 1; // enable INT0 interrupt (S5)
    IFS0bits.INT0IF = 0; //reset interrupt flag (S5)
    
    // S6 INTERRUPT SETUP
    TRISDbits.TRISD0 = 1; // S6 button as input
    IFS1bits.INT1IF = 0; //reset interrupt flag (S6)
    
    // UART2 INTERRUPT SETUP
    IEC1bits.U2RXIE = 1; // enable UART2 Receiver interrupt
    IEC1bits.U2TXIE = 1; // enable UART2 Transmitter interrupt
    IFS1bits.U2RXIF = 0; //reset interrupt flag (UART2 Receiver)
    IFS1bits.U2TXIF = 0; //reset interrupt flag (UART2 Transmitter)
    
    // UART2 INTERRUPT PRIORITY SETUP
    IPC6bits.U2RXIP = 5; // set the priority of the UART2 Receiver interrupt to 5 (main() priority: 0, other ISR priority: 4) so that it is able to block the execution of other ISR
    IPC6bits.U2TXIP = 5; // set the priority of the UART2 Transmitter interrupt to 5 (main() priority: 0, other ISR priority: 4) so that it is able to block the execution of other ISR

    tmr_wait_ms(TIMER1,1000); // wait for the LCD to set-up
    
    // SPI SETUP
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE =6; // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI peripheral
    
    int i; // general purpose index
    
    
    // PRINT "Char Recv: " IN THE SECOND ROW OF THE LCD
    char *str1 = "Char Recv: ";
    place_cursor(SECOND_ROW,0);
    for(i = 0; i< strlen(str1); i++){
        while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
        SPI1BUF = str1[i];
    }
    
    // START THE TIMERS
    tmr_setup_period(TIMER3,10); // Timer3 to avoid button S5 bounces
    tmr_setup_period(TIMER2,10); // Timer2 to loop at 100 Hz
    
    while(1){
        
        // SIMULATE A 7 MS LONG ALGORITHM
        tmr_wait_ms(TIMER1,7);
            
        // PROCESS THE RECEIVED CHARACTERS STORED IN THE CIRCULAR BUFFER
        while (filled_cells_cb > 0){ // if the circular buffer is not empty

            place_cursor(FIRST_ROW,current_position);

            // IF EITHER A 'NEW LINE' OR A 'CARRIAGE RETURN' IS READ FROM THE CIRCULAR BUFFER: CLEAR THE FIRST ROW AND PLACE THE CURSOR AT THE BEGINNING OF THE FIRST ROW
            if (circular_buffer[kr] == '\n' || circular_buffer[kr] == '\r'){ 
                clear_row(FIRST_ROW,0);
                place_cursor(FIRST_ROW,0);
                current_position = 0; // update the current position of the cursor
            }
            // IF THE END OF THE FIRST ROW HAS BEEN REACHED: CLEAR THE FIRST ROW, PLACE THE CURSOR AT THE BEGINNING OF THE FIRST ROW AND PRINT THE CURRENT CHARACTER
            else if(current_position == 16){ 
                clear_row(FIRST_ROW,0);
                place_cursor(FIRST_ROW,0);
                current_position = 0; // update the current position of the cursor
                while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
                SPI1BUF = circular_buffer[kr]; // write the read character on the LCD
                current_position++; // update the current position of the cursor
            }
            // ELSE PRINT THE CURRENT CHARACTER
            else {
                while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
                SPI1BUF = circular_buffer[kr]; // write the read character on the LCD
                current_position++; // update the current position of the cursor
            }

            kr = (kr+1)%SIZE_CB; // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
            filled_cells_cb--; // decrease the number of filled cells since one of them has just been read
        }
        
        // IF THE S6 BUTTON HAS BEEN PRESSED, CLEAR THE ROWS AND RESET THE CHARACTER COUNTER
        if(IFS1bits.INT1IF == 1){
            clear_row(FIRST_ROW,0);
            char_counter = 0; // reset the received characters counter
            clear_row(SECOND_ROW,11);
            current_position = 0; // update the current position of the cursor
            place_cursor(FIRST_ROW,0);
            IFS1bits.INT1IF = 0;
        }
        
        // WRITE THE NUMBER OF RECEIVED CHARACTERS ON THE LCD 
        print_char_counter(SECOND_ROW, 11, char_counter);
        
        // SEND THE NUMBER OF CHARACTERS RECEIVED SO FAR TO THE UART2 FIFO (by keeping an eye on the filled cells of the FIFO to avoid overflow)
        // Since the char_counter is a character (8 bit) and the UART2 receiver is able to retrieve the corresponding integer, I can avoid transform it into string before sending it to the UART2
        if(send_UART == 1){ // if S5 bounces have passed and the button is now released
            if(filled_cells_t < 4){ // if the number of filled spots in the Transmitter FIFO is 3 or less (to avoid overflow)
                U2TXREG = char_counter;
                filled_cells_t++; // increase the counter that takes into account the number of filled cells in the Transmitter FIFO
            }
            send_UART = 0;
        }
        
        // WAIT UNTIL THE ELAPSING OF THE 10 MILLISECONDS (to loop at 100 Hz)
        tmr_wait_period(TIMER2);
    }
    return 0;
}