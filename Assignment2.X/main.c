/*
 * File:   main.c
 * Authors: Emanuele Rambaldi, Francesco Ferrazzi, Mohammad Al Horany, Salvatore D'Ippolito
 *
 * Created on 6 dicembre 2022, 11.35
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
#include <math.h>

#include "timer_functions.h"
#include "spi_functions.h"
#include "parser.h"

// The size of both Circular Buffers is determined by considering both the frequency at
// which they are filled up and the frequency at which they are freed up.

// RECEIVER
// Since the receiver Circular Buffer is filled up by the characters coming from the UART2,
// the filling frequency corresponds to the frequency at which the UART2 exchanges data. 
// In particular, the baud frequency has been set to 9600: this means that 9600 bauds are exchanged every second. 
// Based on the settings, each flow of information consists of 10 bauds: 1 start bit, 8 data bit, 1 stop bit.
// Therefore 10 bauds correspond to 1 data byte. The byte rate is thus 9600/10 = 960 bytes/s = 0.960 bytes/ms. 
// Instead, the task that frees up the cells of the receiver Circular Buffer is executed every heartbeat (5 ms)
// and it lasts at most up to the next heartbeat, therefore the number of bytes stored in the receiver 
// Circular Buffer is at most 5ms*0.960 = 4.80 bytes + the bytes
// that can be received during the task execution (5*0.960 = 4.80). 
// That's around 10 bytes. So the dimension of the receiver Circular Buffer can be set to 14 to leave some allowance.
// Let's now prove that the process of freeing up the cells takes less than 5ms: since the characters are written on the LCD,
// the process is executed at the SPI serial clock frequency.
// The SPI serial clock frequency has been set, via the primary and secondary prescaler, to FCY/2 = 1843200/2 = 921600 Hz.
// This means that 921600 bits are exchanged every second.
// The byte rate is therefore 921600/8 = 115200 bytes/s. The number of exchanged bytes every ms is instead 115.2 bytes/ms.
// Characters though are written on the SPI not at every loop, but only when the '*' in the incoming message is received.
// (that is when the end of the message is reached and therefore the msg_payload field of the pstate structure is filled with a new rpm value).
// When this happens the amount of data exchanged with the SPI is 5 characters to clean the first row and 5 characters to write the rpm value
// (10 bytes in total). 
// The worst case scenario is when all the possible 10 incoming bytes are stored in the receiver Circular Buffer and
// the first element that is read is an asterisk, thus meaning that at the first iteration 10 bytes are written on the SPI.
// Then, to stay in the worst case scenario, the subsequent message should be composed of only 9 characters (minimum number
// of characters a message can be composed of, because it means that the rpm is one-digit long).
// All in all, the writing process is performed at most 2 times, determining 2x10 = 20 bytes to be exchanged with the SPI.
// At a 115.2 bytes/ms rates, 20 bytes take 0.18 ms: far less than 5ms.
// The transmitter IR can be called both during the filling and the emptying.
// This should not lead to problems when the buffer is filled up, because in the worst case less data are written.
// Also when the buffer is freed up no problem should arise, since between two calls to the transmitter IR
// there is time for the receiver Circular Buffer to be freed by the task function.

// TRANSMITTER
// As far as the transmission is concerned, a Circular Buffer is used as well.
// A message containing both the current value and the room temperature is sent every second. 
// So the number of cells of the transmitter Circular Buffer has to be at least big enough to contain an entire message
// in the worst case scenario. 
// This consist in having a negative current with 2 integer digits and 1 decimal digit (3 bytes for the digits, 2 bytes
// for the symbols; 5 overall) and either a positive temperature with 3 integer digits and 1 decimal digit or a negative
// temperature with 2 digits and 1 decimal digit (5 bytes overall).
// Then the message_type string, together with symbols ($,commas,*) should be considered: 9 additional bytes.
// The total number of bytes in the worst case hypothesis is therefore 5+5+9 = 19.
// Let's consider a dimension of 22 bytes to leave an allowance.
// As regards the process of freeing up the cells of the transmitter Circular Buffer, it is carried out by the UART2
// and therefore at a rate equal to 9600 bauds/sec.
// Since the transmitter Circular Buffer is filled up every second, the time available for freeing it up is more or
// less 1 second. Considering the baud rate, 960 bytes could be hypothetically sent in that amount of time.
// Far more than the ones stored in the circular buffer.
// The receiver IR can be called both during the filling and the emptying.
// This should not lead to problems when the buffer is filled up, because in the worst case less data are written.
// Also when the buffer is freed up no problem should arise since every heartbeat (5ms) the receiver interrupt is
// disabled for a while, thus letting the transmitter IR be free to execute. 
// If executed once, this mechanism may not be enough to free up the entire transmitter buffer, but it is executed
// quite some times between one refill and another (at most 1 second is available).




// ASSIGNMENT 2 EMBEDDED SYSTEMS
// DEFINES =====================================================================
#define MAX_TASKS 3
#define PTPER_MAX 32767
#define SIZE_CB_R 14
#define SIZE_CB_T 22


// GLOBAL VARIABLES ============================================================
typedef struct{
    int n;
    int N;
    void (*fptr)();
} heartbeat;                    // Struct for task management

typedef struct{
    int kw;
    int kr;
    char* buffer;
} cb;                           // Struct for circular buffer management

cb cb_r;                        // Circular buffer struct for the receiver
cb cb_t;                        // Circular buffer struct for the transmitter

heartbeat schedInfo[MAX_TASKS]; // Heartbeat struct for the tasks

parser_state pstate;            // Struct containing fields related to the parser-function's state

// PWM SETUP PERIOD FUNCTION ===================================================
void pwm_setup_period(int ms){
    // FCY = 7372800/4 = 1843200 Hz
    // In 1 second there would be 1843200 clocks steps
    // FPWM = 1 KHz, therefore PTPER should count as many clock steps as the one corresponding to 1 ms (1842.2 clock steps)
    // This can be easily put in a 15 bit register (max 32767)
    // Therefore it is enough to set a prescaler of 1:1 -> then we have 1842.2/1 = 1842.2 clock steps
    
    double PTPER_no_prescale = FCY * (ms/1000.0);   // in our case ms = 1 since Fpwm should be 1 kHz -> FCY/1000 = 1843.200
                                                    // [1/s * s = adim.] 1000.0 because I have to cast the division to double
    double prescale_needed = PTPER_no_prescale / PTPER_MAX;

    if (prescale_needed < 1) {
        PTCONbits.PTCKPS = 0;
        PTPER = (int) (PTPER_no_prescale / 1);  
    } else if (prescale_needed < 4) {
        PTCONbits.PTCKPS = 1;
        PTPER = (int) (PTPER_no_prescale / 4);
    } else if (prescale_needed < 16) {
        PTCONbits.PTCKPS = 2;
        PTPER = (int) (PTPER_no_prescale / 16);
    } else if (prescale_needed < 64) {
        PTCONbits.PTCKPS = 3;
        PTPER = (int) (PTPER_no_prescale / 64);
    }
} 

// MESSAGE PAYLOAD PARSING FUNCTIONS ===========================================
int extract_integer(const char* str) {
    int i = 0, number = 0, sign = 1;
    
    if (str[i] == '-'){
        sign = -1;
        i++;
    }
    else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    
    while (str[i] != ',' && str[i] != '\0') {   // if neither a comma nor the end of the string is parsed
        number *= 10;                           // multiply the current number by 10;
        number += str[i] - '0';                 // converting character to decimal number
        i++;
    }
    return sign*number;
}

// TASKS FUNCTIONS =============================================================
void task0(){ // read the bytes stored into the circular_buffer_r

    int possible_rpm;
    int rpm;             // rpm value, sent by the PC via UART2, imposing the PWM signal duty-cycle
    
    IEC1bits.U2RXIE = 0; // disable UART2 Receiver interrupt
    
    while (cb_r.kr != cb_r.kw) {            // while the 'receiver' circular buffer is not empty
        char byte = cb_r.buffer[cb_r.kr];
        int ret = parse_byte(&pstate, byte);
        if (ret == NEW_MESSAGE) {           // if I filled up the msg_payload field of the pstate structure with the payload of a new message
            if (strcmp(pstate.msg_type, "MCREF") == 0) {
              possible_rpm = extract_integer(pstate.msg_payload); 
              if (possible_rpm >= 0 && possible_rpm <= 1000){
                    rpm = possible_rpm;
                    
                    PDC2 = (int)((rpm/1000.0)*2*PTPER); // change the duty cycle of the PWM signal based on the rpm value
                  
                    clear_row(FIRST_ROW,4,8);
                    print_int_digits(FIRST_ROW,4,rpm);
              }
            }
        }
        cb_r.kr = (cb_r.kr+1)%SIZE_CB_R;    // increment the index and evaluate the remainder of the division by
                                            // the size of the circular buffer to avoid exceeding bounds
    }
    IEC1bits.U2RXIE = 1;                    // enable UART2 Receiver interrupt
}

void task1(){ // turn on/off the LED
    LATBbits.LATB0 = !LATBbits.LATB0; // change the LED state
}

void task2(){ // convert the signal (temperature and potentiometer: automatic sampling, automatic conversion)
    
    int ADCValue1;          // int (16 bit) that contains the converted value for the first channel (potentiometer)
    int ADCValue2;          // int (16 bit) that contains the converted value for the second channel (temperature)
    double AmpereValue;     // float that contains the ampere corrisponding to the converted value for the first channel (potentiometer)
    int AmpereValueInt;
    double VoltValue1;      // float that contains the voltage corresponding to the converted value for the first channel (potentiometer)
    double VoltValue2;      // float that contains the voltage corresponding to the converted value for the second channel (temperature)
    double MilliVoltValue;  // float that contains the voltage corrisponding to the converted value in millivolt
    double CelsiusValue;    // float that contains the temperature corrisponding to the converted value
    int CelsiusValueInt;
    char msg[20];
    
    int i;
    
    while (ADCON1bits.DONE == 0);   // while the conversion is not finished wait
    ADCValue1 = ADCBUF0;            // get ADC value for the first channel (potentiometer)
    ADCValue2 = ADCBUF1;            // get ADC value for the second channel (temperature sensor)
      
    VoltValue1=(ADCValue1*5)/1023.0;
    AmpereValue = 10.0*(VoltValue1-3); // y-y0 = m(x-x0) -> y-0 = 10(x-3) -> y = 10(x-3)
    AmpereValueInt = (int) AmpereValue;
    
    // Check if the current intensity exceeds the 15 A threshold
    if(AmpereValue > 15){
        LATBbits.LATB1 = 1; // turn on the D4 LED
    }
    else{
        LATBbits.LATB1 = 0; // turn off the D4 LED
    }
    
    clear_row(SECOND_ROW,2,5);
    print_int_digits(SECOND_ROW,2,AmpereValueInt);    

    
    VoltValue2=(ADCValue2*5)/1023.0;                // since the volts go from 0 to 5 even in this case
    MilliVoltValue = VoltValue2*1000;
    CelsiusValue = ((MilliVoltValue-750)/10.0)+25;  // y-y0 = m(x-x0) -> y-750 = 10(x-25) -> x-25 = (y-750)/10 -> x = (y-750)/10 + 25
    CelsiusValueInt = (int) CelsiusValue;
    
    clear_row(SECOND_ROW,10,13);
    print_int_digits(SECOND_ROW,10,CelsiusValueInt);  
    
    // STORE THE GATHERED DATA AS A MESSAGE INTO THE 'TRANSMITTER' CIRCULAR BUFFER (every 10 iterations)
    sprintf(msg,"$MCFBK,%.1f,%.1f*",AmpereValue,CelsiusValue);
    IEC1bits.U2TXIE = 0; // disable UART2 Transmitter interrupt
    
    // store the message into the 'transmitter' circular buffer
    for(i=0; i<strlen(msg); i++){
        if((cb_t.kw+1)%SIZE_CB_T != cb_t.kr){   // if the 'transmitter' circular buffer is not full
            cb_t.buffer[cb_t.kw] = msg[i];
            cb_t.kw = (cb_t.kw+1)%SIZE_CB_T;    // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }

    // fill the 'transmitter' FIFO to trigger the 'transmitting' mechanism
    while(U2STAbits.UTXBF == 0){            // while the 'transmitter' FIFO is not full
        if(cb_t.kr != cb_t.kw){             // if the 'transmitter' circular buffer is not empty
            U2TXREG = cb_t.buffer[cb_t.kr]; // write the the content of the current cell of the circular buffer into the UART2 'transmitter' fifo
            cb_t.kr = (cb_t.kr+1)%SIZE_CB_T;// increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }
    IEC1bits.U2TXIE = 1; // enable UART2 Transmitter interrupt 
}

// TASK SCHEDULER FUNCTION 
void scheduler(){
    
    for(int i=0; i<MAX_TASKS;i++){
        schedInfo[i].n++;
        if(schedInfo[i].n >= schedInfo[i].N){
            schedInfo[i].fptr();    // call the function corresponding to the i-th task
            schedInfo[i].n = 0;     // reset the loop counter 
        }
    }

}

// INTERRUPT HANDLERS ==========================================================
void __attribute__((__interrupt__,__auto_psv__))  _U2RXInterrupt(){ // Interrupt handler -> event: UART2 Receiver FIFO gets one of the cells filled up
    IFS1bits.U2RXIF = 0; // reset interrupt flag
    
    // fill the 'receiver' circular buffer with bytes from the 'receiver' FIFO until it is empty
    while(U2STAbits.URXDA == 1){                // while the 'receiver' FIFO is not empty
        if((cb_r.kw+1)%SIZE_CB_R != cb_r.kr){   // if the 'receiver' circular buffer is not full
            cb_r.buffer[cb_r.kw] = U2RXREG;     // read the character and store it in the current cell of the circular buffer
            cb_r.kw = (cb_r.kw+1)%SIZE_CB_R;    // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }
}

void __attribute__((__interrupt__,__auto_psv__))  _U2TXInterrupt(){ // Interrupt handler -> event: UART2 Transmitter FIFO gets one of the cells freed up
    IFS1bits.U2TXIF = 0; // reset interrupt flag
    
    // fill the 'transmitter' FIFO with bytes from the 'transmitter' circular buffer until it is full
    while(U2STAbits.UTXBF == 0){                // while the 'transmitter' FIFO is not full
        if(cb_t.kr != cb_t.kw){                 // if the 'transmitter' circular buffer is not empty
            U2TXREG = cb_t.buffer[cb_t.kr];     // write the the content of the current cell of the circular buffer into the UART2 'transmitter' fifo
            cb_t.kr = (cb_t.kr+1)%SIZE_CB_T;    // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }
}

// MAIN FUNCTION ===============================================================
int main(void) {
    
    tmr_wait_ms(TIMER1,1000); // wait for the LCD to set-up
    
    // Parser Inintialisation 
    pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    
    // UART2 SETUP
    U2BRG = 11;             // [FCY/(16*desired baud rate)]-1 with desired baud rate = 9600 baud/s, 10 baud (1 start-bit, 8 data-bit, 1 stop-bit) = 1 data byte -> baud rate = 9600, bit rate = 9600/10 = 960 hz
    U2MODEbits.UARTEN = 1;  // enable the UART2 peripheral
    U2STAbits.UTXEN = 1;    // enable the transmission (on the board side)
    U2STAbits.URXISEL = 1;  // specify that the Interrupt Flag of the receiver is set to 1 whenever it receives a character
    U2STAbits.UTXISEL = 0;  // specify that the Interrupt Flag of the transmitter is set to 1 when a byte of the transmitter FIFO is transmitted
    
    // UART2 INTERRUPT SETUP
    IEC1bits.U2RXIE = 0;    // initially disable UART2 Receiver interrupt
    IEC1bits.U2TXIE = 0;    // initally disable UART2 Transmitter interrupt
    IFS1bits.U2RXIF = 0;    // reset interrupt flag (UART2 Receiver)
    IFS1bits.U2TXIF = 0;    // reset interrupt flag (UART2 Transmitter)
    
    // SPI SETUP
    SPI1CONbits.MSTEN = 1;  // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode
    SPI1CONbits.PPRE = 3;   // 1:1 primary prescaler
    SPI1CONbits.SPRE =6;    // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI peripheral
    
    // LED SETUP
    TRISBbits.TRISB0 = 0;   // D3 LED as output
    TRISBbits.TRISB1 = 0;   // D4 LED as output

    // ADC SETUP (manual sampling - manual conversion)
    // SIMULTANEOUS MODE SINCE WE USE MULTIPLE-CHANNELS -> THE RESULT OF THE FIRST CHANNEL CONVERSION GOES INTO ADCBUF0, THE RESULT OF THE SECOND CHANNEL CONVERSION GOES INTO ADCBUF1
    ADCON3bits.ADCS = 8;    // selects how long is one Tad: in this case it is 8*Tcy
    // Conversion time duration (Tconv): fixed to 12 Tad
    ADCON1bits.ASAM = 1;    // set how the sampling begins as automatic
    ADCON3bits.SAMC = 16;   // Sample time Tsamp 16 Tad -> so I don't need to set SAMP to 0 to make the (sampling end) and the conversion begin
    ADCON1bits.SSRC = 7;    // set how the sampling ends and conversion begins as automatic
    ADCON2bits.CHPS = 2;    // set the number of channels to 2
    ADCHSbits.CH0SA = 2;    // choose the positive input to channel 0 - in particular select pin AN2 (0010) where the potentiometer is connected 
    ADCHSbits.CH123SA = 1;  // choose the positive input to channel 1,2,3 - in particular CH1 positive input is set to AN3 (where the temperature sensor is connected), CH2 positive input is set to AN4, CH3 positive input is set to AN5
    ADPCFG = 0xFFFF;        // first set all the bits to 1 
    ADPCFGbits.PCFG2 = 0;   // set AN2 bit to 0 
    ADPCFGbits.PCFG3 = 0;   // set AN3 bit to 0 
    // selects which pin should be used for A/D (if the bit is 0 the corresponding pin will be used for ADC)
    ADCON1bits.SIMSAM = 1;  // set the multi-channel modality to simultaneous
    ADCON2bits.SMPI = 2;    // set this bit to the number of channels that we use
    ADCON1bits.ADON = 1;    // turn on the ADC
    
    // PWM signal setup
    PTCONbits.PTMOD = 0;    // set the free-running mode (if PTMR<PDC, the signal is 1 (high), otherwise it goes down)
    PWMCON1bits.PEN2L = 1;  // PWM2L pin is enabled for PWM output (low)
    PWMCON1bits.PEN2H = 1;  // PWM2H pin is enabled for PWM output (high)
    pwm_setup_period(1);    // since the Fpwm should be 1 KHz, the period that I count should be of 1 ms, therefore 1843.2 clock cycles
    PDC2 = (int)((0/100.0)*2*PTPER); // initialize the signal to a 0% Duty Cycle (the low takes all the Tpwm period) 
    // -> this is considered in the hypothesis that:
    // * 0/100 duty cycle corresponds to 0 V provided to the motor
    // * 100/100 duty cycle corresponds to +5 V provided to the motor
    PTCONbits.PTEN = 1;     // enable the PWM Time Base Timer
    
    // INITIALIZE BUFFERS
    // Define two char buffers in memory
    char buffer_r[SIZE_CB_R];
    char buffer_t[SIZE_CB_T];
    // Link the char pointers to each one of these buffers
    cb_r.buffer = buffer_r;
    cb_t.buffer = buffer_t;
    
    // INITIALIZE TASKS
    schedInfo[0].n = 0;
    schedInfo[0].N = 1;         // T = 5 ms
    schedInfo[0].fptr = task0;  // read from cb_r task
    
    schedInfo[1].n = 0;
    schedInfo[1].N = 200;       // T = 1000 ms -> the LED should blink at 1 Hz 
    schedInfo[1].fptr = task1;  // D3 blinking task
    
    schedInfo[2].n = 0;
    schedInfo[2].N = 200;       // T = 1000 ms -> the micro should send the message at 1 Hz
    schedInfo[2].fptr = task2;  // ADC and store in cb_t task
    
    // PRINT "RPM=+0000" IN THE FIRST ROW OF THE LCD and BOTH "A=" AND "T=" IN THE SECOND ONE
    char *str1 = "RPM=+0000";
    print_string(str1);
    char *str2 = "A=";
    place_cursor(SECOND_ROW,0);
    print_string(str2);
    char *str3 = "T=";
    place_cursor(SECOND_ROW,8);
    print_string(str3);
    
    // START TIMERS
    tmr_setup_period(TIMER1, 5); // heartbeat: 5ms -> 200 Hz
    
    while(1){        
        scheduler();
        tmr_wait_period(TIMER1);
    }
             
    return 0;
}