/*
 * File:   main.c
 * Author: eppa1
 *
 * Created on 7 gennaio 2023, 17.33
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
#include "parser.h"

// The size of both Circular Buffers is determined by considering both the frequency at which they are filled up and the frequency at which they are freed up.
// RECEIVER
// Since the receiver Circular Buffer is filled up by the characters coming from the UART2, the filling frequency corresponds to the frequency at which the UART2 exchanges data. 
// In particular, the baud frequency has been set to 4800: this means that 4800 bauds are exchanged every second. 
// Based on the settings, each flow of information consists of 10 bauds: 1 start bit, 8 data bit, 1 stop bit. Therefore 10 bauds correspond to 1 data byte. The byte rate is thus 4800/10 = 480 bytes/s = 0.480 bytes/ms. 
// Instead, the task that frees up the cells (task0) of the receiver Circular Buffer is executed every heartbeat (100 ms) and it lasts at most up to the next heartbeat, 
// therefore the number of bytes stored in the receiver Circular Buffer is at most 100ms*0.48 = 48 bytes + the bytes that can be received during the task execution (100*0.48 = 48), so the total is 96.
// However, while new bytes are received from UART Receiver Interrupt, task0 frees up the buffer as well. As a consequence, it is not possible that all the 96 bytes are simultaneously stored in the cb.
// Given these considerations, a size of 80 has been set for the receiver circular buffers.
// TRANSMITTER
// As far as the transmission is concerned, a Circular Buffer is used as well. 
// The worst case scenario is having the MCFBK message or the MCTEM at the maximum length plus the asynchronous messages MCALE and MCACK.
// SYNCHRONOUS MESSAGES: $MCFBK,-50.00,-50.00,1* (max length = 23 bytes) - $MCTEM,-20.9* (max length = 13)
// ASYNCHRONOUS MESSAGE: $MCALE,-100.00,-100.00* (max length = 23 bytes) - $MCACK,ENA,1* (max length = 13)
// Thus, considering MCFBK + MCALE + MCACK, the total maximum length is 59 bytes.
// A dimension of 64 bytes has been set to leave an allowance.
// #########################################################################
// DA QUA IN POI NON CHIARO, LASCIAMO AL MAESTRO ###########################
// As regards the process of freeing up the cells of the transmitter Circular Buffer, it is carried out by the UART2 and therefore at a rate equal to 4800 bauds/sec.
// Since the transmitter Circular Buffer is filled up every 200ms, the time available for freeing it up is more or less 1 second.
// Considering the baud rate, 960 bytes could be hypothetically sent in that amount of time.
// Far more than the ones stored in the circular buffer.
// The receiver IR can be called both during the filling and the emptying. This should not lead to problems when the buffer is filled up, because in the worst case less data are written.
// Also when the buffer is freed up no problem should arise since every heartbeat (5ms) the receiver interrupt is disabled for a while, thus letting the transmitter IR be free to execute. 
// If executed once, this mechanism may not be enough to free up the entire transmitter buffer, but it is executed quite some times between one refill and another (at most 1 second is available).

// ASSIGNMENT 2 EMBEDDED SYSTEMS
// DEFINES =====================================================================
#define MAX_TASKS 7
#define PTPER_MAX 32767
#define SIZE_CB_R 80
#define SIZE_CB_T 64
#define CHASSIS_LENGTH 0.5
#define WHEEL_RADIUS 0.2
#define CONTROLLED 0
#define TIMEOUT 1
#define HALT 2
#define FCY 1843200
#define BAUD_RATE 4800


// GLOBAL VARIABLES ============================================================
typedef struct{
    int n;              // Heartbeats passed from last period
    int N;              // Period in heartbeat units
    void (*fptr)();     // Function of the task to be executed every period
} heartbeat;

typedef struct{
    int kw;             // Writing index
    int kr;             // Reading index
    char* buffer;       // Buffer
} circular_buffer;

typedef struct {
    double angular_vel;    // Angular velocity
    double linear_vel;     // Linear velocity 
    int print_flag;     // Flag determining if the velocities should be printed
} reference_data;

typedef struct {
    double rpm_request;         // Requested angular velocity in RPM for a wheel
    double rpm_applied;          // Applied angular velocity in RPM for a wheel
} rpm;

typedef struct {
    double sum;    // Float that contains the partial sum of temperatures
    double avg;    // Float that contains the averaged temperature over the last 10 readings
    int k;         // Index to keep in account the number of temperature acquisitions
} temperature;

circular_buffer cb_r;           // Receiver circular buffer
circular_buffer cb_t;           // Transmitter circular buffer
reference_data rd;              // Reference angular and linear velocity
heartbeat schedInfo[MAX_TASKS]; // Scheduler of the tasks
rpm wheels[2];                  // RPM required/applied for the wheels
parser_state pstate;            // Struct containing fields related to the parser-function's state
temperature adc_temp;           // Temperature analysis fields

int send_alert = 0;             // Flag notifying if at least one of the two required angular velocities of the wheels have been saturated
int alert_active = 0;           // Flag notifying if the alert is already active 
int mode = CONTROLLED;          // Mode of the microcontroller that starts in CONTROLLED


// PWM SETUP PERIOD FUNCTION ===================================================
void pwm_setup_period(int ms){
    // FCY = 7372800/4 = 1843200 Hz
    // In 1 second there would be 1843200 clocks steps
    // FPWM = 1 KHz, therefore PTPER should count as many clock steps as the one corresponding to 1 ms (1842.2 clock steps)
    // This can be easily put in a 15 bit register (max 32767)
    // Therefore it is enough to set a prescaler of 1:1 -> then we have 1842.2/1 = 1842.2 clock steps
    
    double PTPER_no_prescale = FCY * (ms/1000.0); // in our case ms = 1 since Fpwm should be 1 kHz -> FCY/1000 = 1843.200
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

// UTILS FUNCTIONS  ============================================================
double extract_number(const char* str) {
    int i = 0;
    int j = 1;
    double number = 0;
    double decimal = 0;
    int sign = 1;

    if (str[i] == '-'){
        sign = -1;
        i++;
    }
    else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    
    while (str[i] != '.' && str[i] != ',' && str[i] != '\0') { // if neither a comma nor the end of the string is parsed
        number *= 10; // multiply the current number by 10;
        number += str[i] - '0'; // converting character to decimal number
        i++;
    }
    if (str[i] == '.'){
        i++;
        while (str[i] != ',' && str[i] != '\0') { // if neither a comma nor the end of the string is parsed
            decimal *= 10; // multiply the current number by 10;
            decimal += str[i] - '0'; // converting character to decimal number
            i++;
            j = j*10;
        }
        decimal = decimal/j;
    }
    return sign*(number+decimal);
}

int next_value(const char* msg, int i) {
    while ((msg[i] != ',') && (msg[i] != '\0')){ 
        i++;
    }
    if (msg[i] == ','){
        i++;
    }
    return i;
}

void parse_hlref(const char* msg, reference_data* rdata) {
    int i = 0;
    rdata->angular_vel = extract_number(msg);
    i = next_value(msg, i);
    rdata->linear_vel = extract_number(msg + i);
    
    if (rdata->angular_vel > 9.99)
        rdata->angular_vel = 9.99;
    else if (rdata->angular_vel < -9.99)
        rdata->angular_vel = -9.99;
    if (rdata->linear_vel > 9.99)
        rdata->linear_vel = 9.99;
    else if (rdata->linear_vel < -9.99)
        rdata->linear_vel = -9.99;
                
}

void process_hlref(){   // Updates (if not in HALT mode) and prints the current RPM or reference data values.
    
    // If is not in HALT mode, analyzes the reference data and applies the correct RPM, activating the alert (task5) if a saturation occurs.
    if (mode != HALT){

        int saturation_occurred[2];

        if (mode == TIMEOUT){   // if the current mode is 'timeout'
            mode = CONTROLLED;  // enter 'controlled' mode
            LATBbits.LATB1 = 0; // stop blinking D4 when exiting timeout mode
        }

        // Start Timer4 to eventually enter 'timeout' mode
        IFS1bits.T4IF = 0;      // reset Timer4 interrupt flag
        TMR4 = 0;               // empty the Timer4 'counter' register
        IEC1bits.T4IE = 1;      // enable Timer4 interrupt

        // Parse the reference msg containing angular/linear velocities of the vehicle
        parse_hlref(pstate.msg_payload, &rd);

        // Formulas to obtain the angular velocity of the left wheel [0] and the right wheel [1] in rpm starting from the angular velocity of the vehicle
        wheels[0].rpm_request = ((rd.linear_vel + rd.angular_vel*(CHASSIS_LENGTH/2))/WHEEL_RADIUS)* 9.55; 
        wheels[1].rpm_request = ((rd.linear_vel - rd.angular_vel*(CHASSIS_LENGTH/2))/WHEEL_RADIUS)* 9.55;

        for(int i = 0; i<2; i++){
          if (wheels[i].rpm_request >= -50 && wheels[i].rpm_request <= 50){
              wheels[i].rpm_applied = wheels[i].rpm_request;
          } 
          else if (wheels[i].rpm_request < -50){
              wheels[i].rpm_applied = -50; // saturate
																											   
              saturation_occurred[i] = 1;
          }
          else{
              wheels[i].rpm_applied = 50; // saturate
																											   
              saturation_occurred[i] = 1;
          }
        } 

        // If there is a saturation and the alert is not active, starts the alert task
        // Else if there is no saturation and the alert is active, stops the alert task
        if ((saturation_occurred[0]||saturation_occurred[1]) && !send_alert) {
            schedInfo[5].n = schedInfo[5].N;
            send_alert = 1;
        }
        else if (!(saturation_occurred[0]||saturation_occurred[1]) && send_alert) {
            schedInfo[5].n = 0;
            send_alert = 0;
        }
    }
}

void process_hlena(){
    if (mode == HALT){
        mode = CONTROLLED;
        IFS1bits.T4IF = 0;  // reset Timer4 interrupt flag
        TMR4 = 0;           // empty Timer4 'counter' register
        IEC1bits.T4IE = 1;  // enable Timer4 interrupt flag

        LATBbits.LATB1 = 0; // stop blinking D4 when exiting timeout mode
        // Prepare and transmit MCACK message to confirm the mode switch HALT -> CONTROLLED
        char msg[13];                   
        sprintf(msg,"$MCACK,ENA,1*");
        uart_transmit(msg);
    }
}

void uart_transmit(const char *msg){    // Function to trasmit a message through the UART 
    
    int transmitter_full_flag = 0;
    // Store the message into the 'transmitter' circular buffer
    for(int i=0; i<strlen(msg); i++){
        IEC1bits.U2TXIE = 0;                        // disable UART2 Transmitter interrupt to safely access the 'transmitter' circular buffer variables
        transmitter_full_flag = (cb_t.kw+1)%SIZE_CB_T == cb_t.kr;
        IEC1bits.U2TXIE = 1;                        // disable UART2 Transmitter interrupt to safely access the 'transmitter' circular buffer variables
        if(!transmitter_full_flag){   // if the 'transmitter' circular buffer is not full
            cb_t.buffer[cb_t.kw] = msg[i];      // put the i char in the kw position of the circular buffer 
            cb_t.kw = (cb_t.kw+1)%SIZE_CB_T;    // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }
    
    IEC1bits.U2TXIE = 0;                        // disable UART2 Transmitter interrupt to safely access the 'transmitter' circular buffer variables
    // Fill the 'transmitter' FIFO to trigger the 'transmitting' mechanism
    while(U2STAbits.UTXBF == 0){                // while the 'transmitter' FIFO is not full
        if(cb_t.kr != cb_t.kw){                 // if the 'transmitter' circular buffer is not empty
            U2TXREG = cb_t.buffer[cb_t.kr];     // write the the content of the current cell of the circular buffer into the UART2 'transmitter' fifo
            cb_t.kr = (cb_t.kr+1)%SIZE_CB_T;    // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }
    IEC1bits.U2TXIE = 1;                        // re-enable UART2 Transmitter interrupt 
}

void stop_motors(){
    rd.linear_vel = 0;
    rd.angular_vel = 0;
    wheels[0].rpm_applied = 0;
    wheels[1].rpm_applied = 0;  
    PDC2 = (int)((50/100.0)*2*PTPER); // change the duty cycle of the first PWM signal to 50% (stop motor)
    PDC3 = (int)((50/100.0)*2*PTPER); // change the duty cycle of the second PWM signal to 50% (stop motor)
    send_alert = 0;
}

// TASKS FUNCTIONS =============================================================
void task0_receiver(){      // read the bytes stored into the circular_buffer_r

    IEC1bits.U2RXIE = 0;                                    // disable UART2 Receiver interrupt
    while (cb_r.kr != cb_r.kw) {                            // while the 'receiver' circular buffer is not empty,
        IEC1bits.U2RXIE = 1;    // re-enable UART2 Receiver interrupt
        char byte = cb_r.buffer[cb_r.kr];                   // reads the byte stored
        if (parse_byte(&pstate, byte) == NEW_MESSAGE) {     // if I filled up the msg_payload field of the pstate structure with the payload of a new message
            if (strcmp(pstate.msg_type, "HLREF") == 0) {
                process_hlref();
            }
            
            else if (strcmp(pstate.msg_type, "HLENA") == 0){
                process_hlena();
            }
        }     
        cb_r.kr = (cb_r.kr+1)%SIZE_CB_R;        // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        IEC1bits.U2RXIE = 0;                                    // disable UART2 Receiver interrupt
    }
    IEC1bits.U2RXIE = 1;    // re-enable UART2 Receiver interrupt
    
    IEC1bits.INT1IE = 0; // disable S6 button interrupt to safely access 'print_flag'
    if(rd.print_flag == 0){
        // print the wheels' angular velocities on the LCD every time that a new HLREF message is read
        clear_row(SECOND_ROW,2,7);
        print_float_digits(SECOND_ROW,2,wheels[0].rpm_applied);
        clear_row(SECOND_ROW,9,14);
        print_float_digits(SECOND_ROW,9,wheels[1].rpm_applied);
    }
    else{
        // print the reference angular velocity and speed on the LCD every time that a new HLREF message is read
        clear_row(SECOND_ROW,2,7);
        print_float_digits(SECOND_ROW,2,rd.angular_vel);
        clear_row(SECOND_ROW,9,14);
        print_float_digits(SECOND_ROW,9,rd.linear_vel);
    }
    IEC1bits.INT1IE = 1; // enable S6 button interrupt
    

    // update the duty-cycle of the two PWM signals every time the task0 is executed
    PDC2 = (int) ((((wheels[0].rpm_applied + 60)/1.2)/100.0)*2*PTPER); // change the duty cycle of the first PWM signal based on the rpm value
    PDC3 = (int) ((((wheels[1].rpm_applied + 60)/1.2)/100.0)*2*PTPER); // change the duty cycle of the second PWM signal based on the rpm value
        
    // print the current mode status on the LCD
    place_cursor(FIRST_ROW, 7);
    while(SPI1STATbits.SPITBF == 1);        // wait until not full -> if this bit is 1 the buffer is full
    if (mode == TIMEOUT)
        SPI1BUF = 'T';                      // print 'T' on the LCD
    else if (mode == CONTROLLED)
        SPI1BUF = 'C';
    else if (mode == HALT)
        SPI1BUF = 'H';
    
}

void task1_d3(){ // turn on/off the D3 LED
    LATBbits.LATB0 = !LATBbits.LATB0; // change the D3 LED state
}

void task2_adc(){ // convert the signal (temperature and potentiometer: automatic sampling, automatic conversion)
    
    int ADCValue;           // int (16 bit) that contains the converted value for the second channel (temperature)
    double VoltValue;       // float that contains the voltage corresponding to the converted value for the second channel (temperature)
    double MilliVoltValue;  // float that contains the voltage corrisponding to the converted value in millivolt
    double CelsiusValue;    // float that contains the temperature corrisponding to the converted value
    
    while (ADCON1bits.DONE == 0);   // while the conversion is not finished wait
    ADCValue = ADCBUF0;             // get ADC value for the first channel (temperature sensor)   

    
    VoltValue=(ADCValue*5)/1023.0;                  // since the volts go from 0 to 5 
    MilliVoltValue = VoltValue*1000;                // since the relationship holds between voltage (expressed in millivolt) and temperature (expressed in celsius)
    CelsiusValue = ((MilliVoltValue-750)/10.0)+25;  // y-y0 = m(x-x0) -> y-750 = 10(x-25) -> x-25 = (y-750)/10 -> x = (y-750)/10 + 25
    
    adc_temp.sum = adc_temp.sum + CelsiusValue;
    adc_temp.k = adc_temp.k+1;

    if(adc_temp.k == 10){
        adc_temp.avg = adc_temp.sum / 10;
        adc_temp.sum = 0;
        adc_temp.k = 0;
    }
        
}

void task3_d4(){ // turn on/off the D4 LED
    LATBbits.LATB1 = !LATBbits.LATB1; // change the D4 LED state
}

void task4_mctem(){ // send the MCTEM message to the PC at 1 Hz frequency
                    
    //Prepare and transmit the MCTEM message containing the average temperature
    char msg[14]; // max MCTEM message length: 14 characters
    sprintf(msg,"$MCTEM,%.2f*",adc_temp.avg);
    uart_transmit(msg);

}

void task5_mcale(){ // send the MCALE message to the PC at 1 Hz frequency
    
    // Prepare and trasmit the MCALE message containing the required RPMs that caused saturation
    char msg[21]; // max MCALE message length: 21 characters
    sprintf(msg,"$MCALE,%.2f,%.2f*",wheels[0].rpm_request,wheels[1].rpm_request);
    uart_transmit(msg);
}

void task6_mcfbk(){ // send the MCFBK message to the PC at 5 Hz frequency
    
    // Prepare and transmit the MCFBK message containing the current RPMs values and mode
    char msg[24]; // max MCFBK message length: 24 characters
    sprintf(msg,"$MCFBK,%.2f,%.2f,%d*",wheels[0].rpm_applied ,wheels[1].rpm_applied,mode);
    uart_transmit(msg);
}

// TASK SCHEDULER FUNCTION =====================================================
void scheduler(){
    for(int i=0; i<MAX_TASKS;i++){
        schedInfo[i].n++;
        if(schedInfo[i].n >= schedInfo[i].N){
            if(i != 3 && i != 5){
                schedInfo[i].fptr();    // call the function corresponding to the i-th task
                schedInfo[i].n = 0;     // reset the loop counter 
            }
            else if (i == 3 && mode == TIMEOUT){
                schedInfo[i].fptr();    // call the function corresponding to the i-th task
                schedInfo[i].n = 0;     // reset the loop counter 
            }
            else if (i == 5 && send_alert == 1){
                schedInfo[i].fptr();    // call the function corresponding to the i-th task
                schedInfo[i].n = 0;     // reset the loop counter 
            }
        }
    }
}

// INTERRUPT HANDLERS ==========================================================
void __attribute__((__interrupt__,__auto_psv__))  _U2RXInterrupt(){ // Interrupt handler -> event: UART2 Receiver FIFO gets one of the cells filled up
    IFS1bits.U2RXIF = 0; // reset interrupt flag
    
    // fill the 'receiver' circular buffer with bytes from the 'receiver' FIFO until it is empty
    while(U2STAbits.URXDA == 1){ // while the 'receiver' FIFO is not empty
        if((cb_r.kw+1)%SIZE_CB_R != cb_r.kr){ // if the 'receiver' circular buffer is not full
            cb_r.buffer[cb_r.kw] = U2RXREG; // read the character and store it in the current cell of the circular buffer
            cb_r.kw = (cb_r.kw+1)%SIZE_CB_R; // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }
        
}

void __attribute__((__interrupt__,__auto_psv__))  _U2TXInterrupt(){ // Interrupt handler -> event: UART2 Transmitter FIFO gets one of the cells freed up
    IFS1bits.U2TXIF = 0; // reset interrupt flag
    
    // fill the 'transmitter' FIFO with bytes from the 'transmitter' circular buffer until it is full
    while(U2STAbits.UTXBF == 0){ // while the 'transmitter' FIFO is not full
        if(cb_t.kr != cb_t.kw){ // if the 'transmitter' circular buffer is not empty
            U2TXREG = cb_t.buffer[cb_t.kr]; // write the the content of the current cell of the circular buffer into the UART2 'transmitter' fifo
            cb_t.kr = (cb_t.kr+1)%SIZE_CB_T; // increment the index and evaluate the remainder of the division by the size of the circular buffer to avoid exceeding bounds
        }
        else{
            break;
        }
    }
}

// S5 BUTTON INTERRUPT HANDLER 
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){ // Interrupt handler -> event: S5 button pushed
    IFS0bits.INT0IF = 0; // reset interrupt flag
    
    IFS0bits.T2IF = 0;  // reset Timer2 interrupt flag
    TMR2 = 0;           // empty the Timer2 'counter' register
    
    IEC0bits.T2IE = 1;  // enable Timer2 interrupt
    
}

// TIMER2 INTERRUPT HANDLER (to avoid button S5 bounces)
void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(){ // Interrupt handler -> event: Timer2 elapsed
    IFS0bits.T2IF = 0; //reset Timer2 interrupt flag

    if (PORTEbits.RE8 == 1){ // if button S5 has been released
        
        stop_motors();
        LATBbits.LATB1 = 0;  // stop blink d4 
     
        mode = HALT;         // enter 'safe' mode       
        
        IFS1bits.T4IF = 0;  // reset Timer4 interrupt flag
        TMR4 = 0;           // empty Timer4 counter register
        IEC1bits.T4IE = 0;  // disable Timer4 interrupt
    }
    
    IEC0bits.T2IE = 0; // disable Timer2 interrupt    
}

// S6 BUTTON INTERRUPT HANDLER 
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(){ // Interrupt handler -> event: S6 button pushed
    IFS1bits.INT1IF = 0; // reset interrupt flag
    
    IFS0bits.T3IF = 0;  // reset Timer3 interrupt flag
    TMR3 = 0; // empty the Timer3 'counter' register
    
    IEC0bits.T3IE = 1;  // enable Timer3 interrupt
    
}

// TIMER3 INTERRUPT HANDLER (to avoid button S6 bounces)
void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(){ // Interrupt handler -> event: Timer3 elapsed
    IFS0bits.T3IF = 0; //reset Timer3 interrupt flag

    if (PORTDbits.RD0 == 1){ // if button S6 has been released
        rd.print_flag = !rd.print_flag; // change the value of 'print_flag' in the reference data struct
    }
    
    IEC0bits.T3IE = 0; // disable Timer3 interrupt    
}

// TIMER4 INTERRUPT HANDLER (to eventually enter 'timeout' mode)
void __attribute__((__interrupt__,__auto_psv__)) _T4Interrupt(){ // Interrupt handler -> event: Timer4 elapsed
    IFS1bits.T4IF = 0; //reset Timer4 interrupt flag

    stop_motors();  // stop the motors
    
    mode = TIMEOUT; // enter 'timeout' mode

    IEC1bits.T4IE = 0; // disable Timer4 interrupt    
}

// MAIN FUNCTION ===============================================================
int main(void) {
    
    tmr_wait_ms(TIMER1,1000); // wait for the LCD to set-up
    
    // Parser Inintialisation 
    pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    
    // UART2 SETUP
    U2BRG = ((FCY/16)/BAUD_RATE)-1;     // [FCY/(16*desired baud rate)]-1 with desired baud rate = 9600 baud al secondo, 10 baud (1 start-bit, 8 bit di dato, 1 stop-bit) = 1 byte di dato -> baud rate = 9600, bit rate = 9600/10 = 960 hz
    U2MODEbits.UARTEN = 1;              // enable the UART2 peripheral
    U2STAbits.UTXEN = 1;                // enable the transmission (on the board side)
    U2STAbits.URXISEL = 1;              // specify that the Interrupt Flag of the receiver is set to 1 whenever it receives a character
    U2STAbits.UTXISEL = 0;              // specify that the Interrupt Flag of the transmitter is set to 1 when a byte of the transmitter FIFO is transmitted

        
    // UART2 INTERRUPT SETUP
    IEC1bits.U2RXIE = 0; // initially disable UART2 Receiver interrupt
    IEC1bits.U2TXIE = 0; // initally disable UART2 Transmitter interrupt
    IFS1bits.U2RXIF = 0; //reset interrupt flag (UART2 Receiver)
    IFS1bits.U2TXIF = 0; //reset interrupt flag (UART2 Transmitter)
    
    // TIMER 2 SETUP (to avoid button S5 bounces)
    IEC0bits.T2IE = 0; // disable T2 interrupt (TIMER2)
    IFS0bits.T2IF = 0; //reset interrupt flag (TIMER2)
    
    // TIMER 3 SETUP (to avoid button S5 bounces)
    IEC0bits.T3IE = 0; // disable T3 interrupt (TIMER3)
    IFS0bits.T3IF = 0; //reset interrupt flag (TIMER3)
    
    // TIMER 4 SETUP 
    IEC1bits.T4IE = 0; // disable T4 interrupt (TIMER4)
    IFS1bits.T4IF = 0; //reset interrupt flag (TIMER4)
    
    // S5 INTERRUPT SETUP
    TRISEbits.TRISE8 = 1; // S5 button as input
    IEC0bits.INT0IE = 1; // enable INT0 interrupt (S5)
    IFS0bits.INT0IF = 0; //reset interrupt flag (S5)
    
    // S6 INTERRUPT SETUP
    TRISDbits.TRISD0 = 1; // S6 button as input
    IEC1bits.INT1IE = 1; // enable INT1 interrupt (S6)
    IFS1bits.INT1IF = 0; //reset interrupt flag (S6)
    
    // SPI SETUP
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE =6; // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI peripheral
    
    // LED SETUP
    TRISBbits.TRISB0 = 0; // D3 LED as output
    TRISBbits.TRISB1 = 0;   //D4 LED as output

    // ADC SETUP (automatic sampling - automatic conversion)
    // THE RESULT OF THE CHANNEL CONVERSION GOES INTO ADCBUF0
    ADCON3bits.ADCS = 8; // selects how long is one Tad: in this case it is 8*Tcy
    // Conversion time duration (Tconv): fixed to 12 Tad
    ADCON1bits.ASAM = 1; // set how the sampling begins as automatic
    ADCON3bits.SAMC = 16; // Sample time Tsamp 16 Tad -> so I don't need to set SAMP to 0 to make the (sampling end) and the conversion begin
    ADCON1bits.SSRC = 7; // set how the sampling ends and conversion begins as automatic
    ADCON2bits.CHPS = 1; // set the number of channels to 1
    ADCHSbits.CH0SA = 3; // choose the positive input to channel 0 - in particular select pin AN3 (0011) where the temperature sensor is connected 
    ADPCFG = 0xFFFF; // first set all the bits to 1
    ADPCFGbits.PCFG3 = 0;  // then set the one corresponding to AN3 to 0 
    // selects which pin should be used for A/D (if the bit is 0 the corresponding pin will be used for ADC)
    ADCON1bits.ADON = 1; // turn on the ADC
    
    // PWM signal setup
    PTCONbits.PTMOD = 0; // set the free-running mode (if PTMR<PDC, the signal is 1 (high), otherwise it goes down)
    PWMCON1bits.PEN2L = 1; // PWM2L pin is enabled for PWM output (low)
    PWMCON1bits.PEN2H = 1; // PWM2H pin is enabled for PWM output (high)
    PWMCON1bits.PEN3L = 1; // PWM3L pin is enabled for PWM output (low)
    PWMCON1bits.PEN3H = 1; // PWM3H pin is enabled for PWM output (high)
    pwm_setup_period(1); // since the Fpwm should be 1 KHz, the period that I count should be of 1 ms, therefore 1843.2 clock cycles
    PDC2 = (int) ((50/100.0)*2*PTPER); // initialize the first signal to a 50% Duty Cycle (the low takes half the Tpwm period) 
    PDC3 = (int) ((50/100.0)*2*PTPER); // initialize the second signal to a 50% Duty Cycle (the low takes half the Tpwm period) 
    // -> this is considered in the hypothesis that:
    // * 0/100 duty cycle corresponds to an angular velocity for the wheels of -60 rpm
    // * 100/100 duty cycle corresponds to an angular velocity for the wheels of 60 rpm
    DTCON1bits.DTAPS = 1; // set the dead time unit to 2*TCY = 2*(1/1843200) = 2*54.25*1e-8 s = 108.5*1e-8 = 1.085*1e-6
    DTCON1bits.DTA = 3; // set how many time units the dead time should last (3 -> dead time of around 1.085*3*1e-6 = 3.255us)
    PTCONbits.PTEN = 1; // enable the PWM Time Base Timer
    
    // INITIALIZE BUFFERS
    // Define two char buffers in memory
    char buffer_r[SIZE_CB_R];
    char buffer_t[SIZE_CB_T];
    // Link the char pointers to each one of these buffers
    cb_r.buffer = buffer_r;
    cb_t.buffer = buffer_t;
    
    // INITIALIZE TASKS
    schedInfo[0].n = 0;
    schedInfo[0].N = 1;                 // T = 100 ms -> 10 Hz
    schedInfo[0].fptr = task0_receiver; // read from cb_r for HLREF messages and refresh the PWM value task
    
    schedInfo[1].n = 0;
    schedInfo[1].N = 10;                // T = 1000 ms -> the D3 LED should blink at 1 Hz 
    schedInfo[1].fptr = task1_d3;       // D3 blinking task
    
    schedInfo[2].n = 0;
    schedInfo[2].N = 1;                 // T = 100 ms -> the micro should acquire the temperature at 10 Hz
    schedInfo[2].fptr = task2_adc;      // ADC and store in cb_t task
    
    schedInfo[3].n = 0;
    schedInfo[3].N = 2;                 // T = 200 ms -> the D4 LED should blink at 5 Hz 
    schedInfo[3].fptr = task3_d4;       // D4 blinking task
    
    schedInfo[4].n = 0;
    schedInfo[4].N = 10;                // T = 1000 ms -> send MCTEM message at 1 Hz
    schedInfo[4].fptr = task4_mctem;    // send MCTEM message task (synchronous)
    
    schedInfo[5].n = 0;
    schedInfo[5].N = 10;                // T = 1000 ms -> send MCALE message at 1 Hz
    schedInfo[5].fptr = task5_mcale;    // send MCALE message task (asynchronous)
    
    schedInfo[6].n = -1;                // se sfaso di 100 ms rispetto al task precedente, avrò al minimo 100 ms di distacco tra i due (ciò vuol dire 98 bytes liberati)
    schedInfo[6].N = 2;                 // T = 200 ms -> send MCFBK message at 5 Hz (960/5 = 192 bytes ogni 200 ms liberati da UART2, 23 bytes scritti)
    schedInfo[6].fptr = task6_mcfbk;    // send MCFBK message task (synchronous)

    // PRINT "STATUS:C" IN THE FIRST ROW OF THE LCD and "R: 0; 0" IN THE SECOND ONE AS AN INITIALISATION
    char *str1 = "STATUS:C";
    print_string(str1);
    char *str2 = "R:-00.00;-00.00";
    place_cursor(SECOND_ROW,0);
    print_string(str2);
    
    // START TIMERS
    tmr_setup_period(TIMER1, 100);  // heartbeat: 100ms -> 10 Hz
    tmr_setup_period(TIMER2,10);    // 10ms -> 100 Hz Timer2 to avoid button S5 bounces
    tmr_setup_period(TIMER3,10);    // 10ms -> 100 Hz Timer3 to avoid button S6 bounces
    tmr_setup_period(TIMER4, 5000); // 5000ms -> 0.2 Hz Timer4 to eventually enter 'timeout' mode
    
    // START COUNTING TO EVENTUALLY ENTER IN TIMEOUT MODE
    IFS1bits.T4IF = 0;  // reset Timer4 interrupt flag
    TMR4 = 0; // empty the Timer4 'counter' register
    IEC1bits.T4IE = 1;  // enable Timer4 interrupt
    
    while(1){        
        scheduler();
        tmr_wait_period(TIMER1);
    }
             
    return 0;
}