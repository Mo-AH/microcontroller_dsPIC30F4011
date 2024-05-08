/*
 * File:   spi_functions.c
 * Author: Salvatore D'ippolito, Mohammad Al Horany, Francesco Ferrazzi, Emanuele Rambaldi
 *
 * Created on 15 novembre 2022, 11.30
 */

#include "spi_functions.h"

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

void print_char_counter(int row, int start, unsigned char char_counter){
    place_cursor(row,start);
    
    // WRITE THE NUMBER OF RECEIVED CHARACTERS ON THE LCD
    while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    SPI1BUF = (unsigned char)(char_counter/100) + '0'; // extract the hundreds digit from the char_counter
    
    while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    SPI1BUF = (unsigned char)((char_counter%100)/10) + '0'; // extract the tens digit from the char_counter
    
    while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    SPI1BUF = (unsigned char)((char_counter%10)/1) + '0'; // extract the units digit from the char_counter

}
