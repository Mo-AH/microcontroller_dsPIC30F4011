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

void clear_row(int row, int start, int end){
    place_cursor(row,start);
    
    // CLEAN 'row' PASSED AS ARGUMENT FROM 'start' TO 'end'
    for(int i = start; i<=end; i++){
        while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
        SPI1BUF = ' ';
    }
}

void print_string(char* str){
    int i;
    for(i = 0; i< strlen(str); i++){
        while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
        SPI1BUF = str[i];
    }
}

void print_char_num(int row, int start, unsigned char num){
    place_cursor(row,start);
    
    // WRITE THE NUMBER OF RECEIVED CHARACTERS ON THE LCD
    while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    SPI1BUF = (unsigned char)(num/100) + '0'; // extract the hundreds digit from the char_counter
    
    while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    SPI1BUF = (unsigned char)((num%100)/10) + '0'; // extract the tens digit from the char_counter
    
    while(SPI1STATbits.SPITBF == 1); // wait until not full -> if this bit is 1 the buffer is full
    SPI1BUF = (unsigned char)((num%10)/1) + '0'; // extract the units digit from the char_counter

}
