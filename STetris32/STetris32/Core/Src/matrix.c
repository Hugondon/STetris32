/*
 * matrix.c
 *
 *  Created on: May 2, 2021
 *      Author: hugon
 */


#include "matrix.h"

row_t test_array[][8] = {
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00011100,0b00001000},
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000},
};
row_t figures_array[][8] = {
{0b00010001,0b10101010,0b01000100,0b00000000,0b00000000,0b00100010,0b01010101,0b10001000},
{0b00010001,0b10101010,0b01000100,0b00000000,0b00000000,0b00100010,0b01010101,0b10001000},
};
row_t matrix_buffer[8][8] = {
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000},		// Upper
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000}		// Lower
};

void max_init(void){
	max_transfer_command(0x09, 0x00);       									//  Decode Mode 	[NO]
	max_transfer_command(0x0A, 0x07);       									//  Intensity		[07]
	max_transfer_command(0x0B, 0x07);       									//  Scan Limit 		[07]
	max_transfer_command(0x0C, 0x01);       									//  Shutdown		[01] Normal
	max_transfer_command(0x0F, 0x00);      										//  No Test Display [00]
}
