/*
 * sound.h
 *
 *  Created on: May 2, 2021
 *      Author: hugon
 */

#ifndef INC_SOUND_H_
#define INC_SOUND_H_

#include <stdint.h>

#define NOFNOTES 	73
#define NOTE_C1     0
#define NOTE_CS1    1
#define NOTE_D1     2
#define NOTE_DS1    3
#define NOTE_E1     4
#define NOTE_F1     5
#define NOTE_FS1    6
#define NOTE_G1     7
#define NOTE_GS1    8
#define NOTE_A1     9
#define NOTE_AS1    10
#define NOTE_B1     11
#define NOTE_C2     12
#define NOTE_CS2    13
#define NOTE_D2     14
#define NOTE_DS2    15
#define NOTE_E2     16
#define NOTE_F2     17
#define NOTE_FS2    18
#define NOTE_G2     19
#define NOTE_GS2    20
#define NOTE_A2     21
#define NOTE_AS2    22
#define NOTE_B2     23
#define NOTE_C3     24
#define NOTE_CS3    25
#define NOTE_D3     26
#define NOTE_DS3    27
#define NOTE_E3     28
#define NOTE_F3     29
#define NOTE_FS3    30
#define NOTE_G3     31
#define NOTE_GS3    32
#define NOTE_A3     33
#define NOTE_AS3    34
#define NOTE_B3     35
#define NOTE_C4     36
#define NOTE_CS4    37
#define NOTE_D4     38
#define NOTE_DS4    39
#define NOTE_E4     40
#define NOTE_F4     41
#define NOTE_FS4    42
#define NOTE_G4     43
#define NOTE_GS4    44
#define NOTE_A4     45
#define NOTE_AS4    46
#define NOTE_B4     47
#define NOTE_C5     48
#define NOTE_CS5    49
#define NOTE_D5     50
#define NOTE_DS5    51
#define NOTE_E5     52
#define NOTE_F5     53
#define NOTE_FS5    54
#define NOTE_G5     55
#define NOTE_GS5    56
#define NOTE_A5     57
#define NOTE_AS5    58
#define NOTE_B5     59
#define NOTE_C6     60
#define NOTE_CS6    61
#define NOTE_D6     62
#define NOTE_DS6    63
#define NOTE_E6     64
#define NOTE_F6     65
#define NOTE_FS6    66
#define NOTE_G6     67
#define NOTE_GS6    68
#define NOTE_A6     69
#define NOTE_AS6    70
#define NOTE_B6     71
#define NOTE_C7     72

void custom_tone(uint32_t channel, uint8_t tone, uint8_t duration);

#endif /* INC_SOUND_H_ */
