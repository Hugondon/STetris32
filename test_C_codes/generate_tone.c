#include <stdio.h>
#include <math.h>

/*
    Program to calculate values for PSC, ARR to change PWM frequency.
    Frequencies go from C3 (130.81) to C7 (2093.00)
*/

#define FREQ_HZ 16000000        // APB1
#define NOFNOTES 73            
#define PWM_FREQUENCY_FACTOR 2  // The frequency in hz that the user will hear is half the one calculated by the program

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

int main(){

    int psc_array[NOFNOTES], arr_array[NOFNOTES];
    int minimum_psc = 0, minimum_arr = 0;
    float freq_note_hz[NOFNOTES];
    float relative_error = 0.00, minimum_error = 0.1, first_note_freq_hz = 32.70, freq_note_estimate;

    for(int i = 0; i < NOFNOTES; i++) freq_note_hz[i] = PWM_FREQUENCY_FACTOR*first_note_freq_hz*pow(2,i*0.08333);        // Fill array with note values in hz

    // Calculate PSC and ARR that will make timer frequency match note frequency.
    for(int note = 0; note < NOFNOTES; note++){
        minimum_error = 0.1;
        for(int psc = 1; psc < 1000; psc++){
            for(int arr = 1; arr < 1000; arr++){
                freq_note_estimate = (float)(FREQ_HZ/((float)psc*(float)arr));
                relative_error = ((float)(fabs(freq_note_hz[note] - freq_note_estimate))/freq_note_estimate);
                if(relative_error < minimum_error){
                    minimum_error = relative_error;
                    minimum_psc = psc;
                    minimum_arr = arr;
                }
            }
        }
        psc_array[note] = minimum_psc;
        arr_array[note] = minimum_arr;
        // printf("Minimum for %.2lf: PSC: %d, ARR: %d and Error of %.4lf\n", freq_note_hz[note], minimum_psc, minimum_arr, minimum_error);
    }
    // Copy paste this
    printf("uint16_t psc_array[] = {");
    for(int note = 0; note < NOFNOTES - 1; note++) printf("%d, ", psc_array[note]);
    printf("%d};\n", psc_array[NOFNOTES - 1]);

    printf("uint16_t arr_array[] = {");
    for(int note = 0; note < NOFNOTES - 1; note++) printf("%d, ", arr_array[note]);
    printf("%d};\n", arr_array[NOFNOTES - 1]);

}



