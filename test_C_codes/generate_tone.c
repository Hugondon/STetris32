#include <stdio.h>
#include <math.h>

/*
    Program to calculate values for PSC, ARR to change PWM frequency.
    Frequencies go from C3 (130.81) to C7 (2093.00)
*/

#define FREQ_HZ 16000000        // APB1
#define NOFNOTES 49             
#define PWM_FREQUENCY_FACTOR 2  // The frequency in hz that the user will hear is half the one calculated by the program

#define NOTE_C3  0
#define NOTE_CS3 1
#define NOTE_D3  2
#define NOTE_DS3 3
#define NOTE_E3  4
#define NOTE_F3  5
#define NOTE_FS3 6
#define NOTE_G3  7
#define NOTE_GS3 8
#define NOTE_A3  9
#define NOTE_AS3 10
#define NOTE_B3  11
#define NOTE_C4  12
#define NOTE_CS4 13
#define NOTE_D4  14
#define NOTE_DS4 15
#define NOTE_E4  16
#define NOTE_F4  17
#define NOTE_FS4 18
#define NOTE_G4  19
#define NOTE_GS4 20
#define NOTE_A4  21
#define NOTE_AS4 22
#define NOTE_B4  23
#define NOTE_C5  24
#define NOTE_CS5 25
#define NOTE_D5  26
#define NOTE_DS5 27
#define NOTE_E5  28
#define NOTE_F5  29
#define NOTE_FS5 30
#define NOTE_G5  31
#define NOTE_GS5 32
#define NOTE_A5  33
#define NOTE_AS5 34
#define NOTE_B5  35
#define NOTE_C6  36
#define NOTE_CS6 37
#define NOTE_D6  38
#define NOTE_DS6 39
#define NOTE_E6  40
#define NOTE_F6  41
#define NOTE_FS6 42
#define NOTE_G6  43
#define NOTE_GS6 44
#define NOTE_A6  45
#define NOTE_AS6 46
#define NOTE_B6  47
#define NOTE_C7  48

int main(){

    int psc_array[NOFNOTES], arr_array[NOFNOTES];
    int minimum_psc = 0, minimum_arr = 0;
    float freq_note_hz[NOFNOTES];
    float relative_error = 0.00, minimum_error = 0.1, freq_note_estimate;

    for(int i = 0; i < NOFNOTES; i++) freq_note_hz[i] = PWM_FREQUENCY_FACTOR*130.81*pow(2,i*0.08333);        // Fill array with note values in hz

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



