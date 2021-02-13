#include <stdio.h>
#include <stdint.h>

int main(int argc, char * argv[]){
    uint8_t t = 255;

    printf("%d\n", t);
    printf("%d\n", t>>1);
    printf("%d\n", t>>2);
    printf("%d\n", t>>3);
    printf("%d\n", t>>4);

    uint8_t array[2][3] = {
        {128, 100, 30},
        {200, 250, 12},
    };

    printf("%d\n", array[0][0]);
    printf("%d\n", array[0][0]>>1);

}