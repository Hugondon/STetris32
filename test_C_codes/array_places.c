#include <stdio.h>

int main(int argc, char * argv[]){
    int array[2][3] = {
        {0, 1, 2},
        {3, 4, 5}
    };

    // [VERTICAL MOVEMENT][HORIZONTAL MOVEMENT]

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 3; j++){
            printf("%d ", array[i][j]);
        }
        printf("\n");
    }
}