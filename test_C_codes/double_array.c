#include <stdio.h>

int main(int argc, char * argv[]){
    int array[2][3] = {
        {0, 1, 2},
        {3, 4, 5}
    };
    int array_2[2][3] = {
        {5, 4, 3},
        {2, 1, 0}
    };

    // array[avance en Y][avance en X]
    printf("%d\n", array[1][1]);
    printf("%d\n", array[0][1]);
    
    printf("%d\n", array_2[0][1]);
    printf("%d", array_2[1][1]);

}