/*
 * matrix.h
 *
 *  Created on: May 2, 2021
 *      Author: hugon
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

#include <stdint.h>

enum matrix_selection{Upper = 0, Lower = 1};
enum direction{up = 0, down = 1, left = 2, right = 3, center = 4, none = 5};

typedef uint8_t row_t;


void max_transfer_command(uint8_t address, uint8_t data);
void max_transfer_data(uint8_t address, uint8_t data, uint8_t data_2);
void max_init(void);

void shift_matrix_content(uint8_t direction);

#endif /* INC_MATRIX_H_ */
