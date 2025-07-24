/*
 * queue.h
 *
 *  Created on: Jul 24, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_QUEUE_H_
#define INC_DRIVER_QUEUE_H_

#include "stm32wbxx_hal.h"


typedef struct {
    StepOperation op;
    int steps;
} Command;

bool enqueue_command(StepOperation op, int steps);
bool dequeue_command(Command *cmd);

#endif /* INC_DRIVER_QUEUE_H_ */
