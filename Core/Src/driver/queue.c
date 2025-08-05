/*
 * queue.c
 *
 *  Created on: Jul 24, 2025
 *      Author: RCY
 */

#include "step.h"
#include <stdbool.h>
#include "queue.h"
//typedef struct {
//    StepOperation op;
//    int steps;
//} Command;


#define COMMAND_QUEUE_SIZE 10

Command command_queue[COMMAND_QUEUE_SIZE];
int queue_front = 0;
int queue_rear = 0;


bool enqueue_command(StepOperation op, int steps)
{
    int next = (queue_rear + 1) % COMMAND_QUEUE_SIZE;
    if (next == queue_front)
        return false; // 큐 full

    command_queue[queue_rear].op = op;
    command_queue[queue_rear].steps = steps;
    queue_rear = next;
    return true;
}

bool dequeue_command(Command *cmd)
{
    if (queue_front == queue_rear)
        return false; // 큐 empty

    *cmd = command_queue[queue_front];
    queue_front = (queue_front + 1) % COMMAND_QUEUE_SIZE;
    return true;
}
