/*
 * Queue_GSM.h
 *
 *  Created on: Nov 20, 2024
 *      Author: Admin
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

/*
 * Queue_GSM.h
 *
 * Created on: Dec 8, 2024
 * Author: Admin
 */

#include <stdint.h>

// Maximum size of the queue
#define MAX_SIZE 128

// Queue structure
typedef struct {
    uint32_t data[MAX_SIZE]; // Array to store queue elements
    int front;               // Points to the front of the queue
    int rear;                // Points to the rear of the queue
    int size;                // Current size of the queue
} Queue_GSM;

// Function declarations
void initQueue_GSM(Queue_GSM* q);
int isEmpty_GSM(Queue_GSM* q);
int isFull_GSM(Queue_GSM* q);
void enqueue_GSM(Queue_GSM* q, uint32_t value);
uint32_t dequeue_GSM(Queue_GSM* q);
uint32_t peek_GSM(Queue_GSM* q);
void clearQueue_GSM(Queue_GSM* q);
int printQueue_GSM(Queue_GSM* q);
int checkAddrExistInQueue(uint32_t addr, Queue_GSM* queue);
void deleteMiddle_GSM(Queue_GSM* q, int indexToDelete);

#endif // QUEUE_GSM_H

