/*
 * Queue_GSM.h
 *
 *  Created on: Nov 20, 2024
 *      Author: Admin
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stdint.h>

// Define the node structure for the linked list
typedef struct Node {
    uint32_t data;            // Store uint32_t data
    struct Node* next;        // Pointer to the next node
} Node;

// Define the Queue_GSM structure with pointers to front and rear nodes
typedef struct Queue_GSM {
    Node* front;             // Front of the Queue_GSM
    Node* rear;              // Rear of the Queue_GSM
} Queue_GSM;

// Function declarations for the Queue_GSM operations
void initQueue_GSM(Queue_GSM* q);
int isEmpty_GSM(Queue_GSM* q);
void enqueue_GSM(Queue_GSM* q, uint32_t value);
uint32_t dequeue_GSM(Queue_GSM* q);
uint32_t peek_GSM(Queue_GSM* q);
void clearQueue_GSM(Queue_GSM* q);
int printQueue_GSM(Queue_GSM* q);

#endif /* INC_QUEUE_H_ */
