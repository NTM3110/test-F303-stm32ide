/*
 * Queue_GSM_Array.c
 *
 * Created on: Dec 8, 2024
 * Author: Admin
 */

#include <stdio.h>
#include <stdlib.h>
#include "system_management.h"
#include "Queue_GSM.h"


// Function to initialize the Queue_GSM
void initQueue_GSM(Queue_GSM* q) {
    q->front = 0;
    q->rear = -1;
    q->size = 0;
}

// Function to check if the Queue_GSM is empty
int isEmpty_GSM(Queue_GSM* q) {
    return q->size == 0;
}

// Function to check if the Queue_GSM is full
int isFull_GSM(Queue_GSM* q) {
    return q->size == MAX_SIZE;
}

// Function to enqueue (add) a uint32_t value to the Queue_GSM
void enqueue_GSM(Queue_GSM* q, uint32_t value) {
    if (isFull_GSM(q)) {
        Debug_printf("Queue_GSM is full\n");
        return;
    }
    q->rear = (q->rear + 1) % MAX_SIZE; // Circular increment
    q->data[q->rear] = value;
    q->size++;
}

// Function to dequeue (remove) a uint32_t value from the Queue_GSM
uint32_t dequeue_GSM(Queue_GSM* q) {
    if (isEmpty_GSM(q)) {
        Debug_printf("Queue_GSM is empty\n");
        return 0; // Return a default value if the Queue_GSM is empty
    }
    uint32_t value = q->data[q->front];
    q->front = (q->front + 1) % MAX_SIZE; // Circular increment
    q->size--;
    return value;
}

// Function to peek (get) the front value without removing it
uint32_t peek_GSM(Queue_GSM* q) {
    if (isEmpty_GSM(q)) {
        Debug_printf("Queue_GSM is empty\n");
        return 0; // Return a default value if the Queue_GSM is empty
    }
    return q->data[q->front];
}

// Function to clear the entire Queue_GSM
void clearQueue_GSM(Queue_GSM* q) {
	for (int i = 0; i < MAX_SIZE; i++) {
		q->data[i] = 0; // Clear the data explicitly
	}
    q->front = 0;
    q->rear = -1;
    q->size = 0;
}

// Function to print the Queue_GSM contents (for debugging)
int printQueue_GSM(Queue_GSM* q) {
    if (isEmpty_GSM(q)) {
        Debug_printf("Queue_GSM is empty\n");
        return 0;
    }
    Debug_printf("Queue_GSM contents: \n");
    for (int i = 0; i < q->size; i++) {
        int idx = (q->front + i) % MAX_SIZE;
        Debug_printf("Index %d: %08x\n", i, q->data[idx]);
    }
    Debug_printf("\n");
    return q->size;
}

// Function to check if an address exists in the Queue_GSM
int checkAddrExistInQueue(uint32_t addr, Queue_GSM* q) {
    for (int i = 0; i < q->size; i++) {
        int idx = (q->front + i) % MAX_SIZE;
        if (q->data[idx] == addr) {
            Debug_printf("FOUND ADDRESS: %08lx\n", addr);
            return 1;
        }
//        if(addr > q->data[idx] && addr < end_addr){
//        	Debug_printf("INVALID ADDRESS: %08lx\n", addr);
//        }
    }
    Debug_printf("NOT FOUND ADDRESS: %08lx\n", addr);
    return 0;
}
