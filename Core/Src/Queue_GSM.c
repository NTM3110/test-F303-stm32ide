/*
 * Queue_GSM.c
 *
 *  Created on: Nov 20, 2024
 *      Author: Admin
 */


#include <Queue_GSM.h>
#include <stdio.h>
#include <stdlib.h>

#include "system_management.h"

#define MAX_SIZE 11

// Function to initialize the Queue_GSM
void initQueue_GSM(Queue_GSM* q) {
    q->front = q->rear = NULL;
}

// Function to check if the Queue_GSM is empty
int isEmpty_GSM(Queue_GSM* q) {
    return q->front == NULL;
}

// Function to enqueue (add) a uint32_t value to the Queue_GSM
void enqueue_GSM(Queue_GSM* q, uint32_t value) {
    // Create a new node
    Node* newNode = (Node*)malloc(sizeof(Node));
    if (newNode == NULL) {
        Debug_printf("Memory allocation failed\n");
        return;
    }
    newNode->data = value;
    newNode->next = NULL;

    // If the Queue_GSM is empty, set both front and rear to the new node
    if (isEmpty_GSM(q)) {
        q->front = q->rear = newNode;
    } else {
        // Add the new node at the end and move the rear to the new node
        q->rear->next = newNode;
        q->rear = newNode;
    }
}

// Function to dequeue (remove) a uint32_t value from the Queue_GSM
uint32_t dequeue_GSM(Queue_GSM* q) {
    if (isEmpty_GSM(q)) {
        Debug_printf("Queue_GSM is empty\n");
        return 0; // Return a default value if the Queue_GSM is empty
    }

    // Remove the front node and get its data
    Node* temp = q->front;
    uint32_t value = temp->data;
    q->front = q->front->next;

    // If the Queue_GSM is now empty, set the rear to NULL as well
    if (q->front == NULL) {
        q->rear = NULL;
    }

    // Free the memory of the dequeued node
    free(temp);
    return value;
}

// Function to peek (get) the front value without removing it
uint32_t peek_GSM(Queue_GSM* q) {
    if (isEmpty_GSM(q)) {
        Debug_printf("Queue_GSM is empty\n");
        return 0; // Return a default value if the Queue_GSM is empty
    }
    return q->front->data;
}


// Function to clear the entire Queue_GSM
void clearQueue_GSM(Queue_GSM* q) {
    while (!isEmpty_GSM(q)) {
        dequeue_GSM(q);
    }
}

// Function to print the Queue_GSM contents (for debugging)
int printQueue_GSM(Queue_GSM* q) {
    if (isEmpty_GSM(q)) {
        Debug_printf("Queue_GSM is empty\n");
        return 0;
    }
    int idx = 0;

    Node* temp = q->front;
    printf("Queue_GSM contents: \n");
    while (temp != NULL) {
        Debug_printf("Index %d: %08x\n",idx, (temp->data));
        idx++;
        temp = temp->next;
    }
    Debug_printf("\n");
    return idx;
}

int checkAddrExistInQueue(uint32_t addr, Queue_GSM *queue){
	Node* temp = queue->front;
	int idx =0;
//	Debug_printf("Queue_GSM contents: \n");
	while (temp != NULL) {
//		Debug_printf("Index %d: %08x\n",idx, (temp->data));
		if(temp->data == addr){
			Debug_printf("FOUND ADDRESS: %08lx\n", addr);
			return 1;
		}
		idx++;
		temp = temp->next;
	}
	Debug_printf("NOT FOUND ADDRESS: %08lx\n", addr);
	 return 0;
}
