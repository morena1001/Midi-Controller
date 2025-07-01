#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "queue.h"


/* PRIVATE VARIABLES */
static queue_t *head;
static queue_t *tail;

static uint16_t max_size;
static uint16_t curr_size;

void *head_v =  &head;
void *tail_v =  &tail;

void *max_size_v =  &max_size;
void *curr_size_v =  &curr_size;


/* INITIALIZATION FUNCTION */
void Queue_Init (uint16_t size) {
    // Allocate memory for head and tail
    head = (queue_t *) malloc (sizeof (queue_t));
    head->next = (queue_t *) malloc (sizeof (queue_t));

    tail = (queue_t *) malloc (sizeof (queue_t));
    tail->next = (queue_t *) malloc (sizeof (queue_t));

    head = tail = NULL;

    // Initialize max_size and curr_size
    max_size = size;
    curr_size = 0;
}


/* PROCESS FUNCTIONS */
void Queue_Deinit () {
    // Free all items in queue from head to tail
    while (Dequeue () != NULL) {}

    // Free head and tail
    free (head);
    free (tail);

    // Free void * references to variables
    free (head_v);
    free (tail_v);
    free (max_size_v);
    free (curr_size_v);
}

bool Enqueue (uint8_t *message) {
    // If message is empty, return false
    if (message == NULL)    return false;

    // Create a node to store message
    queue_t *node = (queue_t *) malloc (sizeof (queue_t));
    memcpy (node->message, message, 4);
    node->next = NULL;

    // If head is NULL or curr_size is 0, node becomes both head and tail
    if (head == NULL || curr_size == 0)    head = tail = node;
    // else if max size has not been exceeded, node becomes new tail
    else if (curr_size < max_size) {
        tail->next = node;
        tail = node;
    }

    else    return false;

    curr_size++;
    return true;
}

uint8_t *Dequeue () {
    // if head is NULL or curr_size is 0, return NULL
    if (head == NULL || curr_size == 0)     return NULL;

    queue_t *temp = head;
    uint8_t *message = malloc (sizeof(uint8_t) * 4);
    memcpy (message, temp->message, 4);

    head = head->next;
    curr_size--;

    free (temp);
    return message;
}

uint8_t *Front () {
    // if head is NULL or curr_size is 0, return NULL
    if (head == NULL || curr_size == 0)     return NULL;

    uint8_t *message = malloc (sizeof(uint8_t) * 4);
    memcpy (message, head->message, 4);

    return message;
}


/* PRINTING FUNCTION */
void Print_Queue () {
    queue_t *temp = head;
    for (uint16_t i = 0; i < curr_size; i++) {
        if (temp == NULL)    break;
        printf ("%X %X %X %X\r\n", (temp->message)[0],(temp->message)[1], (temp->message)[2], (temp->message)[3]);
        temp = temp->next;
    }
    free (temp);
}
