#ifndef QUEUE_H_
#define QUEUE_H_

#include <stdint.h>
#include <stdbool.h>


/* DEFINES */
#define DEFAULT_QUEUE_SIZE      (uint16_t ) 30


/* PRIVATE VARIABLES */
typedef struct Queue queue_t;

// extern void * is used to protect variables from being changed outside of queue source file
// Acts like the private keyword in C++

extern void *head_v;
extern void *tail_v;

extern void *max_size_v;
extern void *curr_size_v;


/* QUEUE ITEM STRUCT */
// Queue node will store the midi message and the address of the next node
struct Queue {
    uint8_t message[4];
    queue_t *next;
};


/* INITIALIZATION FUNCTIONS */
// A max size needs to be given when initializing the queue. Default value is defined in DEFAULT_QUEUE_SIZE
void Queue_Init (uint16_t size);
// Deallocates all memory used by the queue
void Queue_Deinit ();


/* PROCESS FUNCTIONS */
// Adds an message to the end of a queue. Returns true if the message was added properly. Returns false if queue is full or if any other error occurred
bool Enqueue (uint8_t *message);
// Pops head node from queue. Returns message if one is in the queue, else returns an UINT8_MAX message [ 255, 255, 255, 255 ]
uint8_t *Dequeue ();

// Return message in the fron the of the queue if one is in the queue, else return INT8_MAX message [ 255, 255, 255, 255 ]
uint8_t *Front ();


/* PRINTING FUNCTION */
// Prints the messages in the queue
void Print_Queue ();

#endif /* QUEUE_H_ */
