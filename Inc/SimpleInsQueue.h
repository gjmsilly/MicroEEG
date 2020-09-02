#ifndef __SIMPLEINSQUEUE_H__
#define __SIMPLEINSQUEUE_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

typedef unsigned char QData_t;

// Loop Queue by natural uint8 overflow 
typedef struct Queue
{
    QData_t array[256];     
    QData_t front;                  
    QData_t tail; 
}InsQUEUE,*PQUEUE;

inline void InitQueue(PQUEUE);
void    EnQueue(PQUEUE,QData_t);
QData_t DeQueue(PQUEUE);
unsigned char GetQueueLength(PQUEUE);
		
#endif
