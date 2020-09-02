#include "simpleInsQueue.h"

inline void Init_queue(PQUEUE Q)
{
    Q->front = 0;
    Q->tail = 0;
}

inline QData_t DeQueue(PQUEUE Q) 
{
    unsigned char PopedValue;
	
		if(Q->front ==  Q->tail)    //Queue is empty
    {
				return 0xFF;
		}
		else
    {
        PopedValue = Q->array[Q->front];
				Q->front +=1;
			
				return PopedValue;
    }
		
}

inline void EnQueue(PQUEUE Q, QData_t value)
{
   if((Q->tail+1) == Q->front)  //Queue is full
   {

   }
	 else
   {
       Q->array[Q->tail] = value;
       Q->tail +=1;
   }
}

unsigned char GetQueueLength(PQUEUE Q)
{
		if(Q->tail >= Q->front)
		{
				return (Q->tail - Q->front);
		}
		else
		{
				return (255 - Q->front + Q->tail + 1);
		}
}
