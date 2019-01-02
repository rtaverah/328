
#include "buffer.h"
#include <avr/interrupt.h>

/*
 * Circular buffer. Made for uart input but can be used for anything.
 * 
 * somebuffer.buffer[0,0,0,0,0,head,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,tail,0,0,0,0,0]
 *                                  <-----buffer contents----------->
 * Items get added to the tail.
 * The head gets processed and incremented.
 */



/*
 * this must be called before the buffer can be used
 */
void buffer_init(struct buffer_t *buf) {
	/* init the buffer head, tail and count to 0 */
	buf->head = buf->tail = 0;
	buf->count = 0;
}


/* 
 * Must be called from interrupt.
 * Appends value to the tail of the buffer.
 */
__inline__ void buffer_append(struct buffer_t *buf, uint8_t value) {
	uint8_t t;
	if(buf->count < BUFFERLEN){ //great: we have place to push our value to
		// add value to tail
		buf->buffer[buf->tail] = value;
		
		// increment tail and count
		t = buf->tail;
		if(++t >= BUFFERLEN)
			t=0;
	
		buf->count++;
		buf->tail = t;
		
		return; // success 
	}
	return;	// sorry. no more place.
}


/*
 * Only call from userspace.
 * Second parameter is a pointer to the function that will use the item in the buffer, you
 * must provide this function. The function gets called with the byte from the buffer as a parameter.
 * Then buffer head gets incremented which effectively removes that byte from the buffer
 */
void buffer_process(struct buffer_t *buf, uint8_t (*func)(uint8_t i)) {
	uint8_t t;

	cli();	// if we put it below, gcc will not reload r->cnt so it could get corrupted
	if((buf->count != 0)){ // something is in there
		buf->count--;	// only the count could be tampered from irq....
		sei();
		
		// process the byte here, run the function with next buffer value as parameter
		(*func)(buf->buffer[buf->head]);
		
		// increment head
		t = buf->head;
		if(++t >= BUFFERLEN)
			t=0;
			
		buf->head = t;
	
	}
	sei();
	return ; // success
}
