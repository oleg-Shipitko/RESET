#ifndef _INIT_INCLUDED_
#define _INIT_INCLUDED_



void initAll(void); // That's all!
void clear_ext_interrupt(unsigned char pin);
void add_ext_interrupt(unsigned char pin, char edge);

#endif
