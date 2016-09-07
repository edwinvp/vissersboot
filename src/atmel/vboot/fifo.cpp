#include "fifo.h"
// ----------------------------------------------------------------------------
volatile unsigned char head = 0,tail = 0;
#define FIFO_SIZE 32
#define FIFO_MASK (FIFO_SIZE-1)
volatile char uart_fifo[FIFO_SIZE];
// ----------------------------------------------------------------------------
bool fifo_avail()
{
    return (head != tail);
}
// ----------------------------------------------------------------------------
char fifo_read()
{
    char data(0);
    if (head != tail) {
        data = uart_fifo[tail];
        unsigned char new_tail(tail);
        new_tail++;
        new_tail &= FIFO_MASK;
        tail = new_tail;
    }
    return data;
}
// ----------------------------------------------------------------------------
void fifo_write(unsigned char c)
{
    // Store byte in FIFO (if FIFO isn't full)
    unsigned char new_head;
    new_head = head + 1;
    new_head &= FIFO_MASK;

    if (new_head != tail) {
        uart_fifo[head] = c;
        head = new_head;
    }
}
// ----------------------------------------------------------------------------
