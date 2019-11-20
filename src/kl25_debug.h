#if !defined(DEBUG_H_)
#define DEBUG_H_

#include "mbed.h"

//init speed in baud
void debugInit(int speed);

//send a char
inline void debugChar(char c){
    while(!(UART0->S1 & 0x80)) {}//wait for transmit buffer empty
    UART0->D = c;// send a char 
}
//send a string
inline void debugString(const char *st)
{
  int ch;
  do
  {
    ch = *st++;
    if (ch==0) break;
    debugChar(ch);
  } while(1);
}
// converting quartet to ASCII
inline void log_hexdigit(unsigned char d)
{
  d&=15;
  if (d > 9) d+=7;
  debugChar(d+'0');
}
//send a byte in hexa
inline void debugByteHex(unsigned char b)
{ log_hexdigit(b >> 4); log_hexdigit(b); }
//send a short in hexa
inline void debugShortHex(uint16_t s)
{ debugByteHex(s >> 8); debugByteHex(s); }
//send a int in hexa
inline void debugIntHex(uint32_t i)
{ debugByteHex(i >> 24); debugByteHex(i >> 16); debugByteHex(i >> 8); debugByteHex(i); }

#endif