#ifndef SENSOR_H
#define SENSOR_H
#include <hal_types.h>

typedef union h
{
  uint8 TEMP[4];
  struct RFRXBUF
  {
    unsigned char Head;
    unsigned char value[2];
    unsigned char Tail;
  }BUF;
}TEMPERATURE;

extern char* ReadTemp(void);

#endif