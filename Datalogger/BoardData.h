#ifndef BOARDDATA_H_INCLUDED
#define BOARDDATA_H_INCLUDED

#include <stdlib.h>

typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
} SensorData;

typedef struct {
  uint8_t start_char = '>';
	uint32_t time;
	SensorData accel;
	SensorData gyro;
	SensorData mag;
	int32_t pressure;
} BoardData;

#endif //BOARDDATA_H_INCLUDED
