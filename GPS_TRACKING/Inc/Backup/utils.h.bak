#ifndef		_utils_h_
#define		_utils_h_
#include "main.h"
#include <math.h>

#define R 6371000
#define M_PI 3.14159265358979323846

extern uint16_t cntTimer5ms;

typedef struct point{
	double lat;
	double lon;
}Point;


Point newPoint(double lat, double lon);
float calDistance(Point A, Point B);
float calBearing(Point A, Point B);
void resetTimer();
uint16_t getTimer5ms();

#endif
