#include "utils.h"

uint16_t cntTimer5ms = 0;

Point newPoint(double lat, double lon){
	Point temp;
	temp.lat = lat;
	temp.lon = lon;
	return temp;
}


float calDistance(Point A, Point B) {
	float phi1 = A.lat * M_PI / 180;
	float phi2 = B.lat * M_PI / 180;
	float deltaLamda = (B.lon - A.lon) * M_PI / 180;
	return acos(sin(phi1) * sin(phi2) + cos(phi1) * cos(phi2) * cos(deltaLamda)) * R;
}

float calBearing(Point A, Point B) {
	float phi1 = A.lat * M_PI / 180;
	float phi2 = B.lat * M_PI / 180;
	float deltaPhi = (B.lat - A.lat) * M_PI / 180;
	float deltaLamda = (B.lon - A.lon) * M_PI / 180;

	float angle = atan2(sin(deltaLamda) * cos(phi2),
			cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLamda));
	angle = (angle * 180 / M_PI + 360);
	angle = angle - ((int) angle / 360) * 360;
	return angle;
}

void resetTimer(){
	cntTimer5ms = 0;
}
uint16_t getTimer5ms(){
	return cntTimer5ms;
}
