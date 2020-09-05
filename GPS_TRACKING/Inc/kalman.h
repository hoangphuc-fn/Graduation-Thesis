#ifndef _KALMAN_H
#define _KALMAN_H

typedef struct MyKalman
{
  double Q_angle, Q_bias, R_measure;
  double K_angle, K_bias, K_rate;
  double P[2][2], K[2];
  double S, y;
  double dt, kt;
} MyKalman;

MyKalman newKalman(double angle, double bias, double measure);
double kalmanUpdate(MyKalman *kalman, double newValue, double newRate, double dt);

#endif
