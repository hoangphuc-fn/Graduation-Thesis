#include "kalman.h"

MyKalman newKalman(double angle, double bias, double measure){
    MyKalman kalman;
    kalman.Q_angle = angle;
    kalman.Q_bias = bias;
    kalman.R_measure = measure;

    kalman.K_angle = 0;
    kalman.K_bias = 0;

    kalman.P[0][0] = 0;
    kalman.P[0][1] = 0;
    kalman.P[1][0] = 0;
    kalman.P[1][1] = 0;

    //kalman.kt = (double)micros();
    return kalman;
}
double kalmanUpdate(MyKalman *kalman, double newValue, double newRate, double dt){
    kalman->dt = (double)(dt) / 1000000;

    kalman->K_rate = newRate - (kalman->K_bias);
    kalman->K_angle += kalman->dt * (kalman->K_rate);

    kalman->P[0][0] += kalman->dt * (kalman->P[1][1] + kalman->P[0][1]) + kalman->Q_angle * kalman->dt;
    kalman->P[0][1] -= kalman->dt * kalman->P[1][1];
    kalman->P[1][0] -= kalman->dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * kalman->dt;

    kalman->S = kalman->P[0][0] + kalman->R_measure;

    kalman->K[0] = kalman->P[0][0] / kalman->S;
    kalman->K[1] = kalman->P[1][0] / kalman->S;

    kalman->y = newValue - kalman->K_angle;

    kalman->K_angle += kalman->K[0] * kalman->y;
    kalman->K_bias += kalman->K[1] * kalman->y;

    kalman->P[0][0] -= kalman->K[0] * kalman->P[0][0];
    kalman->P[0][1] -= kalman->K[0] * kalman->P[0][1];
    kalman->P[1][0] -= kalman->K[1] * kalman->P[0][0];
    kalman->P[1][1] -= kalman->K[1] * kalman->P[0][1];

    //kalman->kt = (double)micros();

    return kalman->K_angle;
}
