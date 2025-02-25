//
// Created by EiveLL on 25-2-6.
//

#ifndef ICM20602_H
#define ICM20602_H

#include <stdint.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <math.h>
#include "config.h"
#include <iostream>

typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
    double Old_Gz;

    float Temperature;

    double KalmanAngleX_Old;
    double KalmanAngleY_Old;
    double KalmanAngleX;
    double KalmanAngleY;
    double AngleZ;
} ICM20602_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void icm20602_read_acc(int fd, ICM20602_t *DataStruct);
void icm20602_read_gyro(int fd, ICM20602_t *DataStruct);
void icm20602_read_all(int fd, ICM20602_t *DataStruct, double dt);
int icm20602_init(int fd);
void icm20602_disable(int fd);

extern ICM20602_t icm20602;

#endif //ICM20602_H
