//
// Created by EiveLL on 25-2-6.
//
#include "icm20602.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define RAD_TO_DEG 57.295779513082320876798154814105

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

ICM20602_t icm20602;

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void icm20602_read_acc(int fd, ICM20602_t *DataStruct){
    uint8_t data[6];
    uint8_t reg = 0x3B;
    write(fd, &reg, 1);
    read(fd, data, 6);
    DataStruct->Accel_X_RAW = (data[0] << 8) | data[1];
    DataStruct->Accel_Y_RAW = (data[2] << 8) | data[3];
    DataStruct->Accel_Z_RAW = (data[4] << 8) | data[5];
    DataStruct->Ax = (float)DataStruct->Accel_X_RAW / 2048;
    DataStruct->Ay = (float)DataStruct->Accel_Y_RAW / 2048;
    DataStruct->Az = (float)DataStruct->Accel_Z_RAW / 2048;
}

void icm20602_read_gyro(int fd, ICM20602_t *DataStruct){
    uint8_t data[6];
    uint8_t reg = 0x43;
    write(fd, &reg, 1);
    read(fd, data, 6);
    DataStruct->Gyro_X_RAW = (data[0] << 8) | data[1];
    DataStruct->Gyro_Y_RAW = (data[2] << 8) | data[3];
    DataStruct->Gyro_Z_RAW = (data[4] << 8) | data[5];
    DataStruct->Gx = (float)DataStruct->Gyro_X_RAW / 16.4;
    DataStruct->Gy = (float)DataStruct->Gyro_Y_RAW / 16.4;
    DataStruct->Gz = (float)DataStruct->Gyro_Z_RAW / 16.4;
}

void icm20602_read_all(int fd, ICM20602_t *DataStruct, double dt){
    icm20602_read_acc(fd, DataStruct);
    icm20602_read_gyro(fd, DataStruct);
    double roll;
    double roll_sqrt = sqrt(pow(DataStruct->Accel_X_RAW, 2) + pow(DataStruct->Accel_Z_RAW, 2));
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);

    double AngleZ = DataStruct->Gz * dt;
    if(ABS((int)AngleZ) > 1000) AngleZ = 0;
    DataStruct->AngleZ += AngleZ;
}

int icm20602_init(int fd) {
    const char* i2c_dev = "/dev/i2c-0";
    fd = open(i2c_dev, O_RDWR);
    if (fd < 0) {
        return 3;
    }

    if (ioctl(fd, I2C_SLAVE, 0x68) < 0) {
        close(fd);
        return 4;
    }

    // 配置传感器为加速度计模式
    uint8_t config[2] = {0x6B, 0x80};
    write(fd, config, 2);
    usleep(3000);
    uint8_t config1[2] = {0x6B, 0x01};
    write(fd, config1, 2);
    uint8_t config2[2] = {0x6C, 0x00};
    write(fd, config2, 2);
    uint8_t config3[2] = {0x1A, 0x1A};
    write(fd, config3, 2);
    uint8_t config4[2] = {0x19, 0x19};
    write(fd, config4, 2);
    uint8_t config5[2] = {0x1D, 0x03};
    write(fd, config5, 2);
    uint8_t config6[2] = {0x1C, 0x18};
    write(fd, config6, 2);
    uint8_t config7[2] = {0x1B, 0x18};
    write(fd, config7, 2);
    return 0;
}

void icm20602_disable(int fd) {
    close(fd);
}

