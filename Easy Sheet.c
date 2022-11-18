//The code snippet shown below is used to perform an angle value filtering with the values ​​read from the features of the MPU6050 sensor (6-Axis Motion Tracker).

#include <math.h>
#include "mpu6050.h"

#define RAD2DEG 57.295779513082320876798154814105
#define WHOAMI 0x75
#define PWR_MGMT 0x6B
#define SMPLRT_DIV 0x19
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define MPU6050_ADDR 0xD0
const uint16_t I2C_TIMEOUT = 100;
const double Acc_Z_corrector = 14418.0;
uint32_t timer;

Filter_t FilterX = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f};

Filter_t FilterY = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f,
};

uint8_t MPU_Init(I2C_HandleTypeDef *I2Cx)                                               /* initilization */
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHOAMI, 1, &check, 1, I2C_TIMEOUT);            //ID information is checked in the HAL_I2C_Mem_Read function and this value is returned as a number.

    if (check == 104)                                                                   //HAL_I2C_Mem_Read returned number value is queried and if the value is true, the value is written to the addresses of the mpu6050 chip
    {
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        return 0;
    }
    return 1;
}

void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)                               //acceleration data read 
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);   //The raw acceleration value of X is found by shifting the 0th byte 8 of rec_data to the left and using the 1st byte or operation
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);   //The raw acceleration value of Y is found by shifting the 2th byte 8 of rec_data to the left and using the 3st byte or operation 
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);   //The raw acceleration value of Z is found by shifting the 4th byte 8 of rec_data to the left and using the 5st byte or operation 

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;                                     //Ax value is found by the ratio of the raw speed value found above to the given value
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;                                     //Ay value is found by the ratio of the raw speed value found above to the given value
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;                             //Ratio the raw velocity of z with the Acc_Z_corrector value to find the Az value
}

void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)                                //gyro data read
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);    //Shift the 8th byte 8 of rec_data to the left to find the raw gyro value of the 9th byte or operation Xin
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);  //Raw gyro value of Y, byte 10 of rec_data shifts byte 8 to the left and by byte 11 or is done
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);  //Shift the 12th byte 8 of rec_data to the left and find the raw gyro value of the 13th byte or operation Z

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;                    //Gyrox's coordinate find real value
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;                  //Gyroy's coordinate find real value
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;                  //Gyroz's coordinate find real value
}

void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)                                //temperature data read
{
    uint8_t Rec_Data[2];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, I2C_TIMEOUT);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);                                    //The temp value is found by shifting the 0.byte 8 of the data to the left and by doing the 1st byte or operation
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);      //It allows us to find the temperature value by dividing the temp value found by the sum of the relevant values.
}

void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)                                 //all value calculate and read
{
    uint8_t Rec_Data[14];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);   //The raw acceleration value of X is found by shifting the 0th byte 8 of rec_data to the left and using the 1st byte or operation
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);   //The raw acceleration value of Y is found by shifting the 2th byte 8 of rec_data to the left and using the 3st byte or operation 
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);   //The raw acceleration value of Z is found by shifting the 4th byte 8 of rec_data to the left and using the 5st byte or operation 
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);                      //Shift the 6th byte 8 of rec_data to the left and find the temp value of the 7th byte or process
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);    //Shift the 8th byte 8 of rec_data to the left to find the raw gyro value of the 9th byte or operation Xin
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);  //Raw gyro value of Y, byte 10 of rec_data shifts byte 8 to the left and by byte 11 or is done
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);  //Shift the 12th byte 8 of rec_data to the left and find the raw gyro value of the 13th byte or operation Z

//Ax, Ay, Az and Gx, Gy, Gz raw values ​​are found by processing the raw values ​​found above. 
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53); //It is obtained by dividing the temp value found by the sum of the relevant numbers.
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);   //The value of the roll_sqrt function is found by taking the square root of the sum of the square of the raw velocity values ​​of X and Z.
    if (roll_sqrt != 0.0)                                                                                         //If the roll_sqrt value is not zero, the ratio of the Y raw speed to the roll_sqrt variable is found with the arc tangent to the roll value                                         
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD2DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD2DEG;                                //The arc tangent is found by multiplying the raw velocity values ​​of X and Z with the radian value. The reason why it is Atan2 is to take the arc tangent of its direction correctly in the coordinate.
    if ((pitch < -90 && DataStruct->FilterAngleY > 90) || (pitch > 90 && DataStruct->FilterAngleY < -90))
    {
        FilterY.angle = pitch;
        DataStruct->FilterAngleY = pitch;
    }
    else
    {
        DataStruct->FilterAngleY = Filter_getAngle(&FilterY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->FilterAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->FilterAngleX = Filter_getAngle(&FilterX, roll, DataStruct->Gx, dt);
}

//For the X and Y values, the new values ​​are calculated with the previous angle values ​​and the deviations are found, and finally the angle value is returned.
double Filter_getAngle(Filter_t *Filter, double newAngle, double newRate, double dt)          
{
    double rate = newRate - Filter->bias;
    Filter->angle += dt * rate;

    Filter->P[0][0] += dt * (dt * Filter->P[1][1] - Filter->P[0][1] - Filter->P[1][0] + Filter->Q_ANGLE);
    Filter->P[0][1] -= dt * Filter->P[1][1];
    Filter->P[1][0] -= dt * Filter->P[1][1];
    Filter->P[1][1] += Filter->Q_BIAS * dt;

    double S = Filter->P[0][0] + Filter->R_MEASURE;
    double K[2];
    K[0] = Filter->P[0][0] / S;
    K[1] = Filter->P[1][0] / S;

    double y = newAngle - Filter->angle;
    Filter->angle += K[0] * y;
    Filter->bias += K[1] * y;

    double P00_temp = Filter->P[0][0];
    double P01_temp = Filter->P[0][1];

    Filter->P[0][0] -= K[0] * P00_temp;
    Filter->P[0][1] -= K[0] * P01_temp;
    Filter->P[1][0] -= K[1] * P00_temp;
    Filter->P[1][1] -= K[1] * P01_temp;

    return Filter->angle;
};