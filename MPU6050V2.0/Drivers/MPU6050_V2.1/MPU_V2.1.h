/*
 * MPU_V2.1.h
 *
 *  Created on: 21 Ağu 2022
 *      Author: hp
 */

#ifndef MPU6050_V2_1_MPU_V2_1_H_
#define MPU6050_V2_1_MPU_V2_1_H_
#include "stdint.h"
////////////////////////////////////////////////////////////////
#define MPU6050_I2C_ADRESS_AD0 ( 0x68<<1 )
#define MPU6050_I2C_ADRESS_AD1 ( 0x69<<1 )
#define CONFIG          (0x1A)
#define SMPLRT_DIV      (0x19)
#define CONFIG          (0x1A)
#define GYRO_CONFIG     (0x1B)
#define ACCEL_CONFIG    (0x1C)
#define ACCEL_XOUT_H    (0x3B)
#define ACCEL_YOUT_H    (0x3D)                            /*MPU6050 REGISTER*/
#define ACCEL_ZOUT_H    (0x3F)
#define TEMP_OUT_H      (0x41)
#define GYRO_XOUT_H     (0x43)
#define GYRO_YOUT_H     (0x45)
#define GYRO_ZOUT_H     (0x47)
#define PWR_MGMT_1      (0x6B)
#define WHO_AM_I        (0x75)
///////////////////////////////////////////////////////////////////
#define DPS_2_RPS  (0.01745329251f) /*Degrees per second to radians per second */
#define RAD_2_DEG  (57.2957795131f) /*radian to degree */
#define DEG_2_RAD  (0.01745329251f)) /*Degrees to radian */
#define TRIALS     (2) /*Number of trials */
#define GYRO_BIAS_X  (0.493997544f) /*Gyroscope X axis bias value */
#define GYRO_BIAS_Y  (1.4021486f) /*Gyroscope X axis bias value */
#define GYRO_BIAS_Z  (0.0656538159f) /*Gyroscope X axis bias value */
#define ACCEL_BIAS_X (-0.06679528493406561f) /*Accelerometer X axis bias value */         //this values depends your sensor you have to measure the values
#define ACCEL_BIAS_Y (0.005650864111887643f) /*Accelerometer Y axis bias value */
#define ACCEL_BIAS_Z (-0.18814515510887585f) /*Accelerometer Z axis bias value */
#define ACCEL_SCALE_X (1.00891108285f) /*Accelerometer X axis scale value */
#define ACCEL_SCALE_Y (1.0072810885f) /*Accelerometer Y axis scale value */
#define ACCEL_SCALE_Z (1.063263652f) /*Accelerometer Z axis scale value */
///////////////////////////////////////////////////////////////////
#define TIMEOUT       (100)  /*100 ms*/
#define SIZE_OF_ADDRES   (1) /*1 byte*/
#define SIZE_OF_DATA     (1) /*1 byte*/
#define ALPHA          (0.9995f)/*Complementary Filter Alpha*/
#define LpfAccel_ALPHA (0.730402f) /*Low pass Filter alpha for Accelerometer */
#define LpfGyro_ALPHA  (0.8282041)/*Low pass Filter alpha for Gyroscope */
#define Sampling_Per   (0.00499999989f) /*this data measured on this code using delta_t1 variable( 0.005s  -> 200 Hz)*/


typedef enum{

	initiliazed,
	not_initiliazed,


}Inital_check; //initial check enum for Complementary filter

typedef enum {
	FS_250,
	FS_500,
	FS_1000,
	FS_2000,
}Fs_sel_e; // Scale Range Gyroscope


typedef enum {
	AFS_2G,
	AFS_4G,
	AFS_8G,
	AFS_16G
}Afs_sel_e ; //Scale Range Accelerometer


typedef struct{

	int16_t ACCEL_OUT_RAW[3]; /*Accelerometer Raw values series [0->X],[1->Y],[2->Z]*/
	int16_t GYRO_OUT_RAW [3]; /*Gyroscope Raw values series [0->X],[1->Y],[2->Z]*/
	float Xacc; /*G value of accel X axis*/
	float Yacc; /*G value of accel Y axis*/
	float Zacc; /*G value of accel Z axis*/
	float Xgy;  /*Radian value of gyro X axis*/
	float Ygy;  /*Radian value of gyro Y axis*/
	float Zgy;  /*Radian value of gyro Z axis*/
	float Div_coef_acc; /*Division coefficient for accelerometer*/
	float Div_coef_gyr; /*Division coefficient for gyroscope*/
	float Roll; /*Final Roll value*/
	float Pitch;/*Final Pitch value*/
	float Yaw ; /*Final Yaw value*/
	float Accel_Error_percent; /* error for accelerometer %error */
	float Gyro_Error_val ; /* if sensor not moving the gyro values equal zero ,this data show total value of gyro*/


}Accel_Gyro;


typedef struct{

	int16_t Temp_Raw;/*Raw temperature data*/
	float Temperature;/*Actual temperature data*/


}Temperature;
Inital_check inital_roll_pitch();
void MPU6050_Init(Accel_Gyro * MPU6050 , Fs_sel_e Gyro_range, Afs_sel_e Acc_range);
void MPU6050_Start(Accel_Gyro* MPU6050 , Temperature *temp);
void MPU6050_READ(uint8_t RegAddress , uint8_t *Rxbuffer, uint8_t size);
void Error_calc(Accel_Gyro * MPU6050, float mag);
void Complementary_filter(Accel_Gyro *MPU6050);




#endif /* MPU6050_V2_1_MPU_V2_1_H_ */

/* Resources
https://www.youtube.com/c/CoskunTasdemirKanal
https://www.youtube.com/user/MacPuffdog

*/
