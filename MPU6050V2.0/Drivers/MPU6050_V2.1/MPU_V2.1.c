/*
 * MPU_V2.1.c
 *
 *  Created on: 21 Ağu 2022
 *      Author: Alihan
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdio.h"
#include "MPU_V2.1.h"
#include "IIR_filter.h"


extern I2C_HandleTypeDef hi2c1;

Inital_check initial_check = not_initiliazed;
uint32_t get_last_time1;
IIRFirstOrder LpfAccel[3];
IIRFirstOrder LpfGyro[3];
static uint8_t  DATA_GYRO_CONFIG  ;
static uint8_t  DATA_ACCEL_CONFIG ;
static uint8_t DATA_PWR_MGTM1 =  (0x00);
static uint8_t DATA_SMPRT_DIV =  (0x07);
static uint8_t DATA_DLPF_CFG  =  (0x01);
uint8_t rxbuff[6];
uint8_t rxbuff2[6];
uint8_t rxbuff3[2];
float Magnitude=0.0f;
float filtered_gyro_dps[3]={0.0f,0.0f,0.0f};
float filtered_accel_g[3]={0.0f,0.0f,0.0f};
float  gyro_dps[3]={0.0f,0.0f,0.0f}  ;
float accel_g[3]={0.0f,0.0f,0.0f};
float delta_t1 = 0.0f;
float roll_acc=0.0f,pitch_acc=0.0f;
float roll_gyro=0.0f ,pitch_gyro=0.0f;
float Yaw;






void MPU6050_Init(Accel_Gyro *MPU6050 , Fs_sel_e Gyro_range, Afs_sel_e Acc_range){


     /* Gyroscope , Accelerometer Range and Division coefficient choice */
	switch (Gyro_range){
	case FS_250 : DATA_GYRO_CONFIG = 0x00 ; MPU6050->Div_coef_gyr  = 131.0f  ; break ;
	case FS_500 : DATA_GYRO_CONFIG = 0x08 ; MPU6050->Div_coef_gyr  = 65.50f   ; break ;
	case FS_1000: DATA_GYRO_CONFIG = 0x10 ; MPU6050->Div_coef_gyr  = 32.80f   ; break ;
	case FS_2000: DATA_GYRO_CONFIG = 0x18 ; MPU6050->Div_coef_gyr  = 16.40f   ; break ;

	default:   DATA_GYRO_CONFIG = 0x00 ; MPU6050->Div_coef_gyr  = 131.0f  ; break ;

	}


	switch (Acc_range){

	case AFS_2G: DATA_ACCEL_CONFIG = 0x00 ; MPU6050->Div_coef_acc = 16384.0f ; break ;
	case AFS_4G: DATA_ACCEL_CONFIG = 0x08 ; MPU6050->Div_coef_acc = 8192.0f  ; break ;
	case AFS_8G: DATA_ACCEL_CONFIG = 0x10 ; MPU6050->Div_coef_acc = 4096.0f  ; break ;
	case AFS_16G:DATA_ACCEL_CONFIG=  0x18 ; MPU6050->Div_coef_acc = 2048.0f  ; break ;

	default:   DATA_ACCEL_CONFIG = 0x00 ; MPU6050->Div_coef_acc  = 16384.0f;

	}

	/*Device Configuration  */


  while(!(HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_I2C_ADRESS_AD0, TRIALS, TIMEOUT)==HAL_OK));

  /* Power manangment All bits are zero -> 0x00 */
  while(!(HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, PWR_MGMT_1 ,SIZE_OF_ADDRES,&DATA_PWR_MGTM1,  SIZE_OF_DATA ,TIMEOUT)==HAL_OK));
  /* Sample Rate Configuration (1kHz) -> 0x07 */
  while(!(HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, SMPLRT_DIV ,SIZE_OF_ADDRES,&DATA_SMPRT_DIV , SIZE_OF_DATA ,TIMEOUT)==HAL_OK));
  /* Inertial Low pass Configuration ,this bits gives  actual bandwidth to us (Accelerometer 184 Hz - Gyroscope 188Hz ~= 200Hz  T=0.005) -> 0x01 */
  while(!(HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, CONFIG ,SIZE_OF_ADDRES,&DATA_DLPF_CFG , SIZE_OF_DATA ,TIMEOUT)==HAL_OK));
  /* Accelerometer Configuration that you desire in Init function -> XXXX */
  while(!(HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, ACCEL_CONFIG ,SIZE_OF_ADDRES, &DATA_ACCEL_CONFIG, SIZE_OF_DATA ,TIMEOUT)==HAL_OK));
  /* Gyroscope Configuration that you desire in Init function-> XXXX */
  while(!(HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, GYRO_CONFIG ,SIZE_OF_ADDRES, &DATA_GYRO_CONFIG, SIZE_OF_DATA ,TIMEOUT)==HAL_OK));

  /*First Order IIR filter Initializing*/
  for(int n=0 ;n<3;n++)
  {

	  IIRFirstOrder_Init(&LpfAccel[n], LpfAccel_ALPHA);

      IIRFirstOrder_Init(&LpfGyro [n], LpfGyro_ALPHA );


   }
  /*Counter start for Sampling Period*/
  get_last_time1 = HAL_GetTick();


}
void MPU6050_Start(Accel_Gyro*MPU6050 , Temperature *temp){



	           MPU6050_READ(GYRO_XOUT_H, rxbuff, 6);


	           MPU6050->GYRO_OUT_RAW[0] = (int16_t) ( (rxbuff[0] << 8) | rxbuff[1]);
	           MPU6050->GYRO_OUT_RAW[1] = (int16_t) ( (rxbuff[2] << 8) | rxbuff[3]);
	           MPU6050->GYRO_OUT_RAW[2] = (int16_t) ( (rxbuff[4] << 8) | rxbuff[5]);


	           gyro_dps[0] =  (float)((MPU6050->GYRO_OUT_RAW [0] / MPU6050->Div_coef_gyr ));
	           gyro_dps[1] =  (float)((MPU6050->GYRO_OUT_RAW [1] / MPU6050->Div_coef_gyr ));
	           gyro_dps[2] =  (float)((MPU6050->GYRO_OUT_RAW [2] / MPU6050->Div_coef_gyr ));


	           /*First Order IIR filter Update for Gyroscope*/
	           for(int i =0 ; i<3;i++){
	        	   IIRFirstOrder_Filter(&LpfGyro[i], gyro_dps[i]);
	        	   filtered_gyro_dps[i]= LpfGyro[i].out;

	           }




	           MPU6050->Xgy +=  (filtered_gyro_dps[0] -  GYRO_BIAS_X) * DPS_2_RPS * Sampling_Per  ;     //GyroX *dt
	           MPU6050->Ygy +=  (filtered_gyro_dps[1] -  GYRO_BIAS_Y) * DPS_2_RPS * Sampling_Per  ;     //GyroY *dt
	           MPU6050->Zgy +=  (filtered_gyro_dps[2] -  GYRO_BIAS_Z) * DPS_2_RPS * Sampling_Per  ;     //GyroZ *dt


	           MPU6050_READ(ACCEL_XOUT_H, rxbuff2, 6);


	           MPU6050->ACCEL_OUT_RAW[0] = ((int16_t)(rxbuff2[0] << 8 | rxbuff2[1]));
	           MPU6050->ACCEL_OUT_RAW[1] = ((int16_t)(rxbuff2[2] << 8 | rxbuff2[3]));
	           MPU6050->ACCEL_OUT_RAW[2] = ((int16_t)(rxbuff2[4] << 8 | rxbuff2[5]));

	           accel_g[0] = (float)((MPU6050->ACCEL_OUT_RAW[0]/MPU6050->Div_coef_acc));
	           accel_g[1] = (float)((MPU6050->ACCEL_OUT_RAW[1]/MPU6050->Div_coef_acc));
	           accel_g[2] = (float)((MPU6050->ACCEL_OUT_RAW[2]/MPU6050->Div_coef_acc));

	           /*First Order IIR filter Update for Accelerometer*/

	           for (int j=0; j<3  ; j++){
	        	   IIRFirstOrder_Filter(&LpfAccel[j], accel_g[j]);

	        	    filtered_accel_g[j] =LpfAccel[j].out;

	           }
	           /*Scale and Bias factors implementation*/

	           MPU6050->Xacc =    ACCEL_SCALE_X * filtered_accel_g[0] + ACCEL_BIAS_X;  //Scale and Bias factors depends your sensor ,I will explain next posts in linedln
	           MPU6050->Yacc =    ACCEL_SCALE_Y * filtered_accel_g[1] + ACCEL_BIAS_Y;  //Scale and Bias factors depends your sensor ,I will explain next posts in linedln
	           MPU6050->Zacc =    ACCEL_SCALE_Z * filtered_accel_g[2] + ACCEL_BIAS_Z;  //Scale and Bias factors depends your sensor ,I will explain next posts in linedln

	           /*Temperature calculations*/
	           MPU6050_READ(TEMP_OUT_H , rxbuff3, 2);

	           temp->Temp_Raw = (int16_t)(rxbuff3[0] <<8 | rxbuff3[1]);
	           temp->Temperature =(float)((temp->Temp_Raw / 340) + 36.53);/*Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53*/

	           /*Total 3D vector Magnitude for angles calculations*/

	           Magnitude =  sqrtf(powf(MPU6050->Xacc,2.0f) +  powf(MPU6050->Yacc,2.0f) + powf(MPU6050->Zacc , 2.0f));


	           Error_calc(MPU6050, Magnitude);



	           Complementary_filter(MPU6050);



	           /*Measuring  Sampling period for bandwidth -> T =0.005 , 200Hz*/

	           delta_t1 = (float)(HAL_GetTick() - get_last_time1) / 1000 ;
	           get_last_time1 = HAL_GetTick();
	           HAL_Delay(2);


}




void MPU6050_READ(uint8_t RegAddress , uint8_t *Rxbuffer, uint8_t size){

	uint8_t Txbuffer[1];


	Txbuffer[0]=RegAddress;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_I2C_ADRESS_AD0, Txbuffer, SIZE_OF_DATA, TIMEOUT);

	HAL_I2C_Master_Receive(&hi2c1, MPU6050_I2C_ADRESS_AD0, Rxbuffer, size, TIMEOUT);

}


void Complementary_filter(Accel_Gyro *MPU6050){


    /* from 3D vector to angle using arcsin , if you choose correct vectors, you can use any method atan2f(arctan) or other terms*/
    roll_acc  = asinf(  MPU6050->Xacc / Magnitude);
    pitch_acc = asinf( -MPU6050->Yacc / Magnitude);

    roll_gyro  = MPU6050->Xgy;
    pitch_gyro = MPU6050->Ygy;

    /*Axis shifting*/

    roll_gyro  -=  pitch_gyro * sinf(MPU6050->Zgy);
    pitch_gyro +=  roll_gyro  * sinf(MPU6050->Zgy);

    /*Initiliaze first values*/
    initial_check = (initial_check == not_initiliazed) ? inital_roll_pitch(): initiliazed ;


    roll_gyro = ( (  (roll_gyro  )   * (1- ALPHA) ) + ALPHA  *  roll_acc ) * RAD_2_DEG;
    pitch_gyro= ( (  (pitch_gyro )   * (1- ALPHA) ) + ALPHA  *  pitch_acc) * RAD_2_DEG;
    Yaw = (MPU6050->Zgy)*RAD_2_DEG;



    MPU6050->Roll  = roll_gyro ;
    MPU6050->Pitch = pitch_gyro;
    MPU6050->Yaw   =   Yaw     ; // We can  use sensor fusion algorithm with Magnetometer for reach more correct Yaw axis.

}


void Error_calc(Accel_Gyro*MPU6050, float mag /*magnitude*/){

	/*Absolute value */

	 (mag < 0 ? (mag *=-1) : (mag *=1));

	 MPU6050->Accel_Error_percent = (1-mag) * 100;
	 MPU6050->Gyro_Error_val  = (  MPU6050->Xgy +  MPU6050->Ygy +   MPU6050->Zgy );

}


Inital_check inital_roll_pitch(){

pitch_gyro = pitch_acc ;
roll_gyro  = roll_acc ;

return  initiliazed ;}











