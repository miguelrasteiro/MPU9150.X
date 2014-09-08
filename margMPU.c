 /**
  * \addtogroup marg
  * \{
  * \file marg.c
  * \author Miguel Rasteiro \n\t
  * \brief Sensor data treatment library for MPU9x50 lib \n
  * \version  Version 1.0 Release 1 \n
  * \date     27 August de 2014, 16:34 \n\n
  * \ref license \n\n
  *    Change History:
  * ************************************************************************
  * | VERSION      | DATE	    | AUTHORS	  | DESCRIPTION            |
  * |:------------:|:--------------:|:------------|:-----------------------|
  * | 1.0          |   27/8/2014    | MR          | First Release          |
  *\n\n
  * \section margcode MARG Source Code
  * \code
  */
#include "mpu9x50.h"
#include "margMPU.h"

#define WARMUP_READINGS  256.0    /**< Number of warmup readings for sensors calibration */
#define AVERAGE_READINGS  256.0  /**< Number of readings for average sensors offsets */

/*******************************************************************************
* Function Name  : ReadGyroXYZ
* Description    : Read the angular velocity treated data
* Input          : Gyro axis storage variable (data_xyz type)
* Output         : x,y,z data
* Return         : None
*******************************************************************************/
void ReadGyroXYZ ( data_xyz* data ) {

    short gyro_raw[3];

    mpu_get_gyro_reg ( gyro_raw);

    data->x = (float) gyro_raw[0] - gyro_offsets[0];
    data->y = (float) gyro_raw[1] - gyro_offsets[1];
    data->z = (float) gyro_raw[2] - gyro_offsets[2];
}
void ReadAccXYZ ( data_xyz* data ) {

    short acc_raw[3];

    mpu_get_accel_reg ( acc_raw);

    data->x = ((float)acc_raw[0] * acc_cal_matrix[0][0] +
               (float)acc_raw[1] * acc_cal_matrix[1][0] +
               (float)acc_raw[2] * acc_cal_matrix[2][0] - acc_offsets [0] );
    data->y = ((float)acc_raw[0] * acc_cal_matrix[0][1] +
               (float)acc_raw[1] * acc_cal_matrix[1][1] +
               (float)acc_raw[2] * acc_cal_matrix[2][1] - acc_offsets [1] );
    data->z = ((float)acc_raw[0] * acc_cal_matrix[0][2] +
               (float)acc_raw[1] * acc_cal_matrix[1][2] +
               (float)acc_raw[2] * acc_cal_matrix[2][2] - acc_offsets [2] );
}
void ReadMagXYZ ( data_xyz* data ) {

    short mag_raw[3];

    mpu_get_compass_reg( mag_raw);

    data->x =((float)mag_raw[0] * mag_cal_matrix[0][0] +
              (float)mag_raw[1] * mag_cal_matrix[1][0] +
              (float)mag_raw[2] * mag_cal_matrix[2][0] - mag_offsets [0]) ;
    data->y =((float)mag_raw[0] * mag_cal_matrix[0][1] +
              (float)mag_raw[1] * mag_cal_matrix[1][1] +
              (float)mag_raw[2] * mag_cal_matrix[2][1] - mag_offsets [1]) ;
    data->z =((float)mag_raw[0] * mag_cal_matrix[0][2] +
              (float)mag_raw[1] * mag_cal_matrix[1][2] +
              (float)mag_raw[2] * mag_cal_matrix[2][2] - mag_offsets [2]) ;
}
void AutoCalibrateAcc ( void ) {

    short cal_readings[3];
    float cal_sum[3]={0,0,0};
    int i;

    for ( i=0; i<WARMUP_READINGS; i++ ) {            // Take a number of readings to warm up
        mpu_get_accel_reg ( cal_readings);
    }

    for ( i=0; i<AVERAGE_READINGS; i++ ) {           // Take a number of readings and average them
                                        // to calculate any bias the accelerometer may have.
        mpu_get_accel_reg ( cal_readings);

        cal_sum[0] += (float) cal_readings[0];
        cal_sum[1] += (float) cal_readings[1];
        cal_sum[2] += (float) cal_readings[2];
    }
    // Find Gravity value
    unsigned short Gravity;
    mpu_get_accel_sens( &Gravity );

    acc_offsets[0] = cal_sum[0] / AVERAGE_READINGS;
    acc_offsets[1] = cal_sum[1] / AVERAGE_READINGS;
    acc_offsets[2] = cal_sum[2] / AVERAGE_READINGS - Gravity;
}
void AutoCalibrateGyro ( void ) {

    short cal_readings[3];
    float cal_sum[3]={0,0,0};
    int i;

    for ( i=0; i<WARMUP_READINGS; i++ ) {            // Take a number of readings to warm up
        mpu_get_gyro_reg ( cal_readings);
    }

    for ( i=0; i<AVERAGE_READINGS; i++ ) {           // Take a number of readings and average them
                                                     // to calculate the bias of the gyroscope may have.
        mpu_get_gyro_reg ( cal_readings);

        cal_sum[0] += (float) cal_readings[0];
        cal_sum[1] += (float) cal_readings[1];
        cal_sum[2] += (float) cal_readings[2];
    }
    gyro_offsets[0] = cal_sum[0] / AVERAGE_READINGS;
    gyro_offsets[1] = cal_sum[1] / AVERAGE_READINGS;
    gyro_offsets[2] = cal_sum[2] / AVERAGE_READINGS;
}
void UpdateGyroBias ( void ) {          // Use carefully, you have to guarantee that the gyro is stationary to update mean
                                        // Updates gyro offsets each reading weighs 1/AVERAGE_READINGS
    short cal_readings[3];

    mpu_get_gyro_reg ( cal_readings);

    gyro_offsets[0] = gyro_offsets[0] + (((float) cal_readings[0] - gyro_offsets[0]) / (AVERAGE_READINGS));
    gyro_offsets[1] = gyro_offsets[1] + (((float) cal_readings[1] - gyro_offsets[1]) / (AVERAGE_READINGS));
    gyro_offsets[2] = gyro_offsets[2] + (((float) cal_readings[2] - gyro_offsets[2]) / (AVERAGE_READINGS));
}
void UpdateAccBias ( void ) {          // Use carefully, you have to guarantee that the acc is stationary and z axis is pointing down to update mean
                                       // Updates acc offsets, each reading weighs 1/AVERAGE_READINGS
    short cal_readings[3];

    mpu_get_accel_reg ( cal_readings);

    acc_offsets[0] = acc_offsets[0] + (((float) cal_readings[0] - acc_offsets[0]) / (AVERAGE_READINGS));
    acc_offsets[1] = acc_offsets[1] + (((float) cal_readings[1] - acc_offsets[1]) / (AVERAGE_READINGS));
    acc_offsets[2] = acc_offsets[2] + (((float) cal_readings[2] - acc_offsets[2]) / (AVERAGE_READINGS));
}

/**\endcode \}*/
