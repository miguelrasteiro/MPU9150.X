/* 
 * File:   margMPU.h
 * Author: MiguelRasteiro
 *
 * Created on 27 de Agosto de 2014, 12:56
 */

#ifndef MARGMPU_H
#define	MARGMPU_H

/** @details Saves calibrated data for all axis. */
typedef struct {
            /// x-axis treated data.
   float  	x,
            /// y-axis treadted data.
                y,
            /// z-axis treated data.
                z;
}data_xyz ;

/************ VARIABLES ************/
float gyro_offsets[3];          /**< Saves gyroscope offsets in a vector --> position [0,1,2] = [x,y,z] data. */
float acc_offsets [3];          /**< Saves accelerometer offsets in a vector --> position [0,1,2] = [x,y,z] data. */
float acc_cal_matrix [3][3];    /**< Calibration matrix for the accelerometer. Obtained in the extended calibration of
                                 * the accelerometer. Otherwise use the identity matrix 3x3  */
float mag_offsets [3];          /**< Saves magnetometer offsets in a vector --> position [0,1,2] = [x,y,z] data. */
float mag_cal_matrix [3][3];    /**< Calibration matrix for the magnetometer. Obtained in the extended calibration of
                                 * the magnetometer. Otherwise use the identity matrix 3x3  */

/************ FUNCTIONS ************/
/** \fn     void  ReadGyroXYZ (data_xyz *data)
 * \brief      Read gyroscope removing offsets.
 * \param      [in/out] struct data_xyz pointer (type float (32bits)).
 */
void  ReadGyroXYZ       ( data_xyz*  data  );
/** \fn     void  ReadAccXYZ (data_xyz *data)
 * \brief      Read accelerometer compensating distortion and removing offsets if using calibration matrix.
 * \param      [in/out] struct data_xyz pointer (type float (32bits)).
 */
void  ReadAccXYZ        ( data_xyz*  data  );
/** \fn     void  ReadMagXYZ (data_xyz *data)
 * \brief      Read magnetometer compensating distortion and removing offsets if using calibration matrix.
 * \param      [in/out] struct data_xyz pointer (type float (32bits)).
 */
void  ReadMagXYZ        ( data_xyz*  data  );
/** \fn     void AutoCalibrateAcc ( void )
 * \brief   Takes a number off readings to calculate accelerometer offsets
 */
void  AutoCalibrateAcc  ( void );
/** \fn     void AutoCalibrateGyro ( void )
 * \brief   Takes a number off readings to calculate gyroscope offsets
 */
void  AutoCalibrateGyro ( void );
/** \fn     void UpdateGyroBias ( void )
 * \brief   Can be used to ajust gyroscope offsets with time
 * \details Use carefully, you have to guarantee that the gyro is stationary to update mean
 */
void  UpdateGyroBias    ( void );
/** \fn     void UpdateAccBias ( void )
 * \brief   Can be used to ajust accelerometer offsets with time
 * \details Use carefully, you have to guarantee that the acc is stationary and z axis
 * is pointing down to update mean
 */
void  UpdateAccBias     ( void );


#endif	/* MARGMPU_H */

