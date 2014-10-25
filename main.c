
#include <p32xxxx.h>
#include <plib.h>
#include <math.h>
#include "uart.h"
#include "hardware.h"
#include "i2c.h"
#include "margMPU.h"
#include "mpu9x50.h"
#include "dmpMPUmotion.h"

/** Configuration Bit settings
 * SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
 * PBCLK = 80 MHz (SYSCLK / FPBDIV)
 * Primary Osc w/PLL (XT+,HS+,EC+PLL)
 * WDT OFF
 * Other options are don't care
 */
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = XT, FNOSC = PRIPLL, FPBDIV = DIV_1

#define DEFAULT_MPU_HZ  (200)
#define USE_DMP  0

#define PI          3.14159265358979

#define ACC_EXT_CAL 0                   /**< 1 - Using Calibrated Matrix; 0 - Default */
#define MAG_EXT_CAL 0                   /**< 1 - Using Calibrated Matrix; 0 - Default */

// Define when using DMP
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/*================================================================
                 G L O B A L   V A R I A B L E S
================================================================*/
UINT8 T1overflow = 1;

BYTE new_data=0;

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};


#if !ACC_EXT_CAL                    /**< Identity matrix if not using external calibration data. */
    float acc_offsets [3] = {0, 0, 0};
    float acc_cal_matrix [3][3] = {{1, 0, 0 },
                                   {0, 1, 0 },
                                   {0, 0, 1 }};
#else
    float gyro_offsets [3] = {0,0,0};                                       /// initial values
    float acc_offsets [3] = {   };                 /// from a calibration example
    float acc_cal_matrix [3][3] = {{   },  /// from a calibration example
                                   {   },
                                   {   }};
#endif
#if !MAG_EXT_CAL                    /**< Identity matrix if not using external calibration data. */
    float mag_offsets [3] = {0, 0, 0};
    float mag_cal_matrix [3][3] = {{1, 0, 0 },
                                   {0, 1, 0 },
                                   {0, 0, 1 }};
#else
    float mag_offsets [3] = {   };                 /// from a calibration example
    float mag_cal_matrix [3][3] = {{   }, /// from a calibration example
                                   {   },
                                   {   }};
#endif

/*================================================================
             F U N C T I O N S   P R O T O T Y P E S
================================================================*/

void InitUART (void);
void InitTimer1 (void);
void mpu_dmp_test (void);
void mpu_test (void);
static inline unsigned short inv_row_2_scale (const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar (const signed char *mtx);
static void gyro_data_ready_cb (void);
static void tap_cb (unsigned char direction, unsigned char count);
static void android_orient_cb (unsigned char orientation);
float get_time (void);
void  reset_timer (void);

/**
 * Timer 1 ISR
 *  Interrupt Priority Level = 2
 *  Vector 4
 */
void __ISR(TIMER_1_INT_VECTOR, ipl1) _Timer1Handler(void) {
    // Clear the interrupt flag
    mT1ClearIntFlag();
    // Protection to timer overflow
    T1overflow ++;
}

#if USE_DMP
    void __ISR(EXTERNAL_2_INT_VECTOR, ipl2) INT2Interrupt() {

        mINT2ClearIntFlag();
        hal.new_gyro = 1;

    }
#else
    void __ISR(EXTERNAL_2_INT_VECTOR, ipl2) INT2Interrupt() {

        mINT2ClearIntFlag();
        new_data = 1;
    }
#endif

/*================================================================
                   E N T R Y   P O I N T
================================================================*/
int main(void)
{
    InitUART();
    i2c_init(MPU_I2C, MASTER, 0);

    // Configure Interrupts
    ConfigINT2(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE); // Configure external int before timer since interrupts are enable in the timer function
    InitTimer1();

    TRISAbits.TRISA2  = 0;	// set RA2 out

#if USE_DMP
    mpu_dmp_test (); 
#else
    mpu_test ();
#endif
    return -1;
}


/*================================================================
                        PRINCIPAL FUNCTIONS
================================================================*/
void mpu_test (void){


    mINT1IntEnable(0);
    mINT2IntEnable(0);

    // Variables
    UINT8 buf[1024];
    UINT8 *ptr;
    unsigned int temptime;
    data_xyz gyro, acc, mag;
    float Gyro_dps, Mag_uT = 0.6;
    unsigned short Acc_G;
    float cycle_time, sampleFreq, time_elapsed = 0;
    float checksumA, checksumB;

    // MPU initialization

    mpu_init();  // 
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Calibrations - Acceleremoter and Gyroscope
    AutoCalibrateAcc ();
    AutoCalibrateGyro();

    // Get Sensitivities accordingly to the initializations
    mpu_get_gyro_sens (&Gyro_dps);
    mpu_get_accel_sens( &Acc_G );


    mINT1IntEnable(1);
    mINT2IntEnable(1);

    // Timer reset
    WriteTimer1(0x00);  // Timer set to 0
    T1overflow = 1;

UINT32 count=ReadCoreTimer();
float tempo=0;

    while(1)
    {
        // Colocar em função
        // temptime = ReadTimer1(); // Reads time elapsed
        // cycle_time = (float)((double) (temptime * T1overflow * 256) / PBCLK );// tempo = valortimer*overflow*prescaler*4/fosc ;

        // Atualizations each 10 ms
        if (new_data)
        {

            LATAbits.LATA2 = 1;

            tempo = (float) (CT_TICKS_SINCE(count))/ONE_SECOND;

            temptime = ReadTimer1(); // Reads time elapsed
            cycle_time = (float)((double) (temptime * T1overflow * 256) / PBCLK );// tempo = valortimer*overflow*prescaler*4/fosc ;
            time_elapsed = time_elapsed + cycle_time;
            sampleFreq = 1/cycle_time;

            new_data = 0; // clear flag
            WriteTimer1(0x00);  // Timer set to 0
            T1overflow = 1;

            count = ReadCoreTimer();

            // Read sensors
            ReadGyroXYZ ( &gyro);
            ReadAccXYZ  ( &acc );
            ReadMagXYZ  ( &mag );

            gyro.x /=  Gyro_dps; gyro.y /=  Gyro_dps; gyro.z /=  Gyro_dps;
            acc.x  /=  Acc_G;    acc.y  /=  Acc_G;    acc.z  /=  Acc_G;
            mag.x  /=  Mag_uT;   mag.y  /=  Mag_uT;   mag.z  /=  Mag_uT;

            // sprintf(buf,"I %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, cycle_time, tempo, sampleFreq);
            // SendDataBuffer(buf, strlen(buf));

            checksumA = gyro.x + acc.x + acc.z;
            checksumB = mag.x + mag.y + cycle_time;

            sprintf(buf,"S");   SendDataBuffer( buf, strlen(buf));      // SYNCRONIZE
            ptr = &gyro;        SendDataBuffer( ptr, 3*sizeof(float));  // gyro data
            ptr = &acc ;        SendDataBuffer( ptr, 3*sizeof(float));  // acc  data
            sprintf(buf,"S");   SendDataBuffer( buf, strlen(buf));      // SYNCRONIZE
            ptr = &mag ;        SendDataBuffer( ptr, 3*sizeof(float));  // mag data
            ptr = &cycle_time;  SendDataBuffer( ptr, sizeof(float));    // time of the cycle
            ptr = &checksumA;   SendDataBuffer( ptr, sizeof(float));    // checksumA
            ptr = &checksumB;   SendDataBuffer( ptr, sizeof(float));    // checksumB

//            UINT32 count = ReadCoreTimer();
//            while(CT_TICKS_SINCE(count) < MS_TO_CORE_TICKS(5)) {};

            LATAbits.LATA2 = 0;
        }
    }
}


void mpu_dmp_test (void){

    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    float  e_time;
    UINT8   buf[1024];

    mINT1IntEnable(0);
    mINT2IntEnable(0);

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    mpu_set_gyro_fsr(2000);
    mpu_set_accel_fsr(2);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_set_lpf(0);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON ;
    hal.report = PRINT_QUAT;

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    reset_timer();

    mINT1IntEnable(1);
    mINT2IntEnable(1);

    while (1) {

        if (hal.motion_int_mode) {
            /* Enable motion interrupt. */
            mpu_lp_motion_interrupt(500, 1, 5);
            hal.new_gyro = 0;
            /* Wait for the MPU interrupt. */

            while (!hal.new_gyro){
						//Write code for Low Power Mode (LPM)
						}

            /* Restore the previous sensor configuration. */
            mpu_lp_motion_interrupt(0, 0, 0);
            hal.motion_int_mode = 0;
        }

        if (!hal.sensors || !hal.new_gyro) {
            /* Put the stm32 to sleep until a timer interrupt or data ready
             * interrupt is detected.
             */
            //continue;
        }


        if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel[3], sensors;
            unsigned char more;
            long quat[4];
            float norm_quat[4], norm, aux, q0, q1, q2, q3;
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel, quat, // &sensor_timestamp,
                            &sensors, &more);
            if (!more){
                hal.new_gyro = 0;
            }
            /* Gyro and accel data are written to the FIFO by the DMP in chip
             * frame and hardware units. This behavior is convenient because it
             * keeps the gyro and accel outputs of dmp_read_fifo and
             * mpu_read_fifo consistent.
             */
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO){
                sprintf(buf,"Gyro: %d %d %d \n",gyro[0],gyro[1],gyro[2]);
                SendDataBuffer(buf, strlen(buf));
            }
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL){
                sprintf(buf,"Acc: %d %d %d \n",accel[0],accel[1],accel[2]);
                SendDataBuffer(buf, strlen(buf));
            }


            /* Unlike gyro and accel, quaternions are written to the FIFO in
             * the body frame, q30. The orientation is set by the scalar passed
             * to dmp_set_orientation during initialization.
             */
            if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
                //sprintf(buf,"Quat: %d %d %d %d \n",quat[0],quat[1],quat[2],quat[3]);

                q0= (float) quat[0];
                q1= (float) quat[1];
                q2= (float) quat[2];
                q3= (float) quat[3];
                aux = q0*q0+q1*q1+q2*q2+q3*q3;
                norm =  sqrt (aux);

                norm_quat[0] = q0 / norm;
                norm_quat[1] = q1 / norm;
                norm_quat[2] = q2 / norm;
                norm_quat[3] = q3 / norm;

                e_time= get_time();
                reset_timer();

                sprintf(buf,"Quat:   %.4f   %.4f   %.4f   %.4f   %.4f\n",norm_quat[0],norm_quat[1],norm_quat[2],norm_quat[3], e_time);
                SendDataBuffer(buf, strlen(buf));
        } else if (hal.new_gyro) {
            short gyro[3], accel[3];
            unsigned char sensors, more;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO.
             */
            mpu_read_fifo(gyro, accel,// &sensor_timestamp,
                            &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                sprintf(buf,"Gyro: %d %d %d \n",gyro[0],gyro[1],gyro[2]);
                SendDataBuffer(buf, strlen(buf));
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                sprintf(buf,"Acc: %d %d %d \n",accel[0],accel[1],accel[2]);
                SendDataBuffer(buf, strlen(buf));
        }
    }
}


/*================================================================
                           FUNCTIONS
================================================================*/
void InitUART(void){
    UARTConfigure(UART_MODULE_ID, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART_MODULE_ID, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART_MODULE_ID, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART_MODULE_ID, GetPeripheralClock(), UARTBAUDRATE);
    UARTEnable(UART_MODULE_ID, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}


void InitTimer1 (void){
    OpenTimer1(T1_ON | T1_PS_1_256, 0xFFFF);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1 | T1_INT_SUB_PRIOR_0);// Set up the core timer interrupt with a priority of 2 and zero sub-priority
    INTEnableSystemMultiVectoredInt(); // STEP 3. enable multi-vector interrupts
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
    UINT8   buf[1024];
    sprintf(buf,"Gyro reading\r\n");
    SendDataBuffer(buf, strlen(buf));
    hal.new_gyro = 1;
}

static void tap_cb(unsigned char direction, unsigned char count)
{
    char data[2];
    data[0] = (char)direction;
    data[1] = (char)count;

    UINT8   buf[1024];
    sprintf(buf,"Tap detected\r\n");
    SendDataBuffer(buf, strlen(buf));
}

static void android_orient_cb(unsigned char orientation)
{
    UINT8   buf[1024];
    sprintf(buf,"Android orient");
    SendDataBuffer(buf, strlen(buf));
}

float get_time (void)
{
    unsigned int temptime;
    float elapsed_time;

    // Colocar em função
    temptime = ReadTimer1(); // Reads time elapsed
    elapsed_time = (float)((double) (temptime * T1overflow * 256) / PBCLK );// tempo = valortimer*overflow*prescaler*4/fosc ;

    return elapsed_time;
}

void reset_timer (void)
{
    WriteTimer1(0x00);  // Timer set to 0
    T1overflow = 1;
}