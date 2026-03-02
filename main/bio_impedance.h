#ifndef BIO_IMPEDANCE_H
#define BIO_IMPEDANCE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

// // GPIO number used for I2C master clock
// #define I2C_MASTER_SCL_IO           GPIO_NUM_20  // for new esp32c6 board

// // GPIO number used for I2C master data
// #define I2C_MASTER_SDA_IO           GPIO_NUM_21

// // I2C master port number
// #define I2C_MASTER_NUM              0
// #define I2C_NUM                     I2C_NUM_0

// // I2C master clock frequency
// #define I2C_MASTER_FREQ_HZ          10000  // from AD5933 dev_cfg

// // I2C master doesn't need buffer for transmission
// #define I2C_MASTER_TX_BUF_DISABLE   0

// // I2C master doesn't need buffer for reception
// #define I2C_MASTER_RX_BUF_DISABLE   0

// // Timeout for I2C master in milliseconds
// #define I2C_MASTER_TIMEOUT_MS       1000

// AD5933 Address
#define AD5933_ADDRESS          ((uint8_t)0x0D)

// AD5933_CONTROL_RANGE(x) options
#define AD5933_RANGE_2000mVpp   ((uint8_t)0x0)
#define AD5933_RANGE_200mVpp    ((uint8_t)0x1)
#define AD5933_RANGE_400mVpp    ((uint8_t)0x2)
#define AD5933_RANGE_1000mVpp   ((uint8_t)0x3)

// AD5933_CONTROL_PGA options
#define AD5933_PGA_1        ((uint8_t)0x1)
#define AD5933_PGA_5        ((uint8_t)0x0)

// AD5933 Registers
#define AD5933_REG_CONTROL_HB       ((uint8_t)0x80)    /*!< HB of the Control register */
#define AD5933_REG_CONTROL_LB       ((uint8_t)0x81)    /*!< LB of the Control register */
#define AD5933_REG_FREQ_START       ((uint8_t)0x82)    /*!< Start frequency */
#define AD5933_REG_FREQ_INC         ((uint8_t)0x85)    /*!< Frequency increment */
#define AD5933_REG_INC_NUM          ((uint8_t)0x88)    /*!< Number of increments */
#define AD5933_REG_SETTLING_CYCLES  ((uint8_t)0x8A)    /*!< Number of settling time cycles */
#define AD5933_REG_STATUS           ((uint8_t)0x8F)    /*!< Status */
#define AD5933_REG_TEMP_DATA        ((uint8_t)0x92)    /*!< Temperature data */
#define AD5933_REG_REAL_DATA        ((uint8_t)0x94)    /*!< Real data MSB*/
#define AD5933_REG_REAL_DATA_2      ((uint8_t)0x95)    /*!< Real Data LSB*/
#define AD5933_REG_IMAG_DATA        ((uint8_t)0x96)    /*!< Imaginary data MSB*/
#define AD5933_REG_IMAG_DATA_2      ((uint8_t)0x97)    /*!<Imaginary data LSB*/

// AD5933 Block Commands
#define AD5933_BLOCK_WRITE      ((uint8_t)0xA0)
#define AD5933_BLOCK_READ       ((uint8_t)0xA1)
#define AD5933_ADDR_POINTER     ((uint8_t)0xB0)

// AD5933_CONTROL_FUNCTION(x) options
#define AD5933_FUNCTION_INIT_START_FREQ     ((uint8_t)0x1)
#define AD5933_FUNCTION_START_SWEEP         ((uint8_t)0x2)
#define AD5933_FUNCTION_INC_FREQ            ((uint8_t)0x3)
#define AD5933_FUNCTION_REPEAT_FREQ         ((uint8_t)0x4)
#define AD5933_FUNCTION_MEASURE_TEMP        ((uint8_t)0x9)
#define AD5933_FUNCTION_POWER_DOWN          ((uint8_t)0xA)
#define AD5933_FUNCTION_STANDBY             ((uint8_t)0xB)

// AD5933_REG_CONTROL_HB Bits
#define AD5933_CONTROL_FUNCTION(x)  ((x) << 4)
#define AD5933_CONTROL_RANGE(x)     ((x) << 1)

// AD5933_REG_STATUS Bits
#define AD5933_STAT_TEMP_VALID  (0x1 << 0)
#define AD5933_STAT_DATA_VALID  (0x1 << 1)
#define AD5933_STAT_SWEEP_DONE  (0x1 << 2)

#define INTERNAL_CLOCK_FREQ 16000000

#define START_FREQ 33000
#define NUM_INCR 1
#define FREQ_INCR 1000
#define CALIBRATION_NUM_INCR 3

// #define BIO_I2C_ADDR 0x0D  // AD5933 address

/**
 * @brief Initialize I2C master for bio-impedance device
 */
// esp_err_t bio_impedance_i2c_master_init(void);

/**
 * @brief Initialize the bio-impedance device
 */
esp_err_t bio_impedance_init(void);

/**
 * @brief Calibrate the bio-impedance measurement
 */
void bio_impedance_calibrate(void);

/**
 * @brief Perform bio-impedance measurement
 * @param real_arr Array to store real parts
 * @param imag_arr Array to store imaginary parts
 */
void bio_impedance_measure(signed short* real_arr, signed short* imag_arr);

/**
 * @brief Calculate impedance from real and imaginary parts
 * @param gainFactor Gain factor from calibration
 * @param real Real part
 * @param imag Imaginary part
 * @return Calculated impedance
 */
double bio_impedance_calculate_impedance(double gainFactor, signed short real, signed short imag);

/**
 * @brief Calculate gain factor from calibration
 * @param Z_calibration Calibration impedance
 * @param magnitude Measured magnitude
 * @return Gain factor
 */
double bio_impedance_gain_factor_calibration(double Z_calibration, double magnitude);

/**
 * @brief Calculate magnitude from real and imaginary
 * @param real Real part
 * @param imag Imaginary part
 * @return Magnitude
 */
double bio_impedance_calc_magnitude(int16_t real, int16_t imag);

/**
 * @brief Calculate phase angle
 * @param R Real part
 * @param I Imaginary part
 * @param undefined Flag if undefined
 * @return Phase angle
 */
double bio_impedance_arctan_phase_angle(double R, double I, bool* undefined);

/**
 * @brief Compensate real and imaginary with system phase
 * @param impedance Impedance
 * @param phase Phase
 * @param system_phase System phase
 * @param comp_real Compensated real
 * @param comp_imag Compensated imaginary
 */
void bio_impedance_compensated_real_and_imag(double impedance, double phase, double system_phase, double* comp_real, double* comp_imag);

/**
 * @brief Calibrate system phases
 * @param system_phases Array to store system phases
 * @param real Real array
 * @param imag Imaginary array
 * @param num_incr Number of increments
 */
void bio_impedance_system_phase_calibration(double* system_phases, int16_t* real, int16_t* imag, int num_incr);

#endif
