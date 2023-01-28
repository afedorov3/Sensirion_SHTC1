#ifndef SHTC1_H
#define SHTC1_H

#include <stdint.h>
#include <stddef.h>

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* I2C device address */
#ifndef SHTC1_ADDRESS
#define SHTC1_ADDRESS             UINT8_C(0x70)
#endif

/* API result codes */
#define SHTC1_OK                  INT8_C(0)
#define SHTC1_E_NULL_PTR          INT8_C(-1)
#define SHTC1_E_DEV_NOT_FOUND     INT8_C(-2)
#define SHTC1_E_INVALID_LEN       INT8_C(-3)
#define SHTC1_E_COMM_FAIL         INT8_C(-4)
#define SHTC1_E_SLEEP_MODE_FAIL   INT8_C(-5)

/* Chip ID */
#define SHTC1_ID_ANY(ID)          (((ID) & 0x003F) == 0x0007)
#define SHTC1_ID_SHTC1(ID)        (((ID) & 0x083F) == 0x0007)
#define SHTC1_ID_SHTC3(ID)        (((ID) & 0x083F) == 0x0807)

/* Measurement options */
#define SHTC1_MEAS_NORMAL         UINT8_C(0)
#define SHTC1_MEAS_LOW_POWER      UINT8_C(1)
#define SHTC1_MEAS_CLK_STRETCH    UINT8_C(2)

/* Timings */
#define SHTC1_RESET_TIME_US       UINT16_C(250)
#define SHTC1_MEAS_DURATION_US    UINT16_C(12100)
#define SHTC1_MEAS_DURATION_LP_US UINT16_C(800)

/* I2C I/O return type */
#ifndef SHTC1_INTF_RET_TYPE
#define SHTC1_INTF_RET_TYPE       int8_t
#endif

/* I2C I/O success code */
#ifndef SHTC1_INTF_RET_SUCCESS
#define SHTC1_INTF_RET_SUCCESS    UINT8_C(0)
#endif

typedef SHTC1_INTF_RET_TYPE (*shtc1_read_fptr_t)(uint8_t *data, uint16_t len,
                                                 void *intf_ptr);
typedef SHTC1_INTF_RET_TYPE (*shtc1_write_fptr_t)(const uint8_t *data, uint16_t len,
                                                  void *intf_ptr);
typedef void (*shtc1_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

struct shtc1_dev
{
    // Chip ID retrivied from sensor during initialization
    uint16_t chip_id;

    // Pointer to interface specific data that will be passed to read/write functions
    void *intf_ptr;

    // Pointer to I2C plain read function
    shtc1_read_fptr_t read;

    // Pointer to I2C plain write function
    shtc1_write_fptr_t write;

    // Pointer to microsecond delay function
    shtc1_delay_us_fptr_t delay_us;

    // I2C read/write result code storage
    SHTC1_INTF_RET_TYPE intf_rslt;
};

/*********************************************************************
 * @fn      shtc1_init
 * @brief   Checks for sensor presence and match, then initializes the
 *          sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t   shtc1_init(struct shtc1_dev *dev);

/*********************************************************************
 * @fn      shtc1_soft_reset
 * @brief   Issues soft reset command to the sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t   shtc1_soft_reset(struct shtc1_dev *dev);

/*********************************************************************
 * @fn      shtc1_sleep
 * @brief   Issues sleep command to the sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t   shtc1_sleep(struct shtc1_dev *dev);

/*********************************************************************
 * @fn      shtc1_wake_up
 * @brief   Issues wake up command to the sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t   shtc1_wake_up(struct shtc1_dev *dev);

/*********************************************************************
 * @fn      shtc1_start_measurement
 * @brief   Triggers low power measurement, clock stretch disable,
 *          read temperature first
 * @param   dev - pointer to device context
 * @param   options - measurement options bit field
 *            SHTC1_MEAS_LOW_POWER    - low power mode
 *            SHTC1_MEAS_CLK_STRETCH - clock stretch enabled
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t   shtc1_start_measurement(struct shtc1_dev *dev, uint8_t flags);

/*********************************************************************
 * @fn      shtc1_get_trh
 * @brief   Reads and validates measurement data from the sensor
 * @param   dev - pointer to device context
 * @param   t - pointer to the temperature storage, unit is 1/100 °C
 * @param   rh - pointer to the humidity storage,   unit is 1/100 %RH
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t   shtc1_get_trh(struct shtc1_dev *dev, int16_t *t, uint16_t *rh);

/*********************************************************************
 * @fn      shtc1_meas_time
 * @brief   Report ongoing measurement time (call AFTER starting a measurement)
 * @param   none
 * @return  Ongoing measurement time in microseconds
 */
uint16_t shtc1_meas_time_us(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SHTC1_H */
