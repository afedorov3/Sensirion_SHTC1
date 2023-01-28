/*****************************************************************
 *          Bare-bones Sensirion SHTC1 / SHTC3 driver            *
 *        Loosely based on Bosch Sensortec BME280 driver         *
 *                                                               *
 * Revision 20230128                                             *
 *****************************************************************/

#include "shtc1.h"

/* Commands */
#define SHTC1_CMD_RESET           UINT16_C(0x805D)
#define SHTC1_CMD_CHIP_ID         UINT16_C(0xEFC8)
#define SHTC1_CMD_SLEEP           UINT16_C(0xB098)
#define SHTC1_CMD_WAKEUP          UINT16_C(0x3517)
#define SHTC1_CMD_MEAS_TRH        UINT16_C(0x7866)
#define SHTC1_CMD_MEAS_RHT        UINT16_C(0x58E0)
#define SHTC1_CMD_MEAS_LP_TRH     UINT16_C(0x609C)
#define SHTC1_CMD_MEAS_LP_RHT     UINT16_C(0x401A)
#define SHTC1_CMD_MEAS_CS_TRH     UINT16_C(0x7CA2)
#define SHTC1_CMD_MEAS_CS_RHT     UINT16_C(0x5C24)
#define SHTC1_CMD_MEAS_LP_CS_TRH  UINT16_C(0x6458)
#define SHTC1_CMD_MEAS_LP_CS_RHT  UINT16_C(0x44DE)

#define SHTC1_MSB16(a) ((uint8_t)(((uint16_t)a) >> 8))
#define SHTC1_LSB16(a) ((uint8_t)(((uint16_t)a) & 0xFF))

/* CRC */
#define CRC8_POLYNOMIAL           0x31
#define CRC8_INIT                 0xFF
#define CRC8_LEN                  1

static int8_t null_ptr_check(const struct shtc1_dev *dev);
static int8_t issue_cmd(struct shtc1_dev *dev, uint16_t cmd);
static int8_t read_data(struct shtc1_dev *dev, uint8_t *buf, uint16_t len);
static int8_t read_id(struct shtc1_dev *dev, uint16_t *id);
static uint8_t generate_crc(const uint8_t* data, uint16_t count);

static uint8_t last_meas_opts = 0;

/* PUBLIC */

/*********************************************************************
 * @fn      shtc1_init
 * @brief   Checks for sensor presence and match, then initializes the
 *          sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t shtc1_init(struct shtc1_dev *dev)
{
    int8_t rslt;

    uint8_t try_count = 5;
    uint16_t chip_id;

    rslt = null_ptr_check(dev);
    if (rslt == SHTC1_OK)
    {
        dev->chip_id = 0;
        while (try_count)
        {
            shtc1_wake_up(dev);
            dev->delay_us(SHTC1_RESET_TIME_US, dev->intf_ptr);
            rslt = read_id(dev, &chip_id);
            if (rslt == SHTC1_OK && SHTC1_ID_ANY(chip_id))
            {
                dev->chip_id = chip_id;

                rslt = shtc1_soft_reset(dev);
                if (rslt == SHTC1_OK)
                    dev->delay_us(SHTC1_RESET_TIME_US, dev->intf_ptr);

                break;
            }

            dev->delay_us(10000, dev->intf_ptr);
            --try_count;
        }

        if (!try_count)
        {
            rslt = SHTC1_E_NO_DEV;
        }
    }

    return rslt;
}

/*********************************************************************
 * @fn      shtc1_soft_reset
 * @brief   Issues soft reset command to the sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t shtc1_soft_reset(struct shtc1_dev *dev)
{
    return issue_cmd(dev, SHTC1_CMD_RESET);
}

/*********************************************************************
 * @fn      shtc1_sleep
 * @brief   Issues sleep command to the sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t shtc1_sleep(struct shtc1_dev *dev)
{
    // SHTC1 doesn't support sleep mode
    return SHTC1_ID_SHTC1(dev->chip_id) ? SHTC1_OK : issue_cmd(dev, SHTC1_CMD_SLEEP);
}

/*********************************************************************
 * @fn      shtc1_wake_up
 * @brief   Issues wake up command to the sensor
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t shtc1_wake_up(struct shtc1_dev *dev)
{
    return SHTC1_ID_SHTC1(dev->chip_id) ? SHTC1_OK : issue_cmd(dev, SHTC1_CMD_WAKEUP);
}

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
int8_t shtc1_start_measurement(struct shtc1_dev *dev, uint8_t options)
{
    uint16_t cmds[] = {
                        [SHTC1_MEAS_NORMAL] = SHTC1_CMD_MEAS_TRH,
                        [SHTC1_MEAS_LOW_POWER] = SHTC1_CMD_MEAS_LP_TRH,
                        [SHTC1_MEAS_CLK_STRETCH] = SHTC1_CMD_MEAS_CS_TRH,
                        [SHTC1_MEAS_LOW_POWER | SHTC1_MEAS_CLK_STRETCH] = SHTC1_CMD_MEAS_LP_CS_TRH
                      };

    // SHTC1 doesn't support low power measurement
    if (SHTC1_ID_SHTC1(dev->chip_id))
        options &= ~(SHTC1_MEAS_LOW_POWER);

    last_meas_opts = options & 0x3;
    return issue_cmd(dev, cmds[last_meas_opts]);
}

/*********************************************************************
 * @fn      shtc1_get_trh
 * @brief   Reads and validates measurement data from the sensor
 * @param   dev - pointer to device context
 * @param   t - pointer to the temperature storage, unit is 1/100 °C
 * @param   rh - pointer to the humidity storage,   unit is 1/100 %RH
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
int8_t shtc1_get_trh(struct shtc1_dev *dev, int16_t *t, uint16_t *rh)
{
    int8_t rslt;
    uint8_t buf[6];
    uint8_t crc;

    rslt = read_data(dev, buf, sizeof(buf));
    if (rslt == SHTC1_OK)
    {
        crc = generate_crc(buf, 2);
        if (crc != buf[2])
            return SHTC1_E_COMM_FAIL;
        crc = generate_crc(&buf[3], 2);
        if (crc != buf[5])
            return SHTC1_E_COMM_FAIL;

        /* fixed point data rounded to second decimal place */
        *t  = (((uint32_t)buf[0] << 8 | buf[1]) * 17500 + 32768) / 65536 - 4500;
        *rh = (((uint32_t)buf[3] << 8 | buf[4]) * 10000 + 32768) / 65536;
    }

    return rslt;
}

/*********************************************************************
 * @fn      shtc1_meas_time
 * @brief   Report ongoing measurement time (call AFTER starting a measurement)
 * @param   none
 * @return  Ongoing measurement time in microseconds
 */
uint16_t shtc1_meas_time_us(void)
{
    return (last_meas_opts & SHTC1_MEAS_LOW_POWER ? SHTC1_MEAS_DURATION_LP_US : SHTC1_MEAS_DURATION_US);
}

/* PRIVATE */

/*********************************************************************
 * @fn      null_ptr_check
 * @brief   Validates device context
 * @param   dev - pointer to device context
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
static int8_t null_ptr_check(const struct shtc1_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        rslt = SHTC1_E_NULL_PTR;
    }
    else
    {
        rslt = SHTC1_OK;
    }

    return rslt;
}

/*********************************************************************
 * @fn      issue_cmd
 * @brief   Issues command to the sensor
 * @param   dev - pointer to device context
 * @param   cmd - command to issue
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
static int8_t issue_cmd(struct shtc1_dev *dev, uint16_t cmd)
{
    int8_t rslt;
    uint8_t tx[2] = { SHTC1_MSB16(cmd), SHTC1_LSB16(cmd) };

    rslt = null_ptr_check(dev);
    if (rslt == SHTC1_OK)
    {
        dev->intf_rslt = dev->write(tx, sizeof(tx), dev->intf_ptr);
        if (dev->intf_rslt != SHTC1_INTF_RET_SUCCESS)
        {
            rslt = SHTC1_E_COMM_FAIL;
        }
    }

    return rslt;
}

/*********************************************************************
 * @fn      read_data
 * @brief   Reads data from the sensor
 * @param   dev - pointer to device context
 * @param   buf - target buffer
 * @param   len - bytes to read
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
static int8_t read_data(struct shtc1_dev *dev, uint8_t *buf, uint16_t len)
{
    int8_t rslt;

    rslt = null_ptr_check(dev);
    if (rslt == SHTC1_OK)
    {
        dev->intf_rslt = dev->read(buf, len, dev->intf_ptr);
        if (dev->intf_rslt != SHTC1_INTF_RET_SUCCESS)
        {
            rslt = SHTC1_E_COMM_FAIL;
        }
    }

    return rslt;
}

/*********************************************************************
 * @fn      read_id
 * @brief   Retrives and validates sensor ID
 * @param   dev - pointer to device context
 * @param   id - pointer to ID storage
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
static int8_t read_id(struct shtc1_dev *dev, uint16_t *id)
{
    int8_t rslt;
    uint8_t buf[3];
    uint8_t crc;

    rslt = issue_cmd(dev, SHTC1_CMD_CHIP_ID);
    if (rslt == SHTC1_OK)
    {
        rslt = read_data(dev, buf, sizeof(buf));
        if (rslt == SHTC1_OK)
        {
            crc = generate_crc(buf, 2);
            if (crc == buf[2])
            {
                *id = buf[0] << 8 | buf[1];
            }
            else
            {
              rslt = SHTC1_E_COMM_FAIL;
            }
        }
    }

    return rslt;
}

/*********************************************************************
 * @fn      generate_crc
 * @brief   Calculates CRC code for the data of given length
 * @param   data - data buffer
 * @param   count - length of the data
 * @return  SHTC1_OK when successful, otherwise SHTC1_E_*
 */
static uint8_t generate_crc(const uint8_t* data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    for (current_byte = 0; current_byte < count; ++current_byte)
    {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}
