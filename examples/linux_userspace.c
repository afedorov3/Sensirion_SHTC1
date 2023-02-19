#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "shtc1.h"

int8_t user_i2c_read(uint8_t *data, uint16_t len, void *intf_ptr);
int8_t user_i2c_write(const uint8_t *data, uint16_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);

/* Structure that contains identifier details used in example */
struct identifier
{
    /* Variable to hold device address */
    uint8_t dev_addr;

    /* Variable that contains file descriptor */
    int8_t fd;
};

int main(int argc, char* argv[])
{
    struct shtc1_dev dev;
    struct identifier id;

    /* Variable to define the result */
    int8_t rslt = SHTC1_OK;

    id.dev_addr = SHTC1_ADDRESS;

    if (argc < 2)
    {
        fprintf(stderr, "Missing argument for i2c bus.\n");
        exit(1);
    }

    if ((id.fd = open(argv[1], O_RDWR)) < 0)
    {
        fprintf(stderr, "Failed to open the i2c bus %s\n", argv[1]);
        exit(1);
    }

    if (ioctl(id.fd, I2C_SLAVE, id.dev_addr) < 0)
    {
        fprintf(stderr, "Failed to acquire bus access and/or talk to slave.\n");
        exit(1);
    }

    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
    dev.intf_ptr = &id;

    rslt = shtc1_init(&dev);
    if (rslt != SHTC1_OK)
    {
        fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
        exit(1);
    }

    printf("Temperature Humidity\n");
    do {
        rslt = shtc1_wake_up(&dev);
        if (rslt != SHTC1_OK)
        {
            fprintf(stderr, "Failed to wake up the device (code %+d).\n", rslt);
            exit(1);
        }
        usleep(SHTC1_RESET_TIME_US);
        rslt = shtc1_start_measurement(&dev, SHTC1_MEAS_NORMAL);
        if (rslt != SHTC1_OK)
        {
            fprintf(stderr, "Failed to initiate measurement (code %+d).\n", rslt);
            exit(1);
        }
        usleep(shtc1_meas_time_us());
        int16_t t = 0;
        uint16_t rh = 0;
        rslt = shtc1_get_trh(&dev, &t, &rh);
        if (rslt != SHTC1_OK)
        {
            fprintf(stderr, "Failed to acquire data (code %+d).\n", rslt);
            usleep(15000);
        }
        rslt = shtc1_sleep(&dev);
        if (rslt != SHTC1_OK)
        {
            fprintf(stderr, "Failed to put the device to sleep (code %+d).\n", rslt);
            exit(1);
        }

        printf("%0.2f %0.2f\n", (double)t / 100.0, (double)rh / 100.0);

        fflush(stdout);
        sleep(5);
    } while(1);

    return 0;
}

int8_t user_i2c_read(uint8_t *data, uint16_t len, void *intf_ptr)
{
    int8_t rslt = SHTC1_INTF_RET_SUCCESS;
    struct identifier id;

    id = *((struct identifier *)intf_ptr);

    if (read(id.fd, data, len) < (ssize_t)len)
    {
         rslt = SHTC1_E_COMM_FAIL;
    }

    return rslt;
}

int8_t user_i2c_write(const uint8_t *data, uint16_t len, void *intf_ptr)
{
    int8_t rslt = SHTC1_INTF_RET_SUCCESS;
    struct identifier id;

    id = *((struct identifier *)intf_ptr);

    if (write(id.fd, data, len) < (ssize_t)len)
    {
         rslt = SHTC1_E_COMM_FAIL;
    }

    return rslt;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr; // unused
    usleep(period);
}
