#include "ist8310driver.h"
#include "ist8310driver_middleware.h"

#define MAG_SEN 0.3f
#define IST8310_WHO_AM_I 0x00
#define IST8310_WHO_AM_I_VALUE 0x10
#define IST8310_WRITE_REG_NUM 4

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},
    {0x41, 0x09, 0x02},
    {0x42, 0xC0, 0x03},
    {0x0A, 0x0B, 0x04}
};

uint8_t ist8310_init(void)
{
    uint8_t res = 0;
    ist8310_GPIO_init();
    ist8310_com_init();
    ist8310_RST_L();
    ist8310_delay_ms(50);
    ist8310_RST_H();
    ist8310_delay_ms(50);
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE) return IST8310_NO_SENSOR;
    ist8310_delay_ms(1);
    for (uint8_t i = 0; i < IST8310_WRITE_REG_NUM; i++) {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[i][0], ist8310_write_reg_data_error[i][1]);
        ist8310_delay_ms(1);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[i][0]);
        ist8310_delay_ms(1);
        if (res != ist8310_write_reg_data_error[i][1]) return ist8310_write_reg_data_error[i][2];
    }
    return IST8310_NO_ERROR;
}

void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{
    if (status_buf[0] & 0x01) {
        int16_t temp;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;
        for (int i = 0; i < 3; i++) {
            temp = (int16_t)((status_buf[i*2+2] << 8) | status_buf[i*2+1]);
            ist8310_real_data->mag[i] = MAG_SEN * temp;
        }
    } else {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

void ist8310_read_mag(float mag[3])
{
    uint8_t buf[6];
    ist8310_IIC_read_muli_reg(0x03, buf, 6);
    for (int i = 0; i < 3; i++) {
        int16_t temp = (int16_t)((buf[i*2+1] << 8) | buf[i*2]);
        mag[i] = MAG_SEN * temp;
    }
}
