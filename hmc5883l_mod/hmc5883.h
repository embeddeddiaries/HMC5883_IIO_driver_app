#ifndef HMC5883_H
#define HMC5883_H

#include <linux/regmap.h>
#include <linux/iio/iio.h>

struct hmc5883_data{
    struct device *dev;
    struct regmap *map;
    unsigned short buffer[8];
};


int hmc5883_reg_write(struct hmc5883_data *hmc,uint8_t addr,uint8_t value);
int hmc5883_reg_read(struct hmc5883_data *hmc,uint8_t addr,uint8_t *value, uint8_t size);

#endif


