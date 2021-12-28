#include<linux/init.h>
#include<linux/module.h>
#include<linux/fs.h>
#include<linux/cdev.h>
#include<linux/slab.h>
#include<linux/kernel.h>
#include<linux/version.h>
#include<linux/errno.h>
#include<linux/device.h>
#include<linux/mutex.h>
#include<linux/i2c.h>
#include<linux/delay.h>
#include<linux/uaccess.h>
#include<linux/ioctl.h>

#include<linux/regmap.h>

#include<linux/iio/sysfs.h>
#include<linux/iio/iio.h>
#include<linux/iio/buffer.h>
#include<linux/iio/events.h>
#include<linux/iio/triggered_buffer.h>
#include<linux/iio/trigger_consumer.h>
#include<linux/interrupt.h>

#include"hmc5883.h"

#define	    CONFIG_A_REG	0x00
#define	    CONFIG_B_REG	0x01
#define	    MODE_REG		0x02
#define	    DATA_X_MSB		0x03
#define	    DATA_Z_MSB		0x05
#define	    DATA_Y_MSB		0x07
#define	    STATUS_REG		0x09
#define	    IDENT_A_REG		0x0A
#define	    IDENT_B_REG		0x0B
#define	    IDENT_C_REG		0x0C
#define	    MAX_REG_ADDR	0x0C

MODULE_AUTHOR("Jaggu");
MODULE_LICENSE("GPL");

/* Scaling factors: 10000000/Gain */
static const int hmc5883_regval_to_nanoscale[] = {
	7299, 9174, 12195, 15152, 22727, 25641, 30303, 43478
};

static const int hmc5883_regval_to_samp_freq[][2] = {
	{0, 750000}, {1, 500000}, {3, 0}, {7, 500000}, {15, 0}, {30, 0},
	{75, 0}
};

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("0.750000 1.500000 3.0 7.500000 15.0 30.0 75.0");

static ssize_t hmc5883_show_scale_avail(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
    size_t len = 0;
    len = scnprintf(buf,PAGE_SIZE - len,"0.000007812 0.000009766 0.000013021 0.000016287 0.000024096 0.000027701 0.000032573 0.000045662");
    buf[len-1] = '\n';
    return len;
}

static IIO_DEVICE_ATTR(scale_available, S_IRUGO,hmc5883_show_scale_avail, NULL, 0);

static struct attribute *hmc5883_attributes[] = {
	&iio_dev_attr_scale_available.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group hmc5883_group = {
	.attrs = hmc5883_attributes,
};

int hmc5883_init(struct hmc5883_data *hmc)
{
    uint8_t identity_data[3] = {0,},status;
    hmc5883_reg_read(hmc, IDENT_A_REG,identity_data,3);
    if((identity_data[0] != 'H') || (identity_data[1] != '4') || (identity_data[2] != '3'))
	return -1;
    
    pr_info("HMC5883 Found!!!\r\n");

    hmc5883_reg_write(hmc,CONFIG_A_REG,0x70);
    
    hmc5883_reg_write(hmc,CONFIG_B_REG,0x20);
    
    hmc5883_reg_write(hmc,MODE_REG,0x00);
    
    hmc5883_reg_read(hmc, MODE_REG, &status, 1);
    //pr_info("Mode= %x\r\n",status);

    return 0;
}


int hmc5883_reg_write(struct hmc5883_data *hmc,uint8_t addr,uint8_t value){

    int ret = 0,reg_value;
    
    reg_value = (int )value;
    if(regmap_write(hmc->map, addr, reg_value) < 0){
	pr_err("Regmap write fail\r\n");
	ret = -1;
    }
    return ret;
}

int hmc5883_reg_read(struct hmc5883_data *hmc,uint8_t addr,uint8_t *value, uint8_t size){

    int single_value = 0x00,ret = 0;
    if(size == 1){
	    if(regmap_read(hmc->map, addr, &single_value) < 0){
		pr_err("Regmap read fail\r\n");
		ret = -1;
	    }
	    *value = single_value & 0xFF;
    }
    else{
	    if(regmap_bulk_read(hmc->map, addr, value, size) < 0){
		 pr_err("Regmap read bulk fail\r\n");
		ret = -1;
	    }
//	    pr_info("Value Read 1 = %d  2 = %d\r\n",value[0],value[1]);
    }
    return ret;
}



bool hmc5883_readable_reg(struct device *dev, unsigned int reg)
{
    if(reg >= 0x00 && reg <= 0x0C)
	return true;
    else
	return false;
}

bool hmc5883_writeable_reg(struct device *dev, unsigned int reg)
{
    if(reg >= 0x00 && reg <= 0x02)
	return true;
    else
	return false;
}

int hmc5883_checkDRDY(struct hmc5883_data *hmc ){

    uint8_t status = 0;

    do{
	hmc5883_reg_read(hmc, STATUS_REG, &status, 1);
	pr_info("Status = %x\r\n",status);
	//udelay(100);
	mdelay(10);
    }while(!(status & 0x01));
    return 0;
}

static int hmc5883_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask){

}

static int hmc5883_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	uint8_t reg_val;
	struct hmc5883_data *hmc= iio_priv(indio_dev);
	uint16_t xyz;
    	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	    {
		hmc5883_checkDRDY(hmc);
		if(hmc5883_reg_read(hmc, DATA_X_MSB, (uint8_t*)hmc->buffer, 6) < 0){
		    return -EINVAL;
		}

		if(chan->address == DATA_X_MSB){
		    xyz = (hmc->buffer[0] << 8) |  hmc->buffer[1];
		    *val = sign_extend32(xyz,15);
		}
		else if(chan->address == DATA_Z_MSB){
		    xyz = (hmc->buffer[2] << 8) |  hmc->buffer[3];
		    *val = sign_extend32(xyz,15);
		}
		else if(chan->address == DATA_Y_MSB){
		    xyz = (hmc->buffer[4] << 8) |  hmc->buffer[5];
		    *val = sign_extend32(xyz,15);
		}
		//pr_info("xyz = %d",*val);
		return IIO_VAL_INT;
		break;
	    }
	case IIO_CHAN_INFO_SCALE:
	    {
		if(hmc5883_reg_read(hmc, CONFIG_B_REG, &reg_val, 1) < 0){
		    return -EINVAL;
		}
		reg_val >>= 5;
		*val = 0;
		*val2 = hmc5883_regval_to_nanoscale[reg_val];
		return IIO_VAL_INT_PLUS_NANO;
	    }
	case IIO_CHAN_INFO_SAMP_FREQ:
	    {
		if(hmc5883_reg_read(hmc, CONFIG_A_REG, &reg_val, 1) < 0){
		    return -EINVAL;
		}
		reg_val = ((reg_val >> 2) & 0x07);
		*val = hmc5883_regval_to_samp_freq[reg_val][0];
		*val2 = hmc5883_regval_to_samp_freq[reg_val][1];
		return IIO_VAL_INT_PLUS_MICRO;
	    }
	}
	return IIO_VAL_INT;
}


const struct iio_info hmc5883_info = {
    .attrs = &hmc5883_group,
    .read_raw = hmc5883_read_raw,
    .write_raw = hmc5883_write_raw,
};

#define HMC5883_CHANNEL(index, axis, addr){		    \
    .type = IIO_MAGN,					    \
    .address = addr,					    \
    .modified = 1,					    \
    .channel2 = IIO_MOD_##axis,				    \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	    \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
			    BIT(IIO_CHAN_INFO_SAMP_FREQ),   \
    .scan_index = index,				    \
    .scan_type = {					    \
	.sign = 's',					    \
	.realbits = 16,					    \
	.storagebits = 16,				    \
	.shift = 0,					    \
	.endianness = IIO_BE,				    \
    },							    \
}

static const struct iio_chan_spec hmc5883_channels[] = {
    HMC5883_CHANNEL(0,X,DATA_X_MSB),
    HMC5883_CHANNEL(1,Y,DATA_Y_MSB),
    HMC5883_CHANNEL(2,Z,DATA_Z_MSB),
    IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const unsigned long hmc5883_scan_masks[] = {0x07, 0x00};


static irqreturn_t hmc5883_top_half(int irq, void *p){
    return IRQ_WAKE_THREAD;
}

static irqreturn_t hmc5883_bottom_half(int irq, void *p){

    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct hmc5883_data *hmc = iio_priv(indio_dev);

    hmc5883_checkDRDY(hmc);
    if(hmc5883_reg_read(hmc, DATA_X_MSB,(uint8_t *)hmc->buffer, 6) < 0){
	return -EINVAL;
    }

    pr_info("Trig buf read\r\n");
    /*
    if(iio_push_to_buffers(indio_dev,data_r) != 0){
        pr_info("IIO push fail\r\n");	
    }
    */
    iio_push_to_buffers_with_timestamp(indio_dev,hmc->buffer, iio_get_time_ns(indio_dev));
    iio_trigger_notify_done(indio_dev->trig);

    return IRQ_HANDLED;
}

void create_oops(void){
    *(int*)0 = 0;
}

int hmc5883_probe(struct i2c_client *client, const struct i2c_device_id *id){

    int ret;
    struct regmap *hmc_regmap = NULL;
    struct hmc5883_data *hmc;
    struct iio_dev *indio_dev;

    struct regmap_config hmc_reg_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX_REG_ADDR,
	.readable_reg = hmc5883_readable_reg,
	.writeable_reg = hmc5883_writeable_reg,
    };

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(struct hmc5883_data));
    if(!indio_dev){
	return -ENOMEM;
    }
    
    dev_set_drvdata(&client->dev, indio_dev);
    hmc = iio_priv(indio_dev);
    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &hmc5883_info;
    indio_dev->name = id->name;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = hmc5883_channels;
    indio_dev->num_channels = ARRAY_SIZE(hmc5883_channels);
    indio_dev->driver_module = THIS_MODULE;
    indio_dev->available_scan_masks = hmc5883_scan_masks;

    hmc->map = regmap_init_i2c(client, &hmc_reg_config);
    if(IS_ERR(hmc_regmap))
    {
	return PTR_ERR(hmc_regmap);
    }

    hmc5883_init(hmc);

   // create_oops();

    //ret = iio_triggered_buffer_setup(indio_dev,hmc5883_top_half,hmc5883_bottom_half,NULL);
    ret = iio_triggered_buffer_setup(indio_dev,NULL,hmc5883_bottom_half,NULL);
    if(ret < 0){
	pr_err("iio trigger buffer fail\r\n");
    }

    iio_device_register(indio_dev);
    return 0;
}

int hmc5883_remove(struct i2c_client *client){
   
    struct iio_dev *indio_dev = dev_get_drvdata(&client->dev);
    iio_device_unregister(indio_dev);
    return 0;
}


static struct i2c_device_id hmc5883_id_table[] = {
    {"hmc5883",0},
    {},
};

static const struct of_device_id hmc5883_of_table[] = {
    { .compatible = "hmc5883"},
};


static struct i2c_driver hmc5883_i2c_driver = {
    .driver = {
	.name = "hmc5883",
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(hmc5883_of_table),
    },
    .id_table = hmc5883_id_table,
    .probe = hmc5883_probe,
    .remove = hmc5883_remove,
};


module_i2c_driver(hmc5883_i2c_driver);









