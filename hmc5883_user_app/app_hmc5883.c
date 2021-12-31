
#define _GNU_SOURCE
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>
#include<stdlib.h>
#include<sys/ioctl.h>
#include <dirent.h>
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/dir.h>

#include "iio_utils.h"

const char *type_device = "iio:device";

static inline int check_prefix(const char *str, const char *prefix)
{
	return strlen(str) > strlen(prefix) &&
	       strncmp(str, prefix, strlen(prefix)) == 0;
}

static inline int check_postfix(const char *str, const char *postfix)
{
	return strlen(str) > strlen(postfix) &&
	       strcmp(str + strlen(str) - strlen(postfix), postfix) == 0;
}

static int dump_channels(const char *dev_dir_name)
{
    DIR *dp;
    struct dirent *ent;
    char string[IIO_MAX_NAME_LENGTH]; 
    char ch,readings,str_value[IIO_MAX_NAME_LENGTH];
    int option;

    dp = opendir(dev_dir_name);
    if (!dp)
	return -errno;

    while (ent = readdir(dp), ent){
	if (check_prefix(ent->d_name, "in_") &&
		(check_postfix(ent->d_name, "_raw") ||
		 check_postfix(ent->d_name, "_input")))
	    printf("   %-10s\n", ent->d_name);
    }
    rewinddir(dp);
    while (ent = readdir(dp), ent){
	if(check_postfix(ent->d_name, "_frequency")){
	    read_sysfs_string("in_magn_sampling_frequency", dev_dir_name, string);
	    printf("Magnetic sampling frequency set = %s\r\n",string);
	}
	if(check_postfix(ent->d_name, "_scale")){
	    read_sysfs_string("in_magn_scale", dev_dir_name, string);
	    printf("Magnetic scale set = %s\r\n",string);
	}
    }

    printf("Sensor read option \n1. Sysfs read - single channel \n2. Triggered buffer read\n");
    scanf("%d",&option);
    getchar();
    if(option == 1){
	printf("Enter channel to read (x | y | z) - ");
	scanf("%c",&ch);

	if(ch == 'x'){
	    strcpy(string,"in_magn_x_raw");
	}
	else if(ch == 'y'){
	    strcpy(string,"in_magn_y_raw");
	}
	else if(ch == 'z'){
	    strcpy(string,"in_magn_z_raw");
	}
	printf("%s: \n",string);
	for(readings = 0; readings < 10; readings++){
	    read_sysfs_string(string, dev_dir_name, str_value);
	    printf("%s\n",str_value);

	}
    }
    else if(option == 2){

    }
    return (closedir(dp) == -1) ? -errno : 0;
}


static int dump_one_device(const char *dev_dir_name)
{
	char name[IIO_MAX_NAME_LENGTH];
	int dev_idx;
	int ret;

	ret = sscanf(dev_dir_name + strlen(iio_dir) + strlen(type_device), "%i",
		     &dev_idx);
	if (ret != 1)
		return -EINVAL;

	ret = read_sysfs_string("name", dev_dir_name, name);
	if (ret < 0)
		return ret;

	printf("Device %03d: %s\n", dev_idx, name);

	if(strcmp(name,"hmc5883") != 0){
		printf("HMC5883 Not found\r\n");
		return -EINVAL;
	}
	dump_channels(dev_dir_name);

	return 0;
}

int main(void){

    printf("HMC5883 user application\r\n");

    const struct dirent *ent;
    DIR *dp;
    int ret;
    int devices_found = 0;

    dp = opendir(iio_dir);
    if (!dp) {
	fprintf(stderr, "No industrial I/O devices available\n");
	return -ENODEV;
    }

    while (ent = readdir(dp), ent) {
	if (check_prefix(ent->d_name, type_device)) {
	    devices_found = 1;
	    char *dev_dir_name;

	    if (asprintf(&dev_dir_name, "%s%s", iio_dir,
			ent->d_name) < 0) {
		ret = -ENOMEM;
		goto error_close_dir;
	    }

	    ret = dump_one_device(dev_dir_name);
	    if (ret) {
		free(dev_dir_name);
		goto error_close_dir;
	    }

	    free(dev_dir_name);
	}
    }
    if(!devices_found){
	printf("No IIO devices found\r\n");
    }

error_close_dir:
	if (closedir(dp) == -1)
		perror("dump_devices(): Failed to close directory");

    return 0;
}
