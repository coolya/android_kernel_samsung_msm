/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

//CAMSENSOR 1/5" isx006

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>

#include "isx006.h"
#include <mach/camera.h>
#include <mach/vreg.h>

#define SENSOR_DEBUG 0
#define ISX006_DEBUG
#define VIEW_FUNCTION_CALL
//#define CONFIG_LOAD_FILE	//For tunning binary

#ifdef ISX006_DEBUG
#ifdef VIEW_FUNCTION_CALL
#define F_IN		printk( "[ISX006] %s +\n", __func__)
#define F_OUT	printk( "[ISX006] %s -\n", __func__)
#define F_INOUT	printk( "[ISX006] %s called \n", __func__)
#endif
#define cam_msleep(x)	do{printk( "[ISX006] %s ,msleep %d\n", __func__,x);msleep(x);}while(0)
#define cam_mdelay(x)	 do{printk( "[ISX006] %s ,msleep %d\n", __func__,x);mdelay(x);}while(0)
#else
#define cam_msleep(x)	msleep(x)
#define cam_mdelay(x)	mdelay(x)
#endif

static char nightshot_set = 0;

struct isx006_work {
	struct work_struct work;
};

static struct  isx006_work *isx006_sensorw;
static struct  i2c_client *isx006_client;

struct isx006_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

static struct isx006_ctrl *isx006_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(isx006_wait_queue);
DECLARE_MUTEX(isx006_sem);

static int16_t isx006_effect = CAMERA_EFFECT_OFF;

/* stop_af_operation is used to cancel the operation while doing, or even before it has started */
static volatile int stop_af_operation;

/* Here we store the status of AF; 0 --> AF not started, 1 --> AF started , 2 --> AF operation finished */
static int af_operation_status;

/*
 * Whenever there is an AF cancell request the timer is started. The cancel operation
 * is valid for 100ms after that it is expired
 */
//static struct timer_list af_cancel_timer;
//static void af_cancel_handler(unsigned long data);

/* Save the focus mode value, it can be marco or auto */
//static int af_mode;

//static int gLowLight = 0;
static int gCurrentScene = CAMSENSOR_SCENE_OFF;
//static int gCamsensor_init_complete = 0;

/* Interrupt */
#define ISX006_INT_OM_CHANGED			(1 << 0)
#define ISX006_INT_CM_CHANGED			(1 << 1)
#define ISX006_INT_JPEG_UPDATE			(1 << 2)
#define ISX006_INT_CAPNUM_END			(1 << 3)
#define ISX006_INT_AF_LOCK				(1 << 4)

enum isx006_oprmode {
	ISX006_OPRMODE_VIDEO = 0,
	ISX006_OPRMODE_IMAGE = 1,
};

enum isx006_frame_size {
	ISX006_PREVIEW_QCIF = 0,	/* 176x144 */
	ISX006_PREVIEW_D1,		/* 720x480 */
	ISX006_PREVIEW_VGA,		/* 640x480 */
	ISX006_PREVIEW_SVGA,		/* 800x600 */
	ISX006_PREVIEW_WSVGA,		/* 1024x600*/
	ISX006_CAPTURE_VGA, /* 640 x 480 */	
	ISX006_CAPTURE_WVGA, /* 800 x 480 */
	ISX006_CAPTURE_SVGA,		/* SVGA  - 800x600 */
	ISX006_CAPTURE_WSVGA,		/* SVGA  - 1024x600 */	
	ISX006_CAPTURE_W1MP, /* 1600 x 960 */
	ISX006_CAPTURE_2MP, /* UXGA  - 1600 x 1200 */
	ISX006_CAPTURE_W2MP, /* 35mm Academy Offset Standard 1.66  - 2048 x 1232, 2.4MP */	
	ISX006_CAPTURE_3MP, /* QXGA  - 2048 x 1536 */
	ISX006_CAPTURE_W4MP, /* WQXGA - 2560 x 1536 */
	ISX006_CAPTURE_5MP, /* 2560 x 1920 */	
};

struct isx006_enum_framesize {
	/* mode is 0 for preview, 1 for capture */
	enum isx006_oprmode mode;
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

/*
static struct isx006_enum_framesize isx006_framesize_list[] = {
	{ ISX006_OPRMODE_VIDEO, ISX006_PREVIEW_QCIF,	176,  144 },
	{ ISX006_OPRMODE_VIDEO, ISX006_PREVIEW_D1,	720,  480 },
	{ ISX006_OPRMODE_VIDEO, ISX006_PREVIEW_VGA,	640,  480 },
	{ ISX006_OPRMODE_VIDEO, ISX006_PREVIEW_SVGA,	800,  600 },
	{ ISX006_OPRMODE_VIDEO, ISX006_PREVIEW_WSVGA,	1024,  600 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_VGA,	640,  480 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_WVGA,	800,  480},	
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_SVGA,	800,  600 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_WSVGA,	1024,  600},	
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_W1MP,	1600,  960 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_2MP,	1600, 1200 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_W2MP,	2048, 1232 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_3MP,	2048, 1536 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_W4MP,	2560, 1536 },
	{ ISX006_OPRMODE_IMAGE, ISX006_CAPTURE_5MP,	2560, 1920 },
};
*/

struct isx006_version {
	unsigned int major;
	unsigned int minor;
};

struct isx006_date_info {
	unsigned int year;
	unsigned int month;
	unsigned int date;
};

enum isx006_runmode {
	ISX006_RUNMODE_NOTREADY,
	ISX006_RUNMODE_IDLE, 
	ISX006_RUNMODE_RUNNING, 
};

struct isx006_firmware {
	unsigned int addr;
	unsigned int size;
};

/* Camera functional setting values configured by user concept */
struct isx006_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int auto_wb;		/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;		/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int effect;		/* Color FX (AKA Color tone) */
	unsigned int contrast;		/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int iso;
	unsigned int brightness;
	unsigned int metering;
};

struct isx006_jpeg_param {
	unsigned int enable;
	unsigned int quality;
	unsigned int main_size;  /* Main JPEG file size */
	unsigned int thumb_size; /* Thumbnail file size */
	unsigned int main_offset;
	unsigned int thumb_offset;
	unsigned int postview_offset;
} ; 

struct isx006_position {
	int x;
	int y;
} ; 

struct exif_tm {
   int     tm_sec;         /* seconds */
   int     tm_min;         /* minutes */
   int     tm_hour;        /* hours */
   int     tm_mday;        /* day of the month */
   int     tm_mon;         /* month */
   int     tm_year;        /* year */
   int     tm_wday;        /* day of the week */
   int     tm_yday;        /* day in the year */
   int     tm_isdst;       /* daylight saving time */

   long int tm_gmtoff;     /* Seconds east of UTC.  */
   const char *tm_zone;    /* Timezone abbreviation.  */
};

struct gps_info_common {
	unsigned int 	direction;
	unsigned int 	dgree;
	unsigned int	minute;
	unsigned int	second;
};

struct isx006_gps_info{
	unsigned char gps_buf[8];
	unsigned char altitude_buf[4];
	long gps_timeStamp;
};

struct isx006_state_struct {
	struct isx006_userset userset;
	struct isx006_jpeg_param jpeg;
	struct isx006_version fw;
	struct isx006_version prm;
	struct isx006_date_info dateinfo;
	struct isx006_firmware fw_info;
	struct isx006_position position;
	struct isx006_gps_info gpsInfo;
	enum isx006_runmode runmode;
	enum isx006_oprmode oprmode;
	int framesize_index;
	int sensor_version;
	int freq;	/* MCLK in Hz */
	int fps;
	int preview_size;
	struct exif_tm *exifTimeInfo;
};

static struct isx006_state_struct isx006_state;

static int cam_hw_init(void);

#ifdef CONFIG_LOAD_FILE
	static int isx006_regs_table_write(struct i2c_client *client, char *name);
#endif

static int isx006_i2c_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { isx006_client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(isx006_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(isx006_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	//*data = ((buf[0] << 8) | buf[1]);
	/*  [Arun c]Data should be written in Little Endian in parallel mode; So there is no need for byte swapping here */
	*data = *(unsigned long *)(&buf);
error:
	return ret;
}
#if 0
static int isx006_i2c_read_multi(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[4];
	struct i2c_msg msg = { isx006_client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(isx006_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	msg.len = 4;
	
	ret = i2c_transfer(isx006_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	/*  [Arun c]Data should be written in Little Endian in parallel mode; So there is no need for byte swapping here */
	*data = *(unsigned long *)(&buf);

error:
	return ret;
}
#endif
static int isx006_i2c_write(unsigned short subaddr, unsigned short val,unsigned int len)
{
	int retry_count = 5;
	int ret = 0;
	unsigned char buf[4];
	struct i2c_msg msg = { isx006_client->addr, 0, len+2, buf };

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	/*  [Arun c]Data should be written in Little Endian in parallel mode; So there is no need for byte swapping here */
	if(len == 1)
		buf[2] = (unsigned char)val;
	else if(len == 2)
		*((unsigned short *)&buf[2]) = (unsigned short)val;
	else
		*((unsigned int *)&buf[2]) = val;

#ifdef ISX006_DEBUG
	//CDBG("[PGH] on write func isx006_client->addr : %x\n", isx006_client->addr);
	//CDBG("[PGH] on write func  isx006_client->adapter->nr : %d\n", isx006_client->adapter->nr);
	
	if(0)
	{
		int j;
		printk("W: ");
		for(j = 0; j <= len+1; j++){
			printk("0x%02x ", buf[j]);
		}
		printk("\n");
	}
#endif

	while(retry_count--){
		ret  = i2c_transfer(isx006_client->adapter, &msg, 1);
		if (likely(ret == 1))break;
		msleep(10);
		}
	if(ret !=1)printk("I2C is not working.\n");
	
	return (ret == 1) ? 0 : -EIO;
}

static int isx006_i2c_write_list(struct samsung_short_t *list, int size, char *name)
{
	int ret=0, i=0;

#ifdef CONFIG_LOAD_FILE	
	ret = isx006_regs_table_write(name);
#else
	for (i = 0; i < size; i++) {
		ret = isx006_i2c_write( list[i].subaddr, list[i].value, list[i].len);
		if (unlikely(ret < 0)) {
			printk("%s: register set failed\n",  __func__);
			return ret;
		}
	}
#endif
	return ret;
}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

static char *isx006_regs_table = NULL;

static int isx006_regs_table_size;

int isx006_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int i;
	int ret;
	mm_segment_t fs = get_fs();

	F_IN;
	
	set_fs(get_ds());
#if 0
	filp = filp_open("/data/camera/isx006.h", O_RDONLY, 0);
#else
	filp = filp_open("/mnt/internal_sd/external_sd/isx006.h", O_RDONLY, 0);
#endif

	if (IS_ERR(filp)) {
		printk( " file open error\n");
		return PTR_ERR(filp);
	}
	
	l = filp->f_path.dentry->d_inode->i_size;	
	CDBG( " l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
//	dp = vmalloc(l);	
	if (dp == NULL) {
		printk( "Out of Memory\n");
		filp_close(filp, current->files);
	}
	
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	
	if (ret != l) {
		printk( "Failed to read file ret = %d\n", ret);
//		kfree(dp);
		vfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}

	filp_close(filp, current->files);
		
	set_fs(fs);
	
	isx006_regs_table = dp;
		
	isx006_regs_table_size = l;
	
	*((isx006_regs_table + isx006_regs_table_size) - 1) = '\0';
	
	CDBG( "isx006_regs_table 0x%08x, %ld\n", dp, l);
	F_OUT;
	return 0;
}

void isx006_regs_table_exit(void)
{
	F_IN;
	if (isx006_regs_table) {
		kfree(isx006_regs_table);
		isx006_regs_table = NULL;
	}
	F_OUT;
}

static int isx006_regs_table_write(struct i2c_client *client, char *name)
{
	char *start, *end, *reg, *data;	
	unsigned short addr;
	unsigned int len, value;	
	char reg_buf[7], data_buf[7], len_buf[2];
	
	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';
	*(len_buf + 1) = '\0';	

	F_IN;
	start = strstr(isx006_regs_table, name);
	end = strstr(start, "};");
	
	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{0x");		
		if (reg)
			start = (reg + 19);  //{0x000b, 0x0004, 1},	
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 6);	
			memcpy(data_buf, (reg + 9), 6);	
			memcpy(len_buf, (reg + 17), 1);			
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned int)simple_strtoul(data_buf, NULL, 16); 
			len = (unsigned int)simple_strtoul(len_buf, NULL, 10); 			
//			printk("addr 0x%04x, value 0x%04x, len %d\n", addr, value, len);
			
			if (addr == 0xdddd)
			{
/*				if (value == 0x0010)
				cam_mdelay(10);
				else if (value == 0x0020)
				cam_mdelay(20);
				else if (value == 0x0030)
				cam_mdelay(30);
				else if (value == 0x0040)
				cam_mdelay(40);
				else if (value == 0x0050)
				cam_mdelay(50);
				else if (value == 0x0100)
				cam_mdelay(100);*/
				cam_mdelay(value);
				CDBG( " delay 0x%04x, value 0x%04x, , len 0x%01x\n", addr, value, len);
			}	
			else
				isx006_i2c_write( addr, value, len);
		}
	}
	F_OUT;
	return 0;
}

static short isx006_regs_max_value(char *name)
{
	char *start, *reg, *data;	
	unsigned short value;
	char data_buf[7];

	F_IN;
	
	*(data_buf + 6) = '\0';
	start = strstr(isx006_regs_table, name);
	
	/* Find Address */	
	reg = strstr(start," 0x");		
	if (reg == NULL)
		return 0;
	/* Write Value to Address */	
	if (reg != NULL) {
		memcpy(data_buf, (reg + 1), 6);	
		value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
	}
		
	F_OUT;
	
	return value;
}

#endif

static int isx006_wait_int( int interrupt)
{
	int err = 0;
	unsigned short read_value = 0;
	int timeout_cnt = 0;

	F_IN;
	
	do {
		timeout_cnt++;
		err = isx006_i2c_read(0x00F8, &read_value);
		msleep(2);
		if (timeout_cnt > 1500) {
			printk( "%s: Entering capture mode timed out\n", __func__);
			break;
		}
	}while(!(read_value&interrupt));

	timeout_cnt = 0;
	do {
		timeout_cnt++;
		err = isx006_i2c_write(0x00FC, 0x00FF, 1);
		msleep(2);
		err = isx006_i2c_read(0x00F8, &read_value);
		if (timeout_cnt > 1500) {
			printk( "%s: Entering capture mode timed out\n", __func__);
			break;
		}
	}while(read_value&interrupt);
	
	if(timeout_cnt > 1500)
		err = -ETIMEDOUT;
	else
		err = 0;
	
	F_OUT;
	
	return err;
}

void cam_pw(int status)
{
	struct vreg *vreg_cam_out8;
	struct vreg *vreg_cam_out9;
	struct vreg *vreg_cam_out10;

	vreg_cam_out8 = vreg_get(NULL, "ldo8"); // VDDIO 2.8v
	vreg_cam_out9 = vreg_get(NULL, "ldo9"); // VDDS 2.8v
	vreg_cam_out10 = vreg_get(NULL, "ldo10"); // AF 2.8v

	if(status == 1) //POWER ON
	{
		printk("[ISX006] Camera Sensor Power ON\n");
		vreg_set_level(vreg_cam_out10, OUT2800mV);
		vreg_set_level(vreg_cam_out9,  OUT2800mV);
		vreg_set_level(vreg_cam_out8,  OUT2800mV);

		gpio_set_value(3,1); //VDDD
		udelay(1);
		vreg_enable(vreg_cam_out8);//VDDIO
		udelay(1);
		vreg_enable(vreg_cam_out9);// VDDS
		udelay(1);
		vreg_enable(vreg_cam_out10);//AF
	}
	else //POWER OFF
	{
		printk("[ISX006] Camera Sensor Power OFF\n");
		vreg_disable(vreg_cam_out9);// VDDS
		udelay(1);
		vreg_disable(vreg_cam_out8); //VDDIO
		udelay(1);
		gpio_set_value(3,0); //VDDD
		udelay(1);
		vreg_disable(vreg_cam_out10); //AF
	}
}

static int cam_hw_init()
{

	int rc = 0;
//	unsigned short id = 0; //CAM FOR FW
//	unsigned int	before_time, after_time, i;//I2C SPEED TEST

	printk("[ISX006] Camera Sensor Power UP\n");

#if 0//BLOCK FOR PRE INIT
	CDBG("++++++++++++++++++++++++++test driver++++++++++++++++++++++++++++++++++++");
 
	gpio_set_value(0, 0);//RESET	
	gpio_set_value(1, 0);//STBY 

	cam_pw(1);

	gpio_set_value(1, 1); //STBY -> UP
#endif	

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	msm_camio_camif_pad_reg_reset();
	msleep(5);
	/* CAM RESET(GPIO 0) set to HIGH */
	gpio_set_value(0, 1); //CAM RESET - GPIO 0  
	msleep(15);
	/* CAM STANDBY(GPIO 37) set to HIGH */
	gpio_set_value(37, 1);

	gpio_direction_output(78, 1);
	gpio_direction_output(79, 1);

#if 0//PGH I2C SPEED TEST
        before_time = get_jiffies_64();
    for (i = 0; i < 3000; i++) 
        {
		isx006_sensor_write(0x002E, 0x0040);
        }       
 
        after_time = get_jiffies_64();
        printk("[ISX006] Total Time 3000: %d\n",  jiffies_to_msecs(after_time-before_time));
#endif//PGH I2C SPEED TEST

#if 0
	isx006_sensor_write(0x002C, 0x0000);
	isx006_sensor_write(0x002E, 0x0040);
	isx006_sensor_read(0x0F12, &id);

	if(id != 0x05CA)
	{
		printk("[ISX006] WRONG SENSOR FW => id 0x%x \n", id);
		rc = -1;
	}
#endif
	return rc;
}

void isx006_sensor_power_control(unsigned short onoff)
{
	struct vreg *vreg_cam_out8;
	struct vreg *vreg_cam_out9;
	struct vreg *vreg_cam_out10;

	vreg_cam_out8 = vreg_get(NULL, "ldo8"); // VDDIO 2.8v
	vreg_cam_out9 = vreg_get(NULL, "ldo9"); // VDDS 2.8v
	vreg_cam_out10 = vreg_get(NULL, "ldo10"); // AF 2.8v

	if(onoff == 1) //POWER ON
	{
		printk("[ISX006] Camera Sensor Power ON\n");
		vreg_set_level(vreg_cam_out10, OUT2800mV);
		vreg_set_level(vreg_cam_out9,  OUT2800mV);
		vreg_set_level(vreg_cam_out8,  OUT2800mV);

		/* VDDD on */
		gpio_set_value(3,1);
		msleep(1);
		/* VDDIO on */
		vreg_enable(vreg_cam_out8);
		msleep(1);
		/* VDDS on */
		vreg_enable(vreg_cam_out9);
		msleep(1);
		/* AF on */
		vreg_enable(vreg_cam_out10);
		/* Input MCLK = 24MHz */
		msm_camio_clk_rate_set(24000000);
		msm_camio_camif_pad_reg_reset();
		msleep(5);
		/* CAM RESET(GPIO 0) set to HIGH */
		gpio_set_value(0, 1);  
		msleep(15);
		/* CAM STANDBY(GPIO 37) set to HIGH */
		gpio_set_value(37, 1);
	}
	else //POWER OFF
	{
		/* CAM STANDBY(GPIO 37) set to HIGH */
		gpio_set_value(37, 1);
		msleep(15);
		/* CAM RESET(GPIO 0) set to HIGH */
		gpio_set_value(0, 1);  
		msleep(1);
		
		/* MLCK OFF in here */
		
		printk("[ISX006] Camera Sensor Power OFF\n");
		/* VDDS on */
		vreg_disable(vreg_cam_out9);
		msleep(1);
		/* VDDIO on */
		vreg_disable(vreg_cam_out8);
		msleep(1);
		/* VDDS on */
		gpio_set_value(3,0);
		udelay(1);
		/* AF on */
		vreg_disable(vreg_cam_out10);
	}
}

static int isx006_set_autofocus(unsigned short value)
{
	int err = 0;
	unsigned short read_value = 0;
	
	F_IN;
	//CDBG("%s -setting value:%d\n", __func__, value);

	switch(value) 
	{
		case CAMSENSOR_AF_CHECK_STATUS :
			CDBG("[ISX006] CAMSENSOR_AF_CHECK_STATUS");
			isx006_i2c_read(0x6D76, &read_value);
			CDBG("[ISX006] AF state : 0x%04x\n", read_value);
			if(read_value == 0x08)
			{
				/* Clear AF_LOCK_STS */
				err = isx006_i2c_write(0x00FC, 0x0010, 1);
				if (err < 0) {
					printk( "%s: register write fail \n", __func__);
					return -EIO;
				}
				/* Read AF result */
				err = isx006_i2c_read(0x6D77, &read_value);
				if (err < 0) {
					printk( "%s: register write fail \n", __func__);
					return -EIO;
				}
				CDBG("[ISX006] AF result : 0x%04x\n", read_value);
				if(read_value)return CAMSENSOR_AF_SUCCESS;
				else return CAMSENSOR_AF_FAILURE;
			}
			else
				return  CAMSENSOR_AF_PROGRESS;
		break;
		
		case CAMSENSOR_AF_OFF :
			CDBG("[ISX006] CAMSENSOR_AF_OFF");
			err = isx006_i2c_write_list( isx006_Single_AF_Off, ISX006_SINGLE_AF_OFF_REGS, "isx006_Single_AF_Off");
			if (err < 0) {
				printk( "%s: register write fail \n", __func__);
				return -EIO;
			}
		break;
			
		case CAMSENSOR_AF_SET_NORMAL :
			CDBG("[ISX006] CAMSENSOR_AF_SET_NORMAL");
			err = isx006_i2c_write_list(isx006_AF_Return_Inf_pos, ISX006_AF_RETURN_INF_POS_REGS, "isx006_AF_Return_Inf_pos");
			if (err < 0) {
				printk( "%s: register write fail \n", __func__);
				return -EIO;
			}
		break;
			
		case CAMSENSOR_AF_SET_MACRO :
			CDBG("[ISX006] CAMSENSOR_AF_SET_MACRO");
			err = isx006_i2c_write_list(isx006_AF_Return_Macro_pos, ISX006_AF_RETURN_MACRO_POS_REGS, "isx006_AF_Return_Macro_pos");
			if (err < 0) {
				printk( "%s: register write fail \n", __func__);
				return -EIO;
			}
		break;
			
		case CAMSENSOR_AF_DO :
			CDBG("[ISX006] CAMSENSOR_AF_DO");
			err = isx006_i2c_write_list( isx006_Single_AF_Start, ISX006_SINGLE_AF_START_REGS, "isx006_Single_AF_Start");
			if (err < 0) {
				printk( "%s: register write fail \n", __func__);
				return -EIO;
			}
		break;

		case CAMSENSOR_AF_CANCEL_F :
			CDBG("[ISX006] CAMSENSOR_AF_CANCEL_F");
			err = isx006_i2c_write(0x4885, 0x0001, 1);
			if (err < 0) {
				printk( "%s: register write fail \n", __func__);
				return -EIO;
			}
		break;
		
		default :
			printk("[ISX006] unexpected AF command : %d\n", value);
		break;
	}
	
	if(err < 0){
		printk( "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	
	F_OUT;
	return 0;
}

static int isx006_set_frame(unsigned short value)
{
	int err = 0;

	F_IN;
	CDBG("%s -setting value:%d\n", __func__, value);

	switch(value) 
	{
		case CAMSENSOR_FRAME_AUTO:
		default:
			//err = isx006_i2c_write_list( 
			//isx006_Effect_Normal, 
			//sizeof(isx006_Effect_Normal) / sizeof(isx006_Effect_Normal[0]), 
			//"isx006_Effect_Normal");
		break;
		
		case CAMSENSOR_FRAME_FIX_15:
			//err = isx006_i2c_write_list( 
			//isx006_Effect_Black_White, 
			//sizeof(isx006_Effect_Black_White) / sizeof(isx006_Effect_Black_White[0]), 
			//"isx006_Effect_Black_White");
		break;
	}

	if(err < 0){
		printk( "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	
	F_OUT;
	return 0;
}

static long isx006_config_effect(int mode, int effect)
{
	long rc = 0;

	switch (mode) {
		case SENSOR_PREVIEW_MODE:
			CDBG("SENSOR_PREVIEW_MODE\n");
		break;

		case SENSOR_SNAPSHOT_MODE:
			CDBG("SENSOR_SNAPSHOT_MODE\n");
		break;

		default:
			printk("[PGH] %s default\n", __func__);
		break;
	}

	switch (effect) {
		case CAMERA_EFFECT_OFF: 
			CDBG("CAMERA_EFFECT_OFF\n");
		break;

		case CAMERA_EFFECT_MONO: 
			CDBG("CAMERA_EFFECT_MONO\n");
		break;

		case CAMERA_EFFECT_NEGATIVE:
			CDBG("CAMERA_EFFECT_NEGATIVE\n");
		break;

		case CAMERA_EFFECT_SOLARIZE:
			CDBG("CAMERA_EFFECT_SOLARIZE\n");
		break;

		case CAMERA_EFFECT_SEPIA: 
			CDBG("CAMERA_EFFECT_SEPIA\n");
		break;

		default: 
			printk("[ISX006] unexpeceted effect  %s/%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	isx006_effect = effect;
	
	return rc;
}

static int isx006_set_effect(unsigned short value)
{
	int err = 0;

	F_IN;
	CDBG("%s -setting value:%d\n", __func__, value);

	switch(value) 
	{
		case CAMSENSOR_EFFECT_NORMAL:
		default:
			err = isx006_i2c_write_list( \
			isx006_Effect_Normal, \
			sizeof(isx006_Effect_Normal) / sizeof(isx006_Effect_Normal[0]), \
			"isx006_Effect_Normal");
		break;
		
		case CAMSENSOR_EFFECT_MONO:
			err = isx006_i2c_write_list( \
			isx006_Effect_Black_White, \
			sizeof(isx006_Effect_Black_White) / sizeof(isx006_Effect_Black_White[0]), \
			"isx006_Effect_Black_White");
		break;
		
		case CAMSENSOR_EFFECT_SEPIA:
			err = isx006_i2c_write_list( \
			isx006_Effect_Sepia, \
			sizeof(isx006_Effect_Sepia) / sizeof(isx006_Effect_Sepia[0]), \
			"isx006_Effect_Sepia");
		break;
		
		case CAMSENSOR_EFFECT_NEGATIVE:
			err = isx006_i2c_write_list( \
			isx006_Effect_Negative, \
			sizeof(isx006_Effect_Negative) / sizeof(isx006_Effect_Negative[0]), \
			"isx006_Effect_Negative");
		break;
	}

	if(err < 0){
		printk( "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	
	F_OUT;
	return 0;
}

static int isx006_set_white_balance(unsigned short value)
{
	int err = 0;
	
	F_IN;
	CDBG( "%s -setting value:%d\n", __func__, value);
	
	switch(value)
	{
		case CAMSENSOR_WB_AUTO:
			err = isx006_i2c_write_list( \
				isx006_WB_Auto, \
				sizeof(isx006_WB_Auto) / sizeof(isx006_WB_Auto[0]), \
				"isx006_WB_Auto");
		break;
		
		case CAMSENSOR_WB_DAYLIGHT:
			err = isx006_i2c_write_list( \
				isx006_WB_Sunny, \
				sizeof(isx006_WB_Sunny) / sizeof(isx006_WB_Sunny[0]), \
				"isx006_WB_Sunny");
		break;
		
		case CAMSENSOR_WB_CLOUDY:
			err = isx006_i2c_write_list( \
				isx006_WB_Cloudy, \
				sizeof(isx006_WB_Cloudy) / sizeof(isx006_WB_Cloudy[0]), \
				"isx006_WB_Cloudy");
		break;
		
		case CAMSENSOR_WB_INCANDESCENT:
			err = isx006_i2c_write_list( \
				isx006_WB_Tungsten, \
				sizeof(isx006_WB_Tungsten) / sizeof(isx006_WB_Tungsten[0]), \
				"isx006_WB_Tungsten");
		break;
		
		case CAMSENSOR_WB_FLUORESCENT:
			err = isx006_i2c_write_list( \
				isx006_WB_Fluorescent, \
				sizeof(isx006_WB_Fluorescent) / sizeof(isx006_WB_Fluorescent[0]), \
				"isx006_WB_Fluorescent");
		break;
		
		default:
			printk( "%s: failed: to set_white_balance, enum: %d\n", __func__, value);
			return -EINVAL;
		break;
	}

	if(err < 0){
		printk( "%s: failed: i2c_write for white_balance\n", __func__);
		return -EIO;
	}
	
	CDBG( "%s: done\n", __func__);
	F_OUT;
	
	return 0;
}

static int isx006_set_ev(unsigned short value)
{
	int err = 0;

	F_IN;

	switch(value)
	{
		case CAMSENSOR_BR_STEP_M_4:
			err = isx006_i2c_write_list( \
				isx006_EV_Minus_4, \
				sizeof(isx006_EV_Minus_4) / sizeof(isx006_EV_Minus_4[0]), \
				"isx006_EV_Minus_4");
		break;

		case CAMSENSOR_BR_STEP_M_3:
			err = isx006_i2c_write_list( \
				isx006_EV_Minus_3, \
				sizeof(isx006_EV_Minus_3) / sizeof(isx006_EV_Minus_3[0]), \
				"isx006_EV_Minus_3");
		break;

		case CAMSENSOR_BR_STEP_M_2:
			err = isx006_i2c_write_list( \
				isx006_EV_Minus_2, \
				sizeof(isx006_EV_Minus_2) / sizeof(isx006_EV_Minus_2[0]), \
				"isx006_EV_Minus_2");
		break;

		case CAMSENSOR_BR_STEP_M_1:
			err = isx006_i2c_write_list( \
				isx006_EV_Minus_1, \
				sizeof(isx006_EV_Minus_1) / sizeof(isx006_EV_Minus_1[0]), \
				"isx006_EV_Minus_1");
		break;

		case CAMSENSOR_BR_STEP_0:
			err = isx006_i2c_write_list( \
				isx006_EV_Default, \
				sizeof(isx006_EV_Default) / sizeof(isx006_EV_Default[0]), \
				"isx006_EV_Default");
		break;

		case CAMSENSOR_BR_STEP_P_1:
			err = isx006_i2c_write_list( \
				isx006_EV_Plus_1, \
				sizeof(isx006_EV_Plus_1) / sizeof(isx006_EV_Plus_1[0]), \
				"isx006_EV_Default");
		break;

		case CAMSENSOR_BR_STEP_P_2:
			err = isx006_i2c_write_list( \
				isx006_EV_Plus_2, \
				sizeof(isx006_EV_Plus_2) / sizeof(isx006_EV_Plus_2[0]), \
				"isx006_EV_Plus_2");
		break;
		
		case CAMSENSOR_BR_STEP_P_3:
			err = isx006_i2c_write_list( \
				isx006_EV_Plus_3, \
				sizeof(isx006_EV_Plus_3) / sizeof(isx006_EV_Plus_3[0]), \
				"isx006_EV_Plus_3");
		break;

		case CAMSENSOR_BR_STEP_P_4:
			err = isx006_i2c_write_list( \
				isx006_EV_Plus_4, \
				sizeof(isx006_EV_Plus_4) / sizeof(isx006_EV_Plus_4[0]), \
				"isx006_EV_Plus_4");
		break;			

		default:
			printk( "%s: failed: to set_ev, enum: %d\n", __func__, value);
			return -EINVAL;
		break;
	}

	//printk("isx006_set_ev: set_ev:, data: 0x%02x\n", isx006_buf_set_ev[1]);
	
		if(err < 0){
		printk( "%s: failed: i2c_write for set_ev\n", __func__);
		return -EIO;
	}
	F_OUT;

	return 0;
}

static int isx006_set_iso(unsigned short value)
{
	int err = 0;
	
	F_IN;
	
	switch(value)
	{
		case CAMSENSOR_ISO_50:
		case CAMSENSOR_ISO_AUTO:
		default:
			err = isx006_i2c_write_list( \
				isx006_ISO_Auto, \
				sizeof(isx006_ISO_Auto) / sizeof(isx006_ISO_Auto[0]), \
				"isx006_ISO_Auto");
		break;

		case CAMSENSOR_ISO_100:
			err = isx006_i2c_write_list( \
				isx006_ISO_100, \
				sizeof(isx006_ISO_100) / sizeof(isx006_ISO_100[0]), \
				"isx006_ISO_100");
		break;

		case CAMSENSOR_ISO_200:
			err = isx006_i2c_write_list( \
				isx006_ISO_200, \
				sizeof(isx006_ISO_200) / sizeof(isx006_ISO_200[0]), \
				"isx006_ISO_200");
		break;

		case CAMSENSOR_ISO_400:
			err = isx006_i2c_write_list( \
				isx006_ISO_400, \
				sizeof(isx006_ISO_400) / sizeof(isx006_ISO_400[0]), \
				"isx006_ISO_400");
		break;
	}
	
	if(err < 0){
		printk( "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	
	F_OUT;
	
	return 0;
}

static int isx006_set_metering(unsigned short value)
{
	int err = 0;
	
	F_IN;

	switch(value)
	{
		case CAMSENSOR_METERING_NORMAL:
			err = isx006_i2c_write_list( \
				isx006_Metering_Matrix, \
				sizeof(isx006_Metering_Matrix) / sizeof(isx006_Metering_Matrix[0]), \
				"isx006_Metering_Matrix");
		break;

		case CAMSENSOR_METERING_CENTER:
			err = isx006_i2c_write_list( \
				isx006_Metering_Center, \
				sizeof(isx006_Metering_Center) / sizeof(isx006_Metering_Center[0]), \
				"isx006_Metering_Center");
		break;

		case CAMSENSOR_METERING_SPOT:
			err = isx006_i2c_write_list( \
				isx006_Metering_Spot, \
				sizeof(isx006_Metering_Spot) / sizeof(isx006_Metering_Spot[0]), \
				"isx006_Metering_Spot");
		break;

		default:
			printk( "%s: failed: to set_photometry, enum: %d\n", __func__, value);
			return -EINVAL;
		break;
	}
	
	if(err < 0){
		printk( "%s: failed: i2c_write for set_photometry\n", __func__);
		return -EIO;
	}
	
	F_OUT;
	
	return 0;
}


static int isx006_set_scene_mode(unsigned short value)
{
	int err = 0;

	F_IN;
	CDBG( "%s: start   CurrentScene : %d, Scene : %d\n", __func__, gCurrentScene, value);

	switch(value)
	{
		case CAMSENSOR_SCENE_OFF:
			err = isx006_i2c_write_list( \
				isx006_Scene_Default, \
				sizeof(isx006_Scene_Default) / sizeof(isx006_Scene_Default[0]), \
				"isx006_Scene_Default");
		break;

		case CAMSENSOR_SCENE_PORTRAIT:
			err = isx006_i2c_write_list( \
				isx006_Scene_Portrait, \
				sizeof(isx006_Scene_Portrait) / sizeof(isx006_Scene_Portrait[0]), \
				"isx006_Scene_Portrait");
		break;

		case CAMSENSOR_SCENE_NIGHTSHOT:
			err = isx006_i2c_write_list( \
				isx006_Scene_Nightshot, \
				sizeof(isx006_Scene_Nightshot) / sizeof(isx006_Scene_Nightshot[0]), \
				"isx006_Scene_Nightshot");
		break;

		case CAMSENSOR_SCENE_BACKLIGHT:
			err = isx006_i2c_write_list( \
				isx006_Scene_Backlight, \
				sizeof(isx006_Scene_Backlight) / sizeof(isx006_Scene_Backlight[0]), \
				"isx006_Scene_Backlight");
		break;

		case CAMSENSOR_SCENE_LANDSCAPE:
			err = isx006_i2c_write_list( \
				isx006_Scene_Landscape, \
				sizeof(isx006_Scene_Landscape) / sizeof(isx006_Scene_Landscape[0]), \
				"isx006_Scene_Landscape");
		break;

		case CAMSENSOR_SCENE_SPORTS:
			err = isx006_i2c_write_list( \
				isx006_Scene_Sports, \
				sizeof(isx006_Scene_Sports) / sizeof(isx006_Scene_Sports[0]), \
				"isx006_Scene_Sports");
		break;

		case CAMSENSOR_SCENE_INDOOR:
			err = isx006_i2c_write_list( \
				isx006_Scene_Party_Indoor, \
				sizeof(isx006_Scene_Party_Indoor) / sizeof(isx006_Scene_Party_Indoor[0]), \
				"isx006_Scene_Party_Indoor");
		break;

		case CAMSENSOR_SCENE_SNOW:
			err = isx006_i2c_write_list( \
				isx006_Scene_Beach_Snow, \
				sizeof(isx006_Scene_Beach_Snow) / sizeof(isx006_Scene_Beach_Snow[0]), \
				"isx006_Scene_Beach_Snow");
		break;

		case CAMSENSOR_SCENE_SUNSET:
			err = isx006_i2c_write_list( \
				isx006_Scene_Sunset, \
				sizeof(isx006_Scene_Sunset) / sizeof(isx006_Scene_Sunset[0]), \
				"isx006_Scene_Sunset");
		break;

		case CAMSENSOR_SCENE_FALL:
			err = isx006_i2c_write_list( \
				isx006_Scene_Fall_Color, \
				sizeof(isx006_Scene_Fall_Color) / sizeof(isx006_Scene_Fall_Color[0]), \
				"isx006_Scene_Fall_Color");
		break;

		case CAMSENSOR_SCENE_FIREWORK:
			err = isx006_i2c_write_list( \
				isx006_Scene_Fireworks, \
				sizeof(isx006_Scene_Fireworks) / sizeof(isx006_Scene_Fireworks[0]), \
				"isx006_Scene_Fireworks");
		break;

		case CAMSENSOR_SCENE_TEXT:
			err = isx006_i2c_write_list( \
				isx006_Scene_Text, \
				sizeof(isx006_Scene_Text) / sizeof(isx006_Scene_Text[0]), \
				"isx006_Scene_Text");
		break;

		case CAMSENSOR_SCENE_CANDLE:
			err = isx006_i2c_write_list( \
				isx006_Scene_Candle_Light, \
				sizeof(isx006_Scene_Candle_Light) / sizeof(isx006_Scene_Candle_Light[0]), \
				"isx006_Scene_Candle_Light");
		break;
		
		default:
			printk( "%s: unsupported scene mode\n", __func__);
		break;
	}

	if(err < 0){
		printk( "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	
	gCurrentScene = value;

	F_OUT;
		
	return 0;
}

static int isx006_set_aeawb(unsigned short value)
{
	//int err = 0;
	//unsigned short read_value;	

	F_IN;
	CDBG( "%s: unlocking AE&AWB\n", __func__);	

	switch(value)
	{
			case CAMSENSOR_AWB_AE_LOCK :
				CDBG("CAMSENSOR_AWB_AE_LOCK");

			break;
				
			case CAMSENSOR_AWB_AE_UNLOCK :
				CDBG("CAMSENSOR_AWB_AE_UNLOCK");

			break;
				
			default :
				printk("[ISX006] Unexpected AWB_AE mode : %d\n",value);
			break;
	}
	
	F_OUT;
	
	return 0;
}

static int isx006_set_contrast(unsigned short value)
{
	int err = 0;
	
	F_IN;

	switch(value)
	{
		case CAMSENSOR_CR_STEP_M_2:
			err = isx006_i2c_write_list( \
				isx006_Contrast_Minus_2, \
				sizeof(isx006_Contrast_Minus_2) / sizeof(isx006_Contrast_Minus_2[0]), \
				"isx006_Contrast_Minus_2");	
		break;

		case CAMSENSOR_CR_STEP_M_1:
			err = isx006_i2c_write_list( \
				isx006_Contrast_Minus_1, \
				sizeof(isx006_Contrast_Minus_1) / sizeof(isx006_Contrast_Minus_1[0]), \
				"isx006_Contrast_Minus_1");	
		break;

		case CAMSENSOR_CR_STEP_0:
		default:
			err = isx006_i2c_write_list( \
				isx006_Contrast_Default, \
				sizeof(isx006_Contrast_Default) / sizeof(isx006_Contrast_Default[0]), \
				"isx006_Contrast_Default");	
		break;

		case CAMSENSOR_CR_STEP_P_1:
			err = isx006_i2c_write_list( \
				isx006_Contrast_Plus_1, \
				sizeof(isx006_Contrast_Plus_1) / sizeof(isx006_Contrast_Plus_1[0]), \
				"isx006_Contrast_Plus_1");	
		break;

		case CAMSENSOR_CR_STEP_P_2:
			err = isx006_i2c_write_list( \
				isx006_Contrast_Plus_2, \
				sizeof(isx006_Contrast_Plus_2) / sizeof(isx006_Contrast_Plus_2[0]), \
				"isx006_Contrast_Plus_2");	
		break;
	}

	if(err < 0){
		printk( "%s: failed: i2c_write for set_contrast\n", __func__);
		return -EIO;
	}
	
	CDBG( "%s: done, contrast: %d\n", __func__, value);
	F_OUT;

	return 0;
}

static int isx006_set_saturation(unsigned short value)
{
	int err = 0;

	F_IN;

	switch(value)
	{
		case CAMSENSOR_SA_STEP_M_2:
			err = isx006_i2c_write_list( \
				isx006_Saturation_Minus_2, \
				sizeof(isx006_Saturation_Minus_2) / sizeof(isx006_Saturation_Minus_2[0]), \
				"isx006_Saturation_Minus_2");	
		break;

		case CAMSENSOR_SA_STEP_M_1:
			err = isx006_i2c_write_list( \
				isx006_Saturation_Minus_1, \
				sizeof(isx006_Saturation_Minus_1) / sizeof(isx006_Saturation_Minus_1[0]), \
				"isx006_Saturation_Minus_1");	
		break;

		case CAMSENSOR_SA_STEP_0:
		default:
			err = isx006_i2c_write_list( \
				isx006_Saturation_Default, \
				sizeof(isx006_Saturation_Default) / sizeof(isx006_Saturation_Default[0]), \
				"isx006_Saturation_Default");	
		break;

		case CAMSENSOR_SA_STEP_P_1:
			err = isx006_i2c_write_list( \
				isx006_Saturation_Plus_1, \
				sizeof(isx006_Saturation_Plus_1) / sizeof(isx006_Saturation_Plus_1[0]), \
				"isx006_Saturation_Plus_1");	
		break;

		case CAMSENSOR_SA_STEP_P_2:
			err = isx006_i2c_write_list( \
				isx006_Saturation_Plus_2, \
				sizeof(isx006_Saturation_Plus_2) / sizeof(isx006_Saturation_Plus_2[0]), \
				"isx006_Saturation_Plus_2");	
		break;
	}

	if(err < 0){
		printk( "%s: failed: i2c_write for set_saturation\n", __func__);
		return -EIO;
	}
	
	CDBG( "%s: done, saturation: %d\n", __func__, value);
	F_OUT;

	return 0;
}

static int isx006_set_sharpness(unsigned short value)
{
	int err = 0;

	F_IN;

	switch(value)
	{
		case CAMSENSOR_SP_STEP_M_2:
			err = isx006_i2c_write_list( \
				isx006_Sharpness_Minus_2, \
				sizeof(isx006_Sharpness_Minus_2) / sizeof(isx006_Sharpness_Minus_2[0]), \
				"isx006_Sharpness_Minus_2");	
		break;

		case CAMSENSOR_SP_STEP_M_1:
			err = isx006_i2c_write_list( \
				isx006_Sharpness_Minus_1, \
				sizeof(isx006_Sharpness_Minus_1) / sizeof(isx006_Sharpness_Minus_1[0]), \
				"isx006_Sharpness_Minus_1");	
		break;

		case CAMSENSOR_SP_STEP_0:
		default:
			err = isx006_i2c_write_list( \
				isx006_Sharpness_Default, \
				sizeof(isx006_Sharpness_Default) / sizeof(isx006_Sharpness_Default[0]), \
				"isx006_Sharpness_Default");	
		break;

		case CAMSENSOR_SP_STEP_P_1:
			err = isx006_i2c_write_list( \
				isx006_Sharpness_Plus_1, \
				sizeof(isx006_Sharpness_Plus_1) / sizeof(isx006_Sharpness_Plus_1[0]), \
				"isx006_Sharpness_Plus_1");	
		break;

		case CAMSENSOR_SP_STEP_P_2:
			err = isx006_i2c_write_list( \
				isx006_Sharpness_Plus_2, \
				sizeof(isx006_Sharpness_Plus_2) / sizeof(isx006_Sharpness_Plus_2[0]), \
				"isx006_Sharpness_Plus_2");	
		break;
	}

	if(err < 0){
		printk( "%s: failed: i2c_write for set_saturation\n", __func__);
		return -EIO;
	}
	
	CDBG( "%s: done, sharpness: %d\n", __func__, value);
	F_OUT;

	return 0;
}

static int isx006_get_exif(ioctl_pcam_info_8bit* pCtrl_info)
{
	//I think this means exif info...
	/*
	unsigned short lsb, msb, rough_iso;
	isx006_sensor_write(0xFCFC, 0xD000);
	isx006_sensor_write(0x002C, 0x7000);
	isx006_sensor_write(0x002E, 0x23E8);
	isx006_sensor_read(0x0F12, &lsb); //0x23E8
	isx006_sensor_read(0x0F12, &msb); //0x23EA
	isx006_sensor_read(0x0F12, &rough_iso); //0x23EC
			
	ctrl_info.value_1 = lsb;
	ctrl_info.value_2 = msb;	
	//printk("[ISX006] exposure %x %x \n", lsb, msb);
	ctrl_info.value_3 = rough_iso;
	//printk("[ISX006] rough_iso %x \n", rough_iso);
	*/
	return 0;
}

static int isx006_set_preview_size(int index)
{
	int err = 0;
	
	F_IN;
	CDBG( "%s - index:%d\n", __func__, index);

	switch(index){
		case ISX006_PREVIEW_QCIF:
			err = isx006_i2c_write(0x0022, 0x00B0, 2);// HSIZE_MONI - 176
			err = isx006_i2c_write(0x0028, 0x0090, 2);// VSIZE_MONI - 144
		break;
		
		case ISX006_PREVIEW_D1:
			err = isx006_i2c_write(0x0022, 0x02D0, 2);// HSIZE_MONI - 720
			err = isx006_i2c_write(0x0028, 0x01E0, 2);// VSIZE_MONI - 480
		break;
		
		case ISX006_PREVIEW_VGA:
			err = isx006_i2c_write(0x0022, 0x0280, 2);// HSIZE_MONI - 640
			err = isx006_i2c_write(0x0028, 0x01E0, 2);// VSIZE_MONI - 480
		break;
		
		case ISX006_PREVIEW_SVGA:
			err = isx006_i2c_write(0x0022, 0x0320, 2);// HSIZE_MONI - 800
			err = isx006_i2c_write(0x0028, 0x0258, 2);// VSIZE_MONI - 600
		break;
		
		case ISX006_PREVIEW_WSVGA:
			err = isx006_i2c_write(0x0022, 0x0400, 2);// HSIZE_MONI - 1024
			err = isx006_i2c_write(0x0028, 0x0258, 2);// VSIZE_MONI - 600
		break;
		
		default:
		/* When running in image capture mode, the call comes here.
 		 * Set the default video resolution - ISX006_PREVIEW_VGA
 		 */ 
		CDBG( "Setting preview resoution as VGA for image capture mode\n");
		break;
	}
	
	err = isx006_i2c_write(0x0012, 0x0001, 1); //Moni_Refresh
	err = isx006_i2c_write(0x00FC, 0x0002, 1); //Clear Interrupt Status (CM_Change)
	msleep(5);
	
	F_OUT;

	return err;
}

static int isx006_set_preview_start(void)
{
	int err = 0;

	unsigned short read_value;

	/* Reset the AF check variables for the next sequence */
	stop_af_operation = 0;
	af_operation_status = 0;
	
	F_IN;

	err = isx006_i2c_write_list( isx006_cap_to_prev,\
							sizeof(isx006_cap_to_prev) / sizeof(isx006_cap_to_prev[0]),\
							"isx006_cap_to_prev");
	isx006_i2c_read(0x0004, &read_value);
	if((read_value & 0x03) != 0) {
		isx006_i2c_write(0x0011, 0x0000, 1);//MODE_SEL  0x00: Monitor mode
		/* Wait for Mode Transition (CM) */
		err = isx006_wait_int( ISX006_INT_CM_CHANGED);
		if(err)
		{
			printk("%s : %s : %s\n",  __func__, "Wait interrupt Error", "ISX006_INT_CM_CHANGED");
			return err;
		}
	}
		
	err = isx006_set_preview_size(ISX006_PREVIEW_VGA);
	if(err < 0){
		printk( "%s: failed: Could not set preview size\n", __func__);
		return -EIO;
	}
/*
	if(state->runmode != ISX006_RUNMODE_RUNNING)
	{
		ctrl.id = 0;
		ctrl.value = state->userset.iso;
		err = isx006_set_iso(sd, &ctrl);
		ctrl.value = state->userset.brightness ;	
		err = isx006_set_ev(sd, &ctrl);
		ctrl.value = state->userset.metering;
		err = isx006_set_metering(sd, &ctrl);
		ctrl.value = state->userset.manual_wb;
		err = isx006_set_white_balance(sd, &ctrl);
		ctrl.value = state->userset.effect;
		err = isx006_set_effect(sd, &ctrl);
	}
	state->runmode = ISX006_RUNMODE_RUNNING;
*/
	msleep(200);
	F_OUT;
	return 0;
}

static int isx006_set_capture_size(int index)
{
	int err = 0;

	F_IN;
	CDBG( "%s - index:%d\n", __func__, index);

	//FIXME : this is temporary patch...
	index = ISX006_CAPTURE_3MP;

	switch(index){
		case ISX006_CAPTURE_VGA: /* 640x480 */
			//err = isx006_i2c_write( 0x0386, 0x0140, 2);// HSIZE_TN - 320
			err = isx006_i2c_write( 0x0024, 0x0280, 2); //HSIZE_CAP 640
			err = isx006_i2c_write( 0x002A, 0x01E0, 2); //VSIZE_CAP 480
		break;
		
		case ISX006_CAPTURE_SVGA: /* 800x600 */
			//err = isx006_i2c_write( 0x0386, 0x0140, 2);// HSIZE_TN - 320
			err = isx006_i2c_write( 0x0024, 0x0320, 2); //HSIZE_CAP 800
			err = isx006_i2c_write( 0x002A, 0x0258, 2); //VSIZE_CAP 600
		break;
		
		case ISX006_CAPTURE_WVGA: /* 800x480 */
		//err = isx006_i2c_write( 0x0386, 0x0190, 2);// HSIZE_TN - 400
			err = isx006_i2c_write( 0x0024, 0x0320, 2); //HSIZE_CAP 800
			err = isx006_i2c_write( 0x002A, 0x01E0, 2); //VSIZE_CAP 480
		break;
		
		case ISX006_CAPTURE_WSVGA: /* 1024x600 */
		//err = isx006_i2c_write( 0x0386, 0x0190, 2);// HSIZE_TN - 400
			err = isx006_i2c_write( 0x0024, 0x0400, 2); //HSIZE_CAP 1024
			err = isx006_i2c_write( 0x002A, 0x0258, 2); //VSIZE_CAP 600
		break;
		
		case ISX006_CAPTURE_W1MP: /* 1600x960 */
		//err = isx006_i2c_write( 0x0386, 0x0190, 2);// HSIZE_TN - 400
			err = isx006_i2c_write( 0x0024, 0x0640, 2); //HSIZE_CAP 1600
			err = isx006_i2c_write( 0x002A, 0x03C0, 2); //VSIZE_CAP 960
		break;
		
		case ISX006_CAPTURE_2MP: /* 1600x1200 */
		//err = isx006_i2c_write( 0x0386, 0x0140, 2);// HSIZE_TN - 320
			err = isx006_i2c_write( 0x0024, 0x0640, 2); //HSIZE_CAP 1600
			err = isx006_i2c_write( 0x002A, 0x04B0, 2); //VSIZE_CAP 1200
		break;
		
		case ISX006_CAPTURE_W2MP: /* 2048x1232 */
		//err = isx006_i2c_write( 0x0386, 0x0190, 2);// HSIZE_TN - 400
			err = isx006_i2c_write( 0x0024, 0x0800, 2); //HSIZE_CAP 2048
			err = isx006_i2c_write( 0x002A, 0x04D0, 2); //VSIZE_CAP 1232
		break;
		
		case ISX006_CAPTURE_3MP: /* 2048x1536 */
		//err = isx006_i2c_write( 0x0386, 0x0140, 2);// HSIZE_TN - 320
			err = isx006_i2c_write( 0x0024, 0x0800, 2); //HSIZE_CAP 2048
			err = isx006_i2c_write( 0x002A, 0x0600, 2); //VSIZE_CAP 1536
		break;
		
		case ISX006_CAPTURE_W4MP: /* 2560x1536 */
		//err = isx006_i2c_write( 0x0386, 0x0190, 2);// HSIZE_TN -400
			err = isx006_i2c_write( 0x0024, 0x0A00, 2); //HSIZE_CAP 2560
			err = isx006_i2c_write( 0x002A, 0x0600, 2); //VSIZE_CAP 1536
		break;
		
		case ISX006_CAPTURE_5MP: /* 2560x1920 */
		//err = isx006_i2c_write( 0x0386, 0x0140, 2);// HSIZE_TN - 320
			err = isx006_i2c_write( 0x0024, 0x0A00, 2); //HSIZE_CAP 2560
			err = isx006_i2c_write( 0x002A, 0x0780, 2); //VSIZE_CAP 1920
		break;
		
		default:
		/* The framesize index was not set properly. 
 		 * Check s_fmt call - it must be for video mode. */
		return -EINVAL;
	}

	if(err < 0){
		printk("%s: failed: i2c_write for capture_resolution\n", __func__);
		return -EIO; 
	}

	F_OUT;
		
	return 0;
}

static int isx006_set_jpeg_quality(int index)
{
	int err = 0;
	
	F_IN;

	if(index < 0)
		index = 0;
	if(index > 100)
		index = 100;

	switch(index)
	{
		case 100: //Super fine
			err = isx006_i2c_write( 0x0204, 0x0002, 1);
			err = isx006_i2c_write( 0x0012, 0x0001, 1);
		break;

		case 70: // Fine
			err = isx006_i2c_write( 0x0204, 0x0001, 1);
			err = isx006_i2c_write( 0x0012, 0x0001, 1);
		break;

		case 40: // Normal
			err = isx006_i2c_write( 0x0204, 0x0000, 1);
			err = isx006_i2c_write( 0x0012, 0x0001, 1);
		break;

		default:
			err = isx006_i2c_write( 0x0204, 0x0002, 1);
			err = isx006_i2c_write( 0x0012, 0x0001, 1);
		break;
	}

	CDBG("%s -Quality:%d \n", __func__, index);

	if(err < 0){
		printk("%s: failed: i2c_write for jpeg_comp_level\n", __func__);
		return -EIO;
	}

	F_OUT;

	return 0;
}
#if 0
static int isx006_get_snapshot_data(void)
{
	int err = 0;
	unsigned short jpeg_framesize;
	
	F_IN;

	/* Get main JPEG size */
	err = isx006_i2c_read_multi(0x1624, &jpeg_framesize);

	if(err < 0){
		printk( "%s: failed: i2c_read for jpeg_framesize\n", __func__);
		return -EIO;
	}
	isx006_state.jpeg.main_size = jpeg_framesize;
	CDBG( "%s: JPEG main filesize = %d bytes\n", __func__, isx006_state.jpeg.main_size );

	isx006_state.jpeg.main_offset = 0;
	isx006_state.jpeg.thumb_offset = 0x271000;
	isx006_state.jpeg.postview_offset = 0x280A00;

	F_OUT;

	return 0;
}

static int isx006_get_LowLightCondition( int *Result)
{
	int err = 0;
	unsigned short read_value = 0;

	F_IN;

	err = isx006_i2c_read(0x0278, &read_value); // AGC_SCL_NOW
	if(err < 0){
		printk("%s: failed: i2c_read for low_light Condition\n", __func__);
		return -EIO; 
	}
	CDBG("%s: isx006_get_LowLightCondition : Read(0x%X) \n", __func__,read_value);
#ifdef CONFIG_LOAD_FILE
	unsigned short max_value = 0;
	max_value = isx006_regs_max_value("MAX_VALUE");
	printk( "%s   max_value = %x \n", __func__, max_value);
	if(read_value >= max_value)//if(read_value >= 0x0B33)
	{
		*Result = 1; //gLowLight
	}
#else
	if(read_value >= 0x0B33)//if(read_value >= 0x0B33)
	{
		*Result = 1; //gLowLight
	}
#endif

	F_OUT;

	return err;
}

static int isx006_LowLightCondition_Off(void)
{
	int err = 0;

	F_IN;

	err = isx006_i2c_write_list(isx006_Outdoor_Off, \
						sizeof(isx006_Outdoor_Off) / sizeof(isx006_Outdoor_Off[0]), \
						"isx006_Outdoor_Off");
	if(err < 0){
		printk("%s: failed: i2c_write for low_light Condition\n", __func__);
		return -EIO; 
	}

	if(gLowLight == 1)
	{
		gLowLight = 0;

		if(gCurrentScene == CAMSENSOR_SCENE_NIGHTSHOT)
		{
			CDBG( "%s: SCENE_MODE_NIGHTSHOT --- isx006_Night_Mode_Off: start \n", __func__);
			err = isx006_i2c_write_list(isx006_Night_Mode_Off, \
								sizeof(isx006_Night_Mode_Off) / sizeof(isx006_Night_Mode_Off[0]), \
								"isx006_Night_Mode_Off");	
		}
		else
		{
			CDBG( "%s: Not Night mode --- isx006_Low_Cap_Off: start \n", __func__);
			err = isx006_i2c_write_list(isx006_Low_Cap_Off, \
								sizeof(isx006_Low_Cap_Off) / sizeof(isx006_Low_Cap_Off[0]), \
								"isx006_Low_Cap_Off");			
		}
	}

	F_OUT;

	return err;
}
#endif

static int isx006_set_capture_start(void)
{
	int err = 0;
	//unsigned short read_value;

	F_IN;
	
	/* 1.capture number */
	
	isx006_i2c_write(0x00FC, 0x00FF, 1);	//interupt clear

	//err = isx006_write_regs(sd, isx006_prev_to_cap, sizeof(isx006_prev_to_cap) / sizeof(isx006_prev_to_cap[0]),"isx006_prev_to_cap");
#if 0	
	/* Outdoor setting */
	isx006_i2c_read(0x6C21, &read_value);
	CDBG("%s: OUTDOOR_F : 0x%x \n", __func__, read_value);
	
	if(read_value == 0x01)
	{
		//isx006_i2c_write(0x0014, 0x0003, 1);/*CAPNUM is setted 3. default value is 2. */
		err = isx006_i2c_write_list(\
			isx006_Outdoor_On, \
			sizeof(isx006_Outdoor_On) / sizeof(isx006_Outdoor_On[0]), \
			"isx006_Outdoor_On");
	}

	err = isx006_get_LowLightCondition(&gLowLight);

	if(gLowLight)
	{
		//isx006_i2c_write(0x0014, 0x0003, 1);/*CAPNUM is setted 3. default value is 2. */
		if(gCurrentScene == CAMSENSOR_SCENE_NIGHTSHOT)
		{
			err = isx006_i2c_write_list( \
				isx006_Night_Mode_On, \
				sizeof(isx006_Night_Mode_On) / sizeof(isx006_Night_Mode_On[0]), \
				"isx006_Night_Mode_On");
		}
		else
		{
			err =isx006_i2c_write_list( \
				isx006_Low_Cap_On, \
				sizeof(isx006_Low_Cap_On) / sizeof(isx006_Low_Cap_On[0]), \
				"isx006_Low_Cap_On");
		}
	}
	else
	{
		//isx006_i2c_write(0x0014, 0x0002, 1);/*CAPNUM is setted 2. default value is 2. */
	}
#endif
	/*CAPNUM is setted AUTO. */
	isx006_i2c_write(0x0014, 0x0000, 1);
	
	/* 2.Set image size */
	
	err = isx006_set_capture_size(isx006_state.framesize_index);
	if(err < 0){
		printk( "%s: failed: i2c_write for capture_resolution\n", __func__);
		return -EIO; 
	}

	isx006_i2c_write(0x0011, 0x0002, 1); //capture_command
	msleep(30);
	
	/* 3.Wait for Mode Transition (CM) */
	err = isx006_wait_int(ISX006_INT_CM_CHANGED);
	if(err)
	{
		printk( "%s : %s : %s\n",  __func__, "Wait interrupt Error", "ISX006_INT_CM_CHANGED");
		return err;
	}

	/* 4.Capture frame out....*/
	CDBG( "%s: Capture frame out. \n", __func__);
	msleep(50);

#if 0
	/* Wait for Mode Transition (CM) */
	err = isx006_wait_int(ISX006_INT_CAPNUM_END);
	if(err)
	{
		printk( "%s : %s : %s\n",  __func__, "Wait interrupt Error", "ISX006_INT_CAPNUM_END");
		return err;
	}

	isx006_i2c_read(0x0200, &read_value);
	CDBG("%s: JPEG STS: 0x%x \n", __func__, read_value);

	/*
	 * 8. Get JPEG Main Data
	 */ 
	err = isx006_get_snapshot_data();
	if(err < 0){
		printk( "%s: failed: get_snapshot_data\n", __func__);
		return err;
	}

	err = isx006_LowLightCondition_Off();
	if(err < 0){
		printk( "%s: failed: isx006_LowLightCondition_Off\n", __func__);
		return err;
	}
#endif
	F_OUT;
	
	return 0;
}

static long isx006_set_sensor_mode(int mode)
{
	CDBG("START");
	
	switch (mode) {
		case SENSOR_PREVIEW_MODE:
			CDBG("PREVIEW");
			isx006_set_preview_start();
		break;
			
		case SENSOR_SNAPSHOT_MODE:
			CDBG("SNAPSHOT");
			isx006_set_capture_start();
		break;
	
		case SENSOR_RAW_SNAPSHOT_MODE:
			//CDBG("RAW_SNAPSHOT NOT SUPPORT!");
			CDBG("RAW SNAPSHOT");
			isx006_set_capture_start();
		break;
	
		default:
			return -EINVAL;
	}

	return 0;
}

/* FIXME:see Config_proc_ctrlcmd.c */
void sensor_rough_control(void __user *arg)
{
	ioctl_pcam_info_8bit ctrl_info;

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("[ISX006] %s fail copy_from_user!\n", __func__);
	}

	CDBG("[ISX006] sensor_rough_control %d %d %d %d %d \n", ctrl_info.mode, ctrl_info.address,\
	 ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);

	switch(ctrl_info.mode)
	{
		case CAMSENSOR_AUTO_TUNNING:
		break;
			
		case CAMSENSOR_SDCARD_DETECT:
		break;
			
		case CAMSENSOR_GET_INFO:
			isx006_get_exif(&ctrl_info);
		break;
			
		case CAMSENSOR_FRAME_CONTROL:
			isx006_set_frame(ctrl_info.value_1);
		break;
			
		case CAMSENSOR_AF_CONTROL:
			ctrl_info.value_3 = isx006_set_autofocus(ctrl_info.value_1);
		break;
			
		case CAMSENSOR_EFFECT_CONTROL:
			isx006_set_effect(ctrl_info.value_1);
		break;
		
		case CAMSENSOR_WB_CONTROL:
			isx006_set_white_balance(ctrl_info.value_1);
		break;
		
		case CAMSENSOR_BR_CONTROL:
			isx006_set_ev(ctrl_info.value_1);
		break;
		
		case CAMSENSOR_ISO_CONTROL:
			isx006_set_iso(ctrl_info.value_1);
		break;
		
		case CAMSENSOR_METERING_CONTROL:
			isx006_set_metering(ctrl_info.value_1);
		break;
		
		case CAMSENSOR_SCENE_CONTROL:
			isx006_set_scene_mode(ctrl_info.value_1);
		break;
		
		case CAMSENSOR_AWB_AE_CONTROL:
			isx006_set_aeawb(ctrl_info.value_1);
		break;
			
		case CAMSENSOR_CR_CONTROL:
			isx006_set_contrast(ctrl_info.value_1);
		break;
			
		case CAMSENSOR_SA_CONTROL:
			isx006_set_saturation(ctrl_info.value_1);
		break;
		
		case CAMSENSOR_SP_CONTROL:
			isx006_set_sharpness(ctrl_info.value_1);
		break;

		case CAMSENSOR_JPEGQUALITY_CONTROL:
			isx006_set_jpeg_quality(ctrl_info.value_1);
		break;
		
		default :
			printk("[ISX006] Unexpected mode on sensor_rough_control!!!\n");
		break;
	}

	if(copy_to_user((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info)))
	{
		printk("[ISX006] %s fail on copy_to_user!\n", __func__);
	}
	
}

static int isx006_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	nightshot_set = 0; //re-init 
	rc = isx006_i2c_write_list( \
		isx006_init_reg, \
		sizeof(isx006_init_reg) / sizeof(isx006_init_reg[0]), \
		"isx006_init_reg");

	return rc;
}

int isx006_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	printk("[ISX006] isx006_sensor_init start.\n");
	
	isx006_ctrl = kzalloc(sizeof(struct isx006_ctrl), GFP_KERNEL);
	if (!isx006_ctrl) {
		CDBG("isx006_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		isx006_ctrl->sensordata = data;

	rc = cam_hw_init();
	if(rc < 0)
	{
		printk("[ISX006] cam_fw_init failed!\n");
		goto init_fail;
	}

#ifdef CONFIG_LOAD_FILE
	isx006_regs_table_init();
#endif	

	rc = isx006_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("isx006_sensor_init failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(isx006_ctrl);
	return rc;
}

static int isx006_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&isx006_wait_queue);
	return 0;
}

int isx006_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&isx006_sem); */

	CDBG("isx006_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = isx006_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = isx006_config_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
			rc = -EINVAL;
			break;
		}

	/* up(&isx006_sem); */

	return rc;
}

int isx006_sensor_release(void)
{
	int rc = 0;

	/* down(&isx006_sem); */
/*
	printk("[ISX006] lens moving to Base before CAM OFF\n");
	isx006_sensor_write(0x0028, 0x7000);
	isx006_sensor_write(0x002A, 0x0254);
	isx006_sensor_write(0x0F12, 0x0030); //Lens Pistion (0x00 ~ 0xfF) normally (0x30 ~ 0x80)
*/
	printk("[ISX006] isx006_sensor_release\n");
	kfree(isx006_ctrl);
	/* up(&isx006_sem); */

#ifdef CONFIG_LOAD_FILE
	isx006_regs_table_exit();
#endif

	return rc;
}

static int isx006_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	isx006_sensorw =
		kzalloc(sizeof(struct isx006_work), GFP_KERNEL);

	if (!isx006_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, isx006_sensorw);
	isx006_init_client(client);
	isx006_client = client;

	CDBG("isx006_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(isx006_sensorw);
	isx006_sensorw = NULL;
	CDBG("isx006_probe failed!\n");
	return rc;
}

static const struct i2c_device_id isx006_i2c_id[] = {
	{ "isx006", 0},
	{ },
};

static struct i2c_driver isx006_i2c_driver = {
	.id_table = isx006_i2c_id,
	.probe  = isx006_i2c_probe,
	.remove = __exit_p(isx006_i2c_remove),
	.driver = {
		.name = "isx006",
	},
};

static int isx006_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{

	int rc = i2c_add_driver(&isx006_i2c_driver);
	if (rc < 0){ // || isx006_client == NULL) {
		printk("[HCHYUN]isx006_sensor_probe rc : %d\n",rc);
		rc = -ENOTSUPP;
		goto probe_done;
	}

#if 0//CAMSENSOR
	rc = isx006_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;
#endif
	gpio_set_value(0, 0);//RESET
	gpio_set_value(37, 0);//STBY
	cam_pw(0);

	s->s_init = isx006_sensor_init;
	s->s_release = isx006_sensor_release;
	s->s_config  = isx006_sensor_config;

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __isx006_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, isx006_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __isx006_probe,
	.driver = {
		.name = "msm_camera_isx006",
		.owner = THIS_MODULE,
	},
};

static int __init isx006_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(isx006_init);
