/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/device.h>
#include <linux/i2c/tsp_gpio.h>

#define DEBUG_INPUT_REPORT

/* firmware - update */
#include <linux/firmware.h>
/* firmware - update */

#define MAX_X  320	//240 
#define MAX_Y  240	//320 

static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	//	struct hrtimer timer;
	struct work_struct  work;
	//	struct work_struct  work_timer;
	struct early_suspend early_suspend;
};

struct synaptics_ts_data *ts_global;
int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);

/* firmware - update */
static int firmware_ret_val = -1;
static int HW_ver = -1;
static int SW_ver = -1;

int firm_update( void );
//CHJ extern int cypress_update( int );
/* firmware - update */

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

int read_ver(void);
static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(firmware	, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret	, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, firmware_ret_show, firmware_ret_store);
/* sys fs */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

#if 0
int tsp_reset( void )
{
	int ret=1;
#if 0	
	uint8_t i2c_addr = 0x07;
	uint8_t buf[1];
#endif
	struct vreg *vreg_touch;
	printk("[TSP] %s+\n", __func__ );

	vreg_touch = vreg_get(NULL, "ldo6");

	if (ts_global->use_irq)
	{
		disable_irq(ts_global->client->irq);
	}

	ret = vreg_disable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		ret=-EIO;
		goto tsp_reset_out;
	}

	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );
	gpio_configure( TSP_SDA, GPIOF_DRIVE_OUTPUT );
	gpio_configure( TSP_INT, GPIOF_INPUT );

#if 0
	gpio_tlmm_config(GPIO_CFG( TSP_SCL, 0, GPIO_OUTPUT, GPIO_PULL_UP,GPIO_2MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG( TSP_SDA, 0, GPIO_OUTPUT, GPIO_PULL_UP,GPIO_2MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG( TSP_INT, 0, GPIO_OUTPUT, GPIO_PULL_UP,GPIO_2MA), GPIO_ENABLE);
#endif

	gpio_set_value( TSP_SCL , 0 ); 
	gpio_set_value( TSP_SDA , 0 ); 
	gpio_set_value( TSP_INT , 0 ); 

	msleep( 5 );

	gpio_set_value( TSP_SCL , 1 ); 
	gpio_set_value( TSP_SDA , 1 ); 
	gpio_set_value( TSP_INT , 1 ); 

	ret = vreg_enable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		ret=-EIO;
		goto tsp_reset_out;
	}

	msleep(10);

tsp_reset_out:
	if (ts_global->use_irq)
	{
		enable_irq(ts_global->client->irq);
	}
	printk("[TSP] %s-\n", __func__ );

	return ret;
}
#endif

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	struct i2c_msg msg[2];

	uint8_t start_reg;
	uint8_t buf1[8];
	int     x_old, y_old;
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	x_old = y_old = 0;


	msg[0].addr = ts_global->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x10;
	ret = i2c_transfer(ts_global->client->adapter, &msg, 1);

	if(ret>=0)
	{
		msg[0].flags = I2C_M_RD;
		msg[0].len = sizeof(buf1);
		msg[0].buf = buf1;
		ret = i2c_transfer(ts_global->client->adapter, msg, 1);
	}	

	if (ret < 0) 
	{
		printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");
	} 
	else 
	{
		int x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8;
		int y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8;
		int z = buf1[5];
		int finger = buf1[0] & 0x01;
		int width = buf1[6];

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);	
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, finger);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width);

		//	input_report_abs(ts->input_dev, ABS_PRESSURE, z);//x
		//	input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);//x

		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);

		printk("[TSP] x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width);

		//	printk("[TSP] buf[0]=%X, buf[1]=%X, buf[2]=%X, buf[3]=%X, buf[4]=%X\n",
		//			 buf[0], buf[1], buf[2], buf[3], buf[4] );
		//	printk("[TSP] temp=%X, key_report=%X, prev_key=%X\n", temp, key_report, prev_key );


		//		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		//		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		//		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, z);
		//		input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
	}
	//	input_sync(ts->input_dev);
	enable_irq(ts->client->irq); 
}

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	rmsg.addr = ts_global->client->addr;
	rmsg.flags = 0;
	rmsg.len = 1;
	rmsg.buf = &start_reg;
	start_reg = reg;
	ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

	if(ret>=0){
		rmsg.flags = I2C_M_RD;
		rmsg.len = buf_size;
		rmsg.buf = rbuf;
		ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);
	}

	if(ret<0){
		printk("[TSP] Error code : %d\n",__LINE__);
	}
	return ret;
}



static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	struct vreg *vreg_touch;

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	vreg_touch = vreg_get(NULL, "ldo6");
	ret = vreg_set_level(vreg_touch, OUT2800mV);
	if (ret) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	ret = vreg_enable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	msleep(100);

	gpio_configure( VTOUCH_EN, GPIOF_OUTPUT_HIGH );	//I2C Pullup 2.6V 
	gpio_set_value( VTOUCH_EN, 1 );

	msleep(100);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	printk(KERN_INFO "synaptics_ts_probe: max_x: %d, max_y: %d\n", MAX_X, MAX_Y);

	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	printk("[TSP] %s, irq=%d\n", __func__, client->irq );

	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler,/* IRQF_TRIGGER_RISING |*/ IRQF_TRIGGER_FALLING , client->name, ts);
		if (ret == 0) 
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	/* sys fs */
	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);
	/* sys fs */

	/* Check point - i2c check - start */
	ret = read_ver();

	if (ret <= 0) {
		printk(KERN_ERR "i2c_transfer failed\n");
		ret = read_ver();

		if (ret <= 0) 
		{
			printk("[TSP] %s, ln:%d, Failed to register TSP!!!\n\tcheck the i2c line!!!, ret=%d\n", __func__,__LINE__, ret);
			goto err_check_functionality_failed;
		}
	}
	/* Check point - i2c check - end */

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	//	else
	//		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	struct vreg *vreg_touch;
	printk("[TSP] %s+\n", __func__ );

	vreg_touch = vreg_get(NULL, "ldo6");

	if (ts->use_irq)
	{
		disable_irq(client->irq);
	}

	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );
	gpio_configure( TSP_SDA, GPIOF_DRIVE_OUTPUT );
	gpio_configure( TSP_INT, GPIOF_INPUT );

	gpio_set_value( TSP_SCL , 0 ); 
	gpio_set_value( TSP_SDA , 0 ); 
	gpio_set_value( TSP_INT , 0 ); 

	gpio_set_value( VTOUCH_EN , 0 );	//I2C Pullup

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
		enable_irq(client->irq);
	}

	msleep(20);

	ret = vreg_disable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct vreg *vreg_touch;

	printk("[TSP] %s+\n", __func__ );

	gpio_set_value( TSP_SCL , 1 ); 
	gpio_set_value( TSP_SDA , 1 ); 
	gpio_set_value( TSP_INT , 1 ); 

	gpio_set_value( VTOUCH_EN , 1 );	//I2C Pullup

	vreg_touch = vreg_get(NULL, "ldo6");

	ret = vreg_enable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	msleep(150);

	enable_irq(client->irq);
	printk("[TSP] %s-\n", __func__ );
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "synaptics-rmi-ts", 1 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, synaptics_ts_id);

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= "synaptics-rmi-ts",
	},
};

static int __devinit synaptics_ts_init(void)
{
	printk("[TSP] %s\n", __func__ );
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq)
		return -ENOMEM;
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

int read_ver(void)
{
	int ret = 0;
	uint8_t i2c_addr = 0x1E;
	uint8_t buf_tmp[3]={0};

	ret = tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

	HW_ver = buf_tmp[0];
	SW_ver = buf_tmp[2];

	if (ret <= 0) {
		printk(KERN_ERR "i2c_transfer failed\n");
		ret = tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

		if (ret <= 0) 
		{
			printk("[TSP] %s, ln:%d, Failed to register TSP!!!\n\tcheck the i2c line!!!, ret=%d\n", __func__,__LINE__, ret);
			
		}
	}
	
	printk("[TSP] %s, ver HW=%x\n", __func__ , HW_ver );
	printk("[TSP] %s, ver SW=%x\n", __func__ , SW_ver );

	return ret;

}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int phone_ver = 0;

	printk("[TSP] %s ln=%d\n",__func__, __LINE__);

	phone_ver = 10; // SW ver.0 - change this value if new firmware be released
	read_ver();

	/* below protocol is defined with App. ( juhwan.jeong@samsung.com )
	   The TSP Driver report like XY as decimal.
	   The X is the Firmware version what phone has.
	   The Y is the Firmware version what TSP has. */

	sprintf(buf, "%d\n", phone_ver + SW_ver + (HW_ver*100));

	return sprintf(buf, "%s", buf );
}

/* firmware - update */
static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	firmware_ret_val = -1;

	if ( value == 1 )
	{
		printk("[TSP] Firmware update start!!\n" );

		firm_update( );
	}

	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk("[TSP] %s!\n", __func__);

	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	printk("[TSP] %s, operate nothing!\n", __func__);

	return size;
}


int firm_update( void )
{
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	printk("[TSP] disable_irq : %d\n", __LINE__ );

	firmware_ret_val = 1;
#if 0
	disable_irq(ts_global->client->irq);

	// TEST
	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );

	if( HW_ver==1 || HW_ver==2 )
		firmware_ret_val = cypress_update( HW_ver );
	else	
	{
		printk(KERN_INFO "[TSP] %s, %d cypress_update blocked, HW ver=%d\n", __func__, __LINE__, HW_ver);
		firmware_ret_val = 0; // Fail
	}

	if( firmware_ret_val )
		printk(KERN_INFO "[TSP] %s success, %d\n", __func__, __LINE__);
	else	
		printk(KERN_INFO "[TSP] %s fail, %d\n", __func__, __LINE__);

	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );

	printk("[TSP] enable_irq : %d\n", __LINE__ );
	enable_irq(ts_global->client->irq);
#endif

	return 0;
} 
/* firmware - update */

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
