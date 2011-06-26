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
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/device.h>
#include <linux/i2c/europa_tsp_gpio.h>

/* firmware - update */
#include <linux/firmware.h>
/* firmware - update */

/*change this value if new firmware be released*/
#define HEX_HW_VER	0x04 
#define HEX_SW_VER	0x04 

#define MAX_X	240//320 
#define MAX_Y	320//480 

#define CHECK_TIME (3*HZ)

#define MAX_KEYS	2
#define MAX_USING_FINGER_NUM 2

static int prev_wdog_val = -1;
static int i2c_error_cnt = 0;

static const int touchkey_keycodes[] = {
	KEY_MENU,
	KEY_BACK,
};

static int touchkey_status[MAX_KEYS];

#define TK_STATUS_PRESS		1
#define TK_STATUS_RELEASE		0

static struct workqueue_struct *synaptics_wq;

typedef struct
{
	uint8_t id;	/*!< (id>>8) + size */
	int8_t status;
	uint8_t z;	/*!< dn>0, up=0, none=-1 */
	uint16_t x;			/*!< X */
	uint16_t y;			/*!< Y */
} report_finger_info_t;

static report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM + 1]={0,};

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct work_struct  work;
	struct delayed_work work_check_ic;
	struct early_suspend early_suspend;
};

struct synaptics_ts_data *ts_global;
int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);

/* firmware - update */
static int firmware_ret_val = -1;
static int8_t HW_ver = -1;
static int8_t SW_ver = -1;
int firm_update( void );
extern int cypress_update( int );
extern int board_hw_revision;
/* firmware - update */

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

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

int tsp_reset( void )
{
	int ret = 1; 
	int key = 0;
	struct vreg *vreg_touch;
	printk("[TSP] %s+\n", __func__ );

	vreg_touch = vreg_get(NULL, "ldo6");

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

	if (ts_global->use_irq)
	{
		disable_irq(ts_global->client->irq);
	}

	ret = vreg_disable(vreg_touch);

	msleep(10);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		ret=-EIO;
		goto tsp_reset_out;
	}

	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );
	gpio_configure( TSP_SDA, GPIOF_DRIVE_OUTPUT );
	gpio_configure( TSP_INT, GPIOF_INPUT );

	msleep(10);

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

	msleep(200);

tsp_reset_out:
	if (ts_global->use_irq)
	{
		enable_irq(ts_global->client->irq);
	}
	printk("[TSP] %s-\n", __func__ );

	return ret;
}

static void process_key_event(uint8_t tsk_msg)
{
	int i;
	int keycode= 0;
	int st_old, st_new;

	//check each key status
	for(i = 0; i < MAX_KEYS; i++)
	{
		st_old = touchkey_status[i];
		st_new = (tsk_msg>>(i+6)) & 0x1;
		keycode = touchkey_keycodes[i];

		touchkey_status[i] = st_new;	// save status

		if(st_new > st_old)
		{
			// press event
			printk("[TSP] press keycode: %4d, keypress: %4d\n", keycode, 1);
			input_report_key(ts_global->input_dev, keycode, 1);
		}
		else if(st_old > st_new)
		{
			// release event
			printk("[TSP] release keycode: %4d, keypress: %4d\n", keycode, 0);
			input_report_key(ts_global->input_dev, keycode, 0);
		}
	}

}

void TSP_forced_release_forkey(void)
{
	int i;
	int temp_value=0;
	
	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
	{
		if(fingerInfo[i].id >=1)
		{
			fingerInfo[i].status = -2; // force release
		}

		if(fingerInfo[i].status != -2) continue;
		
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
		input_report_abs(ts_global->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts_global->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].z);
		input_mt_sync(ts_global->input_dev);

		printk("[TSP] force release\n");
		temp_value++;
	}

	if(temp_value>0)
		input_sync(ts_global->input_dev);

	
}
EXPORT_SYMBOL(TSP_forced_release_forkey);

#define ABS(a,b) (a>b?(a-b):(b-a))
static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret=0;
	uint8_t buf[12];// 02h ~ 0Dh
	uint8_t i2c_addr = 0x02;
	int i = 0;
	uint8_t finger = 0;

	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

	if (ret <= 0) {
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		goto work_func_out;
	}

	finger = buf[0] & 0x07;	

	fingerInfo[0].x = (buf[1] << 8) |buf[2];
	fingerInfo[0].y = (buf[3] << 8) |buf[4];
	fingerInfo[0].z = buf[5]/2;
	fingerInfo[0].id = (buf[6] >>4)& 0x0f;

	fingerInfo[1].x = (buf[7] << 8) |buf[8];
	fingerInfo[1].y = (buf[9] << 8) |buf[10];
	fingerInfo[1].z = buf[11]/2;
	fingerInfo[1].id = buf[6] & 0x0f;

	/* check key event*/
	if(fingerInfo[0].status != 1 && fingerInfo[1].status != 1)	
		process_key_event(buf[0]);

	/* check touch event */
	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
	{
		if(fingerInfo[i].id >=1) // press interrupt
		{
			if(i==0 && fingerInfo[1].status != 1)
			{
				if((fingerInfo[2].id != fingerInfo[0].id)&&(fingerInfo[2].id != 0))// no release with finger id change
				{
					//			if(fingerInfo[1].id ==0)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						printk("[TSP] [%d] 0 (%d, %d,	%x)\n", i, fingerInfo[2].x, fingerInfo[2].y, fingerInfo[2].z);
						fingerInfo[1].status = -1;
					}
				}
				else if(fingerInfo[2].id != 0) // check x or y jump with same finger id
				{

					if(ABS(fingerInfo[2].x,fingerInfo[0].x)>150)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						printk("[TSP] [%d] 0 (%d, %d,	%x)\n", i, fingerInfo[2].x, fingerInfo[2].y, fingerInfo[2].z);
						fingerInfo[1].status = -1;	
					}
					else if(ABS(fingerInfo[2].y,fingerInfo[0].y)>150)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						printk("[TSP] [%d] 0 (%d, %d,	%x)\n", i, fingerInfo[2].x, fingerInfo[2].y, fingerInfo[2].z);
						fingerInfo[1].status = -1;	
					}
					else // no jump
					{
						if(fingerInfo[i].status != -2) // force release
							fingerInfo[i].status = 1;
						else
							fingerInfo[i].status = -2;
					}
				}
				else // single touch with normal condition
				{
					if(fingerInfo[i].status != -2) // force release
						fingerInfo[i].status = 1;
					else
						fingerInfo[i].status = -2;
				}
			}
			else
			{
				if(fingerInfo[i].status != -2) // force release
					fingerInfo[i].status = 1;
				else
					fingerInfo[i].status = -2;
			}
		}
		else if(fingerInfo[i].id ==0) // release interrupt (only first finger)
		{
			if(fingerInfo[i].status == 1 || fingerInfo[i].status == -3) // prev status is press
				fingerInfo[i].status = 0;
			else if(fingerInfo[i].status == 0 || fingerInfo[i].status == -2) // release already or force release
				fingerInfo[i].status = -1;
		}

		if(fingerInfo[i].status < 0) continue;

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].status);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].z);
		input_mt_sync(ts->input_dev);

		printk("[TSP] [%d] %d (%d, %d,	%x)		%x\n", i, fingerInfo[i].id, fingerInfo[i].x, fingerInfo[i].y, fingerInfo[i].z, fingerInfo[i].status);		
	}

	input_sync(ts->input_dev);

	fingerInfo[2].x = fingerInfo[0].x;
	fingerInfo[2].y = fingerInfo[0].y;
	fingerInfo[2].z = fingerInfo[0].z;
	fingerInfo[2].id = fingerInfo[0].id;	

work_func_out:
	if (ts->use_irq)
	{
		enable_irq(ts->client->irq);
	}
}

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	rmsg.addr = ts_global->client->addr;
	rmsg.flags = 0;//I2C_M_WR;
	rmsg.len = 1;
	rmsg.buf = &start_reg;
	start_reg = reg;
	ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

	if(ret>=0) {
		rmsg.flags = I2C_M_RD;
		rmsg.len = buf_size;
		rmsg.buf = rbuf;
		ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1 );
	}

	if( ret < 0 )
	{
		printk("[TSP] Error code : %d\n", __LINE__ );
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

static void check_ic_work_func(struct work_struct *work)
{
	int ret=0;
	uint8_t i2c_addr = 0x1F;
	uint8_t wdog_val[1];
	
	ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));
	if (ret <= 0) {
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);

		i2c_error_cnt++;
	}else {
		i2c_error_cnt = 0;
	}

	if(wdog_val[0] == (uint8_t)prev_wdog_val || i2c_error_cnt >=3 )
	{
		tsp_reset();
		i2c_error_cnt=0;
		prev_wdog_val = -1;
	}
	else
	{
		prev_wdog_val = wdog_val[0];
	}

	schedule_delayed_work(&ts_global->work_check_ic, CHECK_TIME );
}

static int synaptics_ts_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	int key = 0;
	struct vreg *vreg_touch;
	uint8_t i2c_addr = 0x1B;
	uint8_t buf_tmp[3]={0,};

	HW_ver = 0;
	SW_ver = 0;

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	vreg_touch = vreg_get(NULL, "ldo6");
	ret = vreg_set_level(vreg_touch, OUT3000mV);
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

	msleep(700);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_DELAYED_WORK(&ts->work_check_ic, check_ic_work_func );
	schedule_delayed_work(&ts->work_check_ic, CHECK_TIME );

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

	printk(KERN_INFO "synaptics_ts_probe: max_x: 240, max_y: 320\n");
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);	//0, MAX_X, 0, 0
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);	//0, MAX_Y, 0, 0
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	for(key = 0; key < MAX_KEYS ; key++)
		input_set_capability(ts->input_dev, EV_KEY, touchkey_keycodes[key]);

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

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
	ret = tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));
	HW_ver = buf_tmp[1];
	SW_ver = buf_tmp[2];

	if (ret <= 0) {
		printk(KERN_ERR "i2c_transfer failed\n");
		ret = tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

		if (ret <= 0) 
		{
			printk("[TSP] %s, ln:%d, Failed to register TSP!!!\n\tcheck the i2c line!!!, ret=%d\n", __func__,__LINE__, ret);
			goto err_check_functionality_failed;
		}
	}
	/* Check point - i2c check - end */
	printk("[TSP] %s, ver CY=%x\n", __func__ , buf_tmp[0] );
	printk("[TSP] %s, ver HW=%x\n", __func__ , buf_tmp[1] );
	printk("[TSP] %s, ver SW=%x\n", __func__ , buf_tmp[2] );

	/* Check point - Firmware */
	printk("[TSP] %s:%d, ver SW=%x, HW=%x\n", __func__,__LINE__, SW_ver, HW_ver );

	if( HW_ver == HEX_HW_VER ){
		if((SW_ver<HEX_SW_VER)||((SW_ver&0xF0)==0xF0)||(SW_ver==0)){
			firm_update();
		}else{
			printk(KERN_INFO "[TSP] Firmware version is up-to-date");
		}	
	}
	else if( HW_ver == 0x03 ){
		if((SW_ver<0x03)||((SW_ver&0xF0)==0xF0)||(SW_ver==0)){	//HW03 SW ver 03 -fixed
			firm_update();
		}else{
			printk(KERN_INFO "[TSP] target HW version 03 - SW version is up-to-date\n");
			HW_ver = 0x04;
			firm_update();
		}
	}
	else if( HW_ver<HEX_HW_VER && HW_ver>0 ){
		if((SW_ver<0x02)||((SW_ver&0xF0)==0xF0)||(SW_ver==0)){	//HW02 SW ver 02 -fixed
			firm_update();
		}else{
			printk(KERN_INFO "[TSP] target HW version 02 - SW version is up-to-date\n");
		}
	}
	else{	
		printk("[TSP] HW version is Wrong! ln:%d\n", __LINE__);
	}

	if(board_hw_revision == 0x03){
		if( HW_ver == 0x02 ){
			printk(KERN_INFO "[TSP] board_rev=%d, TSP HW_ver=%d\n", board_hw_revision, HW_ver);
			HW_ver = 0x04;
			firm_update();
		}
	}

	printk(KERN_INFO "%s: Manufacturer ID: %x, HW ver=%x, SW ver=%x\n", __func__, buf_tmp[0], buf_tmp[1], buf_tmp[2]);


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

	cancel_delayed_work_sync(&ts->work_check_ic);
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

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
		enable_irq(client->irq);
	}

	cancel_delayed_work_sync(&ts->work_check_ic);

	ret = vreg_disable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	TSP_forced_release_forkey();

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	int key;
	struct vreg *vreg_touch;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	uint8_t i2c_addr = 0x1D;
	uint8_t buf[1];

	printk("[TSP] %s+\n", __func__ );

	gpio_set_value( TSP_SCL , 1 ); 
	gpio_set_value( TSP_SDA , 1 ); 
	gpio_set_value( TSP_INT , 1 ); 

	vreg_touch = vreg_get(NULL, "ldo6");

	ret = vreg_enable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	msleep(200);

	// for TSK
	for(key = 0; key < MAX_KEYS; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

	while (ts->use_irq)
	{
		msleep(20);

		ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));
		if (ret <= 0) {
			//			printk("[TSP] %d : i2c_transfer failed\n", __LINE__);
		}
		else if	( buf[0] == 0 )
		{
			continue;
		}
		else
		{
			printk("[TSP] %s:%d, ver SW=%x\n", __func__,__LINE__, buf[0] );
			enable_irq(client->irq);
			break;
		}
		msleep(20);
	}

	prev_wdog_val = -1;
	schedule_delayed_work(&ts->work_check_ic, CHECK_TIME );
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

// this funtion is garbage funtion for battery driver
int set_tsp_for_ta_detect(int state)
{
	return 1;
}
EXPORT_SYMBOL(set_tsp_for_ta_detect);

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

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i2c_addr = 0x1C;
	uint8_t buf_show[2] = {0};
	int phone_ver = 0;

	tsp_i2c_read( i2c_addr, buf_show, sizeof(buf_show));
	HW_ver = buf_show[0];
	SW_ver = buf_show[1];
	printk("[TSP] %s:%d, ver SW=%x, HW=%x\n", __func__,__LINE__, SW_ver, HW_ver );


	if ( HW_ver == 0x04 )
	{
		phone_ver = HEX_SW_VER*10;
	}
	else if ( HW_ver == 0x03 )
	{
		phone_ver = 30;  	
	}
	else if ( HW_ver == 0x02 )
	{
		phone_ver = 20;	
	}
	else
	{
		printk("[TSP] %s:%d,HW_ver is wrong!!\n", __func__,__LINE__ );
	}

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
	disable_irq(ts_global->client->irq);
	cancel_delayed_work_sync(&ts_global->work_check_ic);

	// TEST
	printk("[TSP] %s:%d, ver SW=%x, HW=%x\n", __func__,__LINE__, SW_ver, HW_ver );

	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );

	if ( HW_ver==0x04 )
	{
		firmware_ret_val = cypress_update ( 1 );
		printk("[TSP] %s:%d, ver SW=%x, HW=%x\n", __func__,__LINE__, SW_ver, HW_ver );
	}
	else if ( HW_ver==0x02 || HW_ver==0x03 )
	{
		firmware_ret_val = cypress_update( HW_ver );
		printk("[TSP] %s:%d, ver SW=%x, HW=%x\n", __func__,__LINE__, SW_ver, HW_ver );
	}
	else	
	{
		printk(KERN_INFO "[TSP] %s, %d cypress_update blocked, HW ver=%d\n", __func__, __LINE__, HW_ver);
		firmware_ret_val = cypress_update( 1 );	//temporary firmware update
	}

	msleep(1000);
	if( firmware_ret_val )
		printk(KERN_INFO "[TSP] %s success, %d\n", __func__, __LINE__);
	else	
		printk(KERN_INFO "[TSP] %s fail, %d\n", __func__, __LINE__);

	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );

	printk("[TSP] enable_irq : %d\n", __LINE__ );
	enable_irq(ts_global->client->irq);
	schedule_delayed_work(&ts_global->work_check_ic, CHECK_TIME );

	return 0;
} 
/* firmware - update */

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
