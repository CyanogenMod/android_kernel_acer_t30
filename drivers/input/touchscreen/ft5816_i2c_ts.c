/*
 * drivers/input/touchscreen/ft5816_ts.c
 *
 * FocalTech Multi-chip solution (FT5301xN + Nuc102)
 *
 * Copyright (c) 2011  Focal tech Ltd.
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
 *
 *    note: only support mulititouch    Wenfs 2010-10-01
 *    Suppport Version: Android V3.2    Ben 2011-9-28
 */

//#define CONFIG_FTS_CUSTOME_ENV
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <linux/input/ft5816_i2c_ts.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/sysfs.h>
#include <linux/sysdev.h>

#include <linux/gpio.h>

#define FT5606_NEW_DRIVER
#define UPGRADE_AA_DELAY 50
#define UPGRADE_55_DELAY 10
#define UPGRADE_ID_1	0x79
#define UPGRADE_ID_2	0x06
#define UPGRADE_READID_DELAY 100

/* -------------- global variable definition -----------*/
static struct i2c_client *this_client;
static REPORT_FINGER_INFO_T _st_finger_infos[CFG_MAX_POINT_NUM];
//static unsigned int _sui_irq_num= IRQ_EINT(6);
static unsigned int _sui_irq_num = 0;
static int _si_touch_num = 0;

int tsp_keycodes[CFG_NUMOFKEYS] = {
	KEY_MENU,
	KEY_HOME,
	KEY_BACK,
	KEY_SEARCH
};

char *tsp_keyname[CFG_NUMOFKEYS] = {
	"Menu",
	"Home",
	"Back",
	"Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];

/***********************************************************************
    [function]:
                            callback:            read data from ctpm by i2c interface;
    [parameters]:
                            buffer[in]:          data buffer;
                            length[in]:          the length of the data buffer;
    [return]:
                            FTS_TRUE:            success;
                            FTS_FALSE:           fail;
************************************************************************/
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth)
{
	int ret;

	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if (ret <= 0) {
		printk("[TSP]i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/***********************************************************************
    [function]:
                       callback:             write data to ctpm by i2c interface;
    [parameters]:
                       buffer[in]:          data buffer;
                       length[in]:          the length of the data buffer;
    [return]:
                       FTS_TRUE:            success;
                       FTS_FALSE:           fail;
************************************************************************/
static bool i2c_write_interface(u8* pbt_buf, int dw_lenth)
{
	int ret;
	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret <= 0) {
		printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}



/***********************************************************************
    [function]:
                        callback:             read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to read;
                        rx_buf[in]:           data buffer which is used to store register value;
                        rx_length[in]:        the length of the data buffer;
    [return]:
                        FTS_TRUE:             success;
                        FTS_FALSE:            fail;
************************************************************************/
static bool fts_register_read(u8 reg_name, u8* rx_buf, int rx_length)
{
	u8 read_cmd[2] = {0};
	u8 cmd_len  = 0;

	read_cmd[0] = reg_name;
	cmd_len = 1;

	/*send register addr*/
	if (!i2c_write_interface(&read_cmd[0], cmd_len)) {
		return FTS_FALSE;
	}

	/*call the read callback function to get the register value*/
	if (!i2c_read_interface(rx_buf, rx_length)) {
		return FTS_FALSE;
	}
	return FTS_TRUE;
}

/***********************************************************************
    [function]:
                        callback:             read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to write;
                        tx_buf[in]:           buffer which is contained of the writing value;
    [return]:
                        FTS_TRUE:             success;
                        FTS_FALSE:            fail;
************************************************************************/
static bool fts_register_write(u8 reg_name, u8* tx_buf)
{
	u8 write_cmd[2] = {0};

	write_cmd[0] = reg_name;
	write_cmd[1] = *tx_buf;

	/*call the write callback function*/
	return i2c_write_interface(write_cmd, 2);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************
    [function]:
                        callback:                early suspend function interface;
    [parameters]:
                        handler:                 early suspend callback pointer
    [return]:
                        NULL
************************************************************************/
static void fts_ts_suspend(struct early_suspend *handler)
{
	u8 cmd;

	cmd = PMODE_HIBERNATE;
	printk("\n [TSP]:device will suspend! \n");
	fts_register_write(FT5816_REG_PMODE, &cmd);
}


/***********************************************************************
    [function]:
                        callback:                power resume function interface;
    [parameters]:
                        handler:                 early suspend callback pointer
    [return]:
                        NULL
************************************************************************/
static void fts_ts_resume(struct early_suspend *handler)
{
	u8 cmd;

	gpio_set_value(TS_RESET, 0);
	msleep(10);
	gpio_set_value(TS_RESET, 1);
	msleep(40);

	cmd = PMODE_ACTIVE;
	printk("\n [TSP]:device will resume from sleep! \n");
	fts_register_write(FT5816_REG_PMODE, &cmd);
}
#endif

/***********************************************************************
    [function]:
                   callback:        report to the input system that the finger is put up;
    [parameters]:
                         null;
    [return]:
                         null;
************************************************************************/
static void fts_ts_release(REPORT_FINGER_INFO_T *finger)
{
	int i;
	struct FTS_TS_DATA_T *data = i2c_get_clientdata(this_client);

	for (i = 0; i < CFG_MAX_POINT_NUM; i++) {
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, finger[i].i2_x);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, finger[i].i2_y);
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, finger[i].u2_pressure);
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, finger[i].ui2_id);
		input_mt_sync(data->input_dev);
#if 0
		if (_st_finger_infos[i].u2_pressure == 0)
			_st_finger_infos[i].u2_pressure= -1;
#endif
	}

	input_sync(data->input_dev);
	_si_touch_num = 0;
}

/***********************************************************************
    [function]:
                            callback:              read touch  data ftom ctpm by i2c interface;
    [parameters]:
                rxdata[in]:            data buffer which is used to store touch data;
                length[in]:            the length of the data buffer;
    [return]:
                FTS_TRUE:              success;
                FTS_FALSE:             fail;
************************************************************************/
static int fts_i2c_rxdata(u8 *rxdata, int length)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = this_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = rxdata;
	ret = i2c_transfer(this_client->adapter, &msg, 1);

	if (ret < 0) {
		pr_err("msg %s i2c write error: %d\n", __func__, ret);
	}
	msg.addr = this_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = rxdata;
	ret = i2c_transfer(this_client->adapter, &msg, 1);

	if (ret < 0) {
		pr_err("msg %s i2c write error: %d\n", __func__, ret);
	}
	return ret;
}

/***********************************************************************
    [function]:
                            callback:             send data to ctpm by i2c interface;
    [parameters]:
                           txdata[in]:            data buffer which is used to send data;
                           length[in]:            the length of the data buffer;
    [return]:
                           FTS_TRUE:              success;
                           FTS_FALSE:             fail;
************************************************************************/
static int fts_i2c_txdata(u8 *txdata, int length)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = this_client->addr;
	msg.flags = 0;
	msg.len = length;
	msg.buf = txdata;
	ret = i2c_transfer(this_client->adapter, &msg, 1);
	if (ret < 0) {
		pr_err("%s i2c write error: %d\n", __func__, ret);
	}
	return ret;
}

/***********************************************************************
    [function]:
                         callback:            gather the finger information and calculate the X,Y
                                      coordinate then report them to the input system;
    [parameters]:
                         null;
    [return]:
                         null;
************************************************************************/
int fts_read_data(void)
{
	struct FTS_TS_DATA_T *data = i2c_get_clientdata(this_client);
	REPORT_FINGER_INFO_T _st_finger_infos[CFG_MAX_POINT_NUM] = {0};
	//struct FTS_TS_EVENT_T *event = &data->event;
	u8 buf[ 3 + CFG_MAX_POINT_NUM * 6 + 1] = {0};

	int i, id, temp, i_count, ret = -1;
	int touch_num = 0, touch_event, pressure; //x, y, size;
	int num;

	i_count = 0;
	disable_irq(_sui_irq_num);

	_si_touch_num = 0;
	ret = fts_i2c_rxdata(buf, 3 + CFG_MAX_POINT_NUM * 6);
	buf[ 3 + CFG_MAX_POINT_NUM * 6] = '\0';

	if (ret > 0) {
		touch_num = buf[FT5816_REG_TD_STATUS] & 0x0F;
		for (num = 0; num < touch_num; num++) {
			id = buf[FT5816_FIRST_ID_ADDR + FT5816_POINTER_INTERVAL * num] >> 4;

			if (id >= 0 && id < CFG_MAX_POINT_NUM) {
				touch_event = buf[3 + FT5816_POINTER_INTERVAL * num] >> 6;
				temp = buf[3 + FT5816_POINTER_INTERVAL * num] & 0x0f;
				temp = temp << 8;
				temp = temp | buf[4 + FT5816_POINTER_INTERVAL * num];
				_st_finger_infos[num].i2_x = temp;

				temp = (buf[5 + FT5816_POINTER_INTERVAL * num]) & 0x0f;
				temp = temp << 8;
				temp = temp | buf[6 + FT5816_POINTER_INTERVAL * num];
				_st_finger_infos[num].i2_y = temp;

				if (touch_event == 0 || touch_event == 2 || touch_event == 3) {
					pressure = 32;
					//width = 2;
					_st_finger_infos[num].u2_pressure = pressure;
					//printk("Finger[%d] event: %d, x: %d, y: %d, pressure: %d\n", num, touch_event, _st_finger_infos[num].i2_x,_st_finger_infos[num].i2_y, _st_finger_infos[num].u2_pressure);
					_st_finger_infos[num].ui2_id = id;
					_si_touch_num++;
				} else if (touch_event == 1) {
					_st_finger_infos[num].ui2_id = id;
					_st_finger_infos[num].u2_pressure = 0;
					//printk("Finger[%d] event: %d, x: %d, y: %d, pressure: %d\n", num, touch_event, _st_finger_infos[num].i2_x,_st_finger_infos[num].i2_y, _st_finger_infos[num].u2_pressure);
					//width = 0;
					_si_touch_num++;
				}

				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, _st_finger_infos[num].ui2_id);
				input_report_abs(data->input_dev, ABS_MT_PRESSURE, _st_finger_infos[num].u2_pressure);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, _st_finger_infos[num].u2_pressure/2);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, _st_finger_infos[num].u2_pressure/2);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[num].i2_x);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, _st_finger_infos[num].i2_y);

				input_mt_sync(data->input_dev);
			} else {
				break;
			}
		}
		// Add For FTM
		//	input_report_key(data->input_dev, BTN_TOUCH, _si_touch_num > 0);

		input_sync(data->input_dev);
	}

ENABLE_IRQ:
	if (_si_touch_num == 0) {
		for (i = 0; i < CFG_MAX_POINT_NUM; i++) {
			_st_finger_infos[i].u2_pressure= 0;
			_st_finger_infos[i].ui2_id  = 0;
		}
		fts_ts_release(_st_finger_infos);
	}

	enable_irq(_sui_irq_num);
	return 0;
}

static void fts_work_func(struct work_struct *work)
{
	fts_read_data();
}

static irqreturn_t fts_ts_irq(int irq, void *dev_id)
{
	struct FTS_TS_DATA_T *ft5816_ts = dev_id;
	if (!work_pending(&ft5816_ts->pen_event_work)) {
		queue_work(ft5816_ts->ts_workqueue, &ft5816_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

/***********************************************************************
[function]:
                      callback:         send a command to ctpm.
[parameters]:
                      btcmd[in]:       command code;
                      btPara1[in]:     parameter 1;
                      btPara2[in]:     parameter 2;
                      btPara3[in]:     parameter 3;
                      num[in]:         the valid input parameter numbers,
                                           if only command code needed and no
                                           parameters followed,then the num is 1;
[return]:
                      FTS_TRUE:      success;
                      FTS_FALSE:     io fail;
************************************************************************/
static bool cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	u8 write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(write_cmd, num);
}

/***********************************************************************
[function]:
                      callback:      write a byte data  to ctpm;
[parameters]:
                      buffer[in]:    write buffer;
                      length[in]:    the size of write data;
[return]:
                      FTS_TRUE:      success;
                      FTS_FALSE:     io fail;
************************************************************************/
static bool byte_write(u8* buffer, int length)
{
	return i2c_write_interface(buffer, length);
}

/***********************************************************************
[function]:
                      callback:         read a byte data  from ctpm;
[parameters]:
                      buffer[in]:      read buffer;
                      length[in]:      the size of read data;
[return]:
                      FTS_TRUE:      success;
                      FTS_FALSE:     io fail;
************************************************************************/
static bool byte_read(u8* buffer, int length)
{
	return i2c_read_interface(buffer, length);
}

/***********************************************************************
              SYSDEV FS
************************************************************************/

static ssize_t touch_ftmping_show(struct sys_device *dev,
        struct sysdev_attribute *attr, char *buf)
{
	unsigned char chip_id = 0;
	fts_register_read(FT5816_REG_CIPHER, &chip_id, 1);
	return sprintf(buf, "%d", chip_id);
}

static SYSDEV_ATTR(ftmping, 0644, touch_ftmping_show, NULL);

static ssize_t touch_ftmgetversion_show(struct sys_device *dev,
	struct sysdev_attribute *attr, char *buf)
{
	unsigned char reg_version = 0;
	fts_register_read(FT5816_REG_FIRMID, &reg_version, 1);
	return sprintf(buf, "%d", reg_version);
}

static SYSDEV_ATTR(ftmgetversion, 0644, touch_ftmgetversion_show, NULL);

static struct sysdev_class touch_sysclass = {
	.name	= "touch",
};

static struct sys_device device_touch = {
	.id	= 0,
	.cls	= &touch_sysclass,
};

#define FTS_PACKET_LENGTH 200

static unsigned char CTPM_FW[]=
{
#include "ft_app.i"
};
void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++) {
			udelay(1);
		}
	}
}

/***********************************************************************************************
Name :

Input :

Output :

function :

***********************************************************************************************/
static int ft5816_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr   = this_client->addr,
			.flags  = 0,
			.len    = length,
			.buf    = txdata,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

/***********************************************************************************************
Name : ft5816_write_reg

Input : addr -- address
        para -- parameter

Output :

function : write register of ft5816

***********************************************************************************************/
static int ft5816_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5816_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static int ft5816_read_reg(uint8_t addr, unsigned char *rdata)
{
	uint8_t wdata[1] = {0};

	wdata[0] = addr;
	if (1 != i2c_master_send(this_client, wdata, 1))
		pr_info("%s: i2c send err\n", __func__);
	if (1 != i2c_master_recv(this_client, rdata, 1))
		pr_info("%s: i2c recv err\n", __func__);

	return 0;
}

/***********************************************************************
[function]:
	callback:     burn the FW to ctpm.
[parameters]:
	pbt_buf[in]:  point to Head+FW ;
	dw_lenth[in]: the length of the FW + 6(the Head length);
[return]:
	ERR_OK:       no error;
	ERR_MODE:     fail to switch to UPDATE mode;
	ERR_READID:   read id fail;
	ERR_ERASE:    erase chip fail;
	ERR_STATUS:   status error;
	ERR_ECC:      ecc error.
************************************************************************/

#if 0
E_UPGRADE_ERR_TYPE fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	FTS_BYTE reg_val[2] = {0};
	FTS_DWRD i = 0;

	FTS_DWRD packet_number;
	FTS_DWRD j;
	FTS_DWRD temp;
	FTS_DWRD lenght;
	FTS_BYTE packet_buf[FTS_PACKET_LENGTH + 6];
	FTS_BYTE auc_i2c_write_buf[10];
	FTS_BYTE bt_ecc;
	int      i_ret;
	int      error = 0;
	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	ft5816_write_reg(0xfc, 0xaa);
	delay_qt_ms(50);
	/*write 0x55 to register 0xfc*/
	ft5816_write_reg(0xfc, 0x55);
	printk("[TSP] Step 1: Reset CTPM test\n");

	delay_qt_ms(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i ++;
		i_ret = ft5816_i2c_txdata(auc_i2c_write_buf, 2);
		delay_qt_ms(5);
	} while (i_ret <= 0 && i < 5 );
	printk("[TSP] chris******************, function (%s)---Line(%d), -- step2\n",  __FUNCTION__,__LINE__);

	/*********Step 3:check READ-ID***********************/
	error=cmd_write(0x90, 0x00, 0x00, 0x00, 4);
	if (error == 0) {
		printk("[TSP] chris****************** cmd_write error, ---Line(%d), -- step3\n", __LINE__);
	}

	error = byte_read(reg_val, 2);

	if(error == 0) {
		printk("[TSP] chris****************** byte_read error, ---Line(%d), -- step3\n", __LINE__);
	}

	if (reg_val[0] == 0x79 && reg_val[1] == 0x6) { //orig
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else {
		printk("[TSP] Step 3 error: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
		return ERR_READID;
		//i_is_new_protocol = 1;
	}
	printk("[TSP] chris******************, function (%s)---Line(%d), -- step3\n",  __FUNCTION__,__LINE__);

	/*********Step 4:erase app*******************************/
	cmd_write(0x61, 0x00, 0x00, 0x00, 1);

	delay_qt_ms(1500);

	cmd_write(0x63, 0x00, 0x00, 0x00, 1);
	printk("[TSP] Step 4: erase. \n");
	printk("[TSP] chris******************, function (%s)---Line(%d), -- step4\n",  __FUNCTION__,__LINE__);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[TSP] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(lenght>>8);
		packet_buf[5] = (FTS_BYTE)lenght;

		for (i = 0;i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0) {
			printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);
		delay_qt_ms(20);
	}

	//send the last six byte
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp >> 8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp = 1;
		packet_buf[4] = (FTS_BYTE)(temp >> 8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i];
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0], 7);
		delay_qt_ms(20);
	}
	printk("[TSP] chris******************, function (%s)---Line(%d), -- step5\n",  __FUNCTION__,__LINE__);

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	cmd_write(0xcc, 0x00, 0x00, 0x00, 1);
	byte_read(reg_val, 1);
	printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc) {
		return ERR_ECC;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07, 0x00, 0x00, 0x00, 1);
	printk("[TSP] chris******************, function (%s)---Line(%d), -- step6\n",  __FUNCTION__,__LINE__);

	return ERR_OK;
}
#endif

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp;
	unsigned char i ;

	printk("[FTS] start auto CLB.\n");
	msleep(200);
	ft5816_write_reg(0, 0x40);
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5816_write_reg(2, 0x4);  //write command to start calibration
	delay_qt_ms(300);
	for(i = 0; i < 100; i++) {
		ft5816_read_reg(0,&uc_temp);
		if (((uc_temp&0x70) >> 4) == 0x0) { //return to normal mode, calibration finish
			break;
		}
		delay_qt_ms(200);
		printk("[FTS] waiting calibration %d\n",i);
	}
	printk("[FTS] calibration OK.\n");

	msleep(300);
	ft5816_write_reg(0, 0x40);  //goto factory mode
	delay_qt_ms(100);           //make sure already enter factory mode
	ft5816_write_reg(2, 0x5);   //store CLB result
	delay_qt_ms(300);
	ft5816_write_reg(0, 0x0);   //return to normal mode
	msleep(300);
	printk("[FTS] store CLB result OK.\n");
	return 0;
}

#if 1
E_UPGRADE_ERR_TYPE fts_ctpm_fw_upgrade(u8* pbt_buf, int dw_lenth)
{
	u8  cmd,reg_val[2] = {0};
	u8  packet_buf[FTS_PACKET_LENGTH + 6];
	u8  auc_i2c_write_buf[10];
	u8  bt_ecc;

	int  j, temp, lenght, packet_number, i = 0;
	int  i_is_new_protocol = 0;

	/******write 0xaa to register 0xf3******/

	cmd = 0x81;
	fts_register_write(0xf3, &cmd);
	byte_read(reg_val, 2);
	if (reg_val[0] == 0x00 && reg_val[1] == 0x00) {
		printk("[TSP]  ok: CTPM ID,ID1 = 0x%x,ID2 = 0x%x----Line(%d)\n",reg_val[0],reg_val[1],__LINE__);
	}
	else {
		printk("[TSP]  fail: CTPM ID,ID1 = 0x%x,ID2 = 0x%x----Line(%d)\n",reg_val[0],reg_val[1],__LINE__);
	}

	auc_i2c_write_buf[0] = 0x3c ;
	fts_i2c_txdata(&auc_i2c_write_buf[0], 1);
	udelay(10);

	byte_read(reg_val, 1);
	if (reg_val[0] == 0xFF) {
		printk("[TSP] ok reg_val[0] = 0x%x ----Line(%d) \n",reg_val[0],__LINE__);
	}
	else {
		printk("[TSP] error reg_val[0] = 0x%x ----Line(%d) \n",reg_val[0],__LINE__);
	}

	/******write 0xaa to register 0xfc******/
	cmd = 0xaa;
	fts_register_write(0xfc, &cmd);

	mdelay(50);

	/******write 0x55 to register 0xfc******/
	cmd = 0x55;
	fts_register_write(0xfc, &cmd);
	printk("[TSP] Step 1: Reset CTPM test\n");

	mdelay(10);

	/*******Step 2:Enter upgrade mode ****/
	printk("\n[TSP] Step 2:enter new update mode\n");
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;

	fts_i2c_txdata(&auc_i2c_write_buf[0], 1);
	mdelay(2);
	fts_i2c_txdata(&auc_i2c_write_buf[1], 1);
	mdelay(1000);
	udelay(10);

	/********Step 3:check READ-ID********/
	cmd_write(0x90, 0x00, 0x00, 0x00, 4);
	udelay(100);
	byte_read(reg_val, 2); //orig
	udelay(1500);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x6) {
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
	}
	else {
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
		i_is_new_protocol = 1;
		return ERR_READID;
	}

	/*********Step 4:erase app**********/
	printk("[TSP] Step 4: erase. \n");

	mdelay(3);
	cmd_write(0x61, 0x00, 0x00, 0x00, 1);

	mdelay(3000);

	/*Step 5:write firmware(FW) to ctpm flash*/
	bt_ecc = 0;
	printk("[TSP] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	//packet_number=20;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	printk("\n----packet number is %d-----\n",packet_number);
	for (j = 0; j < packet_number; j++) {
		//printk("----packet number count is %d-----\n", j);
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp >> 8);
		packet_buf[3] = (u8)(temp & 0xff);
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(lenght >> 8);
		packet_buf[5] = (u8)(lenght & 0xff);

		//printk("buf[2]: 0x%X, buf[3]: 0x%X, buf[4]: 0x%X, buf[5]: 0x%X\n", packet_buf[2], packet_buf[3], packet_buf[4], packet_buf[5]);

		for(i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6+i];
		}
		mdelay(20);
		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		//mdelay(FTS_PACKET_LENGTH/6 + 1);
		msleep(12);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0) {
			printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp >> 8);
		packet_buf[3] = (u8)(temp & 0xff);

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		printk("\n-----the remainer data is %d------\n",temp);
		packet_buf[4] = (u8)(temp >> 8);
		packet_buf[5] = (u8)(temp & 0xff);

		printk("buf[2]: 0x%X, buf[3]: 0x%X, buf[4]: 0x%X, buf[5]: 0x%X\n", packet_buf[2], packet_buf[3], packet_buf[4], packet_buf[5]);

		for (i = 0; i < temp; i++) {
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6+i];
		}
		printk("\n--transfer the rest data--\n");
		byte_write(&packet_buf[0],temp+6);
		msleep(20);
	}
	/***********send the last eight bytes**********/

#if 0
	config_add = 0x1ff00;
	packet_buf[0] = 0xbf;
	packet_buf[1] = ((config_add & 0x10000)>>16);
	packet_buf[2] = ((config_add & 0xffff)>>8);
	packet_buf[3] = (config_add & 0xff);
	temp =6;

	packet_buf[4] = 0;
	packet_buf[5] = temp;
	packet_buf[6] = pbt_buf[sizeof(CTPM_FW)-8];
	bt_ecc ^=packet_buf[6];
	packet_buf[7] = pbt_buf[sizeof(CTPM_FW)-7];
	bt_ecc ^=packet_buf[7];
	packet_buf[8] = pbt_buf[sizeof(CTPM_FW)-6];
	bt_ecc ^=packet_buf[8];
	packet_buf[9] = pbt_buf[sizeof(CTPM_FW)-5];
	bt_ecc ^=packet_buf[9];
	packet_buf[10] = pbt_buf[sizeof(CTPM_FW)-4];
	bt_ecc ^=packet_buf[10];
	packet_buf[11] = pbt_buf[sizeof(CTPM_FW)-3];
	bt_ecc ^=packet_buf[11];
	byte_write(&packet_buf[0],12);
#endif

	//send the last six byte
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp >> 8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp = 1;
		packet_buf[4] = (FTS_BYTE)(temp >> 8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i];
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0], 7);
		msleep(20);
	}
	msleep(20);
	/********send the checkout************/
	cmd_write(0xcc, 0x00, 0x00, 0x00, 1);
	mdelay(200);
	//printk("\n  byte read error \n");

	byte_read(reg_val, 1);
	printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc) {
		printk("ECC Error\n");
		return ERR_ECC;
	}
	mdelay(1);
	/*******Step 7: reset the new FW**********/
	cmd_write(0x07, 0x00, 0x00, 0x00, 1);

	gpio_set_value(TS_RESET, 0);
	mdelay(50);
	gpio_set_value(TS_RESET, 1);
	mdelay(40);
	fts_ctpm_auto_clb(); //start auto CLB
	gpio_set_value(TS_RESET, 0);
	mdelay(50);
	gpio_set_value(TS_RESET, 1);
	mdelay(40);

	return ERR_OK;
}

#endif

int fts_ctpm_fw_upgrade_with_i_file(void)
{

#if 0
	FTS_BYTE* pbt_buf = FTS_NULL;
	int i_ret;
	printk("[TSP] chris******************, function (%s)---Line(%d), -- start\n",  __FUNCTION__,__LINE__);
	//=========FW upgrade========================*/
	pbt_buf = CTPM_FW;
	/*call the upgrade function*/
	i_ret = fts_ctpm_fw_upgrade(pbt_buf, sizeof(CTPM_FW));
	if (i_ret != 0) {
		//error handling ...
		printk("[TSP] chris******************, function (%s)---Line(%d), -- error\n",  __FUNCTION__,__LINE__);
	}
	printk("[TSP] chris******************, function (%s)---Line(%d), -- end\n",  __FUNCTION__,__LINE__);

	return i_ret;

#endif

#if 1
	u8* pbt_buf = FTS_NULL;
	int i_ret;
	u16 update_length;

	pbt_buf = CTPM_FW;
#ifdef FT5606_NEW_DRIVER
	printk("CTPM_FW Size: %d\n", sizeof(CTPM_FW));
	printk("\n   the buf lenth high  is %d,low is %d \n", pbt_buf[sizeof(CTPM_FW)-8], pbt_buf[sizeof(CTPM_FW)-7]);
	update_length = pbt_buf[sizeof(CTPM_FW)-8] << 8 | pbt_buf[sizeof(CTPM_FW)-7];
	printk("\n   the buf lenth is %d ", update_length);
	//i_ret =  fts_ctpm_fw_upgrade(pbt_buf,update_length);
	i_ret = fts_ctpm_fw_upgrade(pbt_buf, sizeof(CTPM_FW));
#else
	i_ret = fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
#endif
	return i_ret;
#endif
}

unsigned char fts_ctpm_get_upg_ver(void)
{
	unsigned int ui_sz;

	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2) {
		return CTPM_FW[ui_sz - 2];
	} else
		return 0xff;
}

static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct FTS_TS_DATA_T *ft5816_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char reg_value;
	unsigned char reg_version;
	int i;

	_sui_irq_num = (unsigned int)client->irq;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5816_ts = kzalloc(sizeof(*ft5816_ts), GFP_KERNEL);
	if (!ft5816_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	i2c_set_clientdata(client, ft5816_ts);

	INIT_WORK(&ft5816_ts->pen_event_work, fts_work_func);

	ft5816_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5816_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	/***wait CTP to bootup normally***/
	msleep(200);

	fts_register_read(FT5816_REG_FIRMID, &reg_version, 1);
	printk("[TSP] firmware version = 0x%2x\n", reg_version);
	fts_register_read(FT5816_REG_REPORT_RATE, &reg_value, 1);
	printk("[TSP]firmware report rate = %dHz\n", reg_value * 10);
	fts_register_read(FT5816_REG_THRES, &reg_value, 1);
	printk("[TSP]firmware threshold = %d\n", reg_value * 4);
	fts_register_read(FT5816_REG_NOISE_MODE, &reg_value, 1);
	printk("[TSP]noise mode = 0x%2x\n", reg_value);

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("\n [TSP]:register the early suspend \n");
	ft5816_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5816_ts->early_suspend.suspend = fts_ts_suspend;
	ft5816_ts->early_suspend.resume  = fts_ts_resume;
	register_early_suspend(&ft5816_ts->early_suspend);
#endif

	msleep(200);
	fts_ctpm_fw_upgrade_with_i_file();

#if 0
	if (fts_ctpm_get_upg_ver() != reg_version) {
		printk("[TSP] start upgrade new verison 0x%2x\n", fts_ctpm_get_upg_ver());
		msleep(200);
		err =  fts_ctpm_fw_upgrade_with_i_file();
		if (err == 0) {
			printk("[TSP] ugrade successfuly.\n");
			msleep(300);
			fts_register_read(FT5816_REG_FIRMID, &reg_value, 1);
			printk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_value);
		} else {
			printk("[TSP]  ugrade fail err=%d, line = %d.\n", err, __LINE__);
		}
		msleep(4000);
	}
#endif
	err = request_irq(_sui_irq_num, fts_ts_irq, IRQF_TRIGGER_FALLING, FT5816_NAME, ft5816_ts);

	if (err < 0) {
		dev_err(&client->dev, "ft5816_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(_sui_irq_num);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5816_ts->input_dev = input_dev;

	/***setup coordinate area******/
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);

	/****** for multi-touch *******/
	for (i = 0; i < CFG_MAX_POINT_NUM; i++)
		_st_finger_infos[i].u2_pressure = -1;

	input_set_abs_params(input_dev,
		ABS_MT_POSITION_X, 0, SCREEN_MAX_X - 1, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y - 1, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TRACKING_ID, 0, 30, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_PRESSURE, 0, 32, 0, 0);

	input_set_abs_params(input_dev,
		ABS_X, 0, SCREEN_MAX_X - 1, 0, 0);
	input_set_abs_params(input_dev,
		ABS_Y, 0, SCREEN_MAX_Y - 1, 0, 0);

	input_dev->name = "acer-touch";
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"fts_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	// Create SYSFS
	err = sysdev_class_register(&touch_sysclass);
	if (err) goto exit_sysdev_register_device_failed;
		err = sysdev_register(&device_touch);
	if (err) goto exit_sysdev_register_device_failed;
		err = sysdev_create_file(&device_touch, &attr_ftmping);
	if (err) goto exit_sysdev_register_device_failed;
		err = sysdev_create_file(&device_touch, &attr_ftmgetversion);
	if (err) goto exit_sysdev_register_device_failed;

	enable_irq(_sui_irq_num);
	printk("[TSP] file(%s), function (%s), -- end\n", __FILE__, __FUNCTION__);
	return 0;

exit_sysdev_register_device_failed:
	printk("Touch Probe Sysdev Create File Error %d\n", err);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(_sui_irq_num, ft5816_ts);
exit_irq_request_failed:
	cancel_work_sync(&ft5816_ts->pen_event_work);
	destroy_workqueue(ft5816_ts->ts_workqueue);
exit_create_singlethread:
	printk("[TSP] ==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5816_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit fts_ts_remove(struct i2c_client *client)
{
	struct FTS_TS_DATA_T *ft5816_ts;

	ft5816_ts = (struct FTS_TS_DATA_T *)i2c_get_clientdata(client);
	free_irq(_sui_irq_num, ft5816_ts);

	sysdev_remove_file(&device_touch, &attr_ftmping);
	sysdev_remove_file(&device_touch, &attr_ftmgetversion);

	input_unregister_device(ft5816_ts->input_dev);
	kfree(ft5816_ts);
	cancel_work_sync(&ft5816_ts->pen_event_work);
	destroy_workqueue(ft5816_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5816_ts_id[] = {
	{ FT5816_NAME, 0 },{ }
};

MODULE_DEVICE_TABLE(i2c, ft5816_ts_id);

static struct i2c_driver fts_ts_driver = {
	.probe        = fts_ts_probe,
	.remove        = __devexit_p(fts_ts_remove),
	.id_table    = ft5816_ts_id,
	.driver    = {
		.name = FT5816_NAME,
	},
};

static int __init fts_ts_init(void)
{
	return i2c_add_driver(&fts_ts_driver);
}

static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("<Modify by Benjmain 2011/10/6");
MODULE_DESCRIPTION("FocalTech  Multi-chip solution TouchScreen driver");
MODULE_LICENSE("GPL");
