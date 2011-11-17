/*
  SEC ISX012
 */
/***************************************************************
CAMERA DRIVER FOR 5M CAM (SONY)
****************************************************************/

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>


#include "sec_isx012.h"

#include "sec_cam_pmic.h"
#include "sec_cam_dev.h"


#include <linux/clk.h>
#include <linux/io.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>


#include <asm/mach-types.h>
#include <mach/vreg.h>
#include <linux/io.h>
#include "msm.h"


//#define CONFIG_LOAD_FILE

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#else

#include "sec_isx012_reg.h"	

#endif

#define ISX012_WRITE_LIST(A)			isx012_i2c_write_list(A,(sizeof(A) / sizeof(A[0])),#A);

/*native cmd code*/
#define CAM_AF		1
#define CAM_FLASH	2

struct isx012_work_t {
	struct work_struct work;
};

static struct  isx012_work_t *isx012_sensorw;
static struct  i2c_client *isx012_client;


static struct isx012_ctrl_t *isx012_ctrl;
int temp_i2c_address = 0;
int iscapture = 0;

static DECLARE_WAIT_QUEUE_HEAD(isx012_wait_queue);
DECLARE_MUTEX(isx012_sem);

#ifdef CONFIG_LOAD_FILE

struct test {
	u8 data;
	struct test *nextBuf;
};
static struct test *testBuf;
static s32 large_file;


#define TEST_INIT	\
{			\
	.data = 0;	\
	.nextBuf = NULL;	\
}


static int isx012_write_regs_from_sd(char *name);
static int isx012_regs_table_write(char *name);
#endif

static int isx012_start(void);
 

//#define ISX012_WRITE_LIST(A) \
 //   isx012_i2c_write_list(A,(sizeof(A) / sizeof(A[0])),#A);



 static int isx012_i2c_read_multi(unsigned short subaddr, unsigned long *data)
 {
	 unsigned char buf[4];
	 struct i2c_msg msg = {isx012_client->addr, 0, 2, buf};
 
	 int err = 0;
 
	 if (!isx012_client->adapter) {
		 //dev_err(&isx012_client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		 return -EIO;
	 }
 
	 buf[0] = subaddr>> 8;
	 buf[1] = subaddr & 0xff;
 
	 err = i2c_transfer(isx012_client->adapter, &msg, 1);
	 if (unlikely(err < 0)) {
		 //dev_err(&isx012_client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		 return -EIO;
	 }
 
	 msg.flags = I2C_M_RD;
	 msg.len = 4;
 
	 err = i2c_transfer(isx012_client->adapter, &msg, 1);
	 if (unlikely(err < 0)) {
		 //dev_err(&isx012_client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		 return -EIO;
	 }
 
	 /*
	  * Data comes in Little Endian in parallel mode; So there
	  * is no need for byte swapping here
	  */
	 *data = *(unsigned long *)(&buf);
 
	 return err;
 }

 static int isx012_i2c_read_temp(unsigned short subaddr, unsigned short *data)
{
	unsigned char buf[2];
	struct i2c_msg msg = {0x1a, 0, 2, buf};

	int err = 0;

	if (!isx012_client->adapter) {
		//dev_err(&isx012_client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = subaddr>> 8;
	buf[1] = subaddr & 0xff;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		//dev_err(&isx012_client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	msg.flags = I2C_M_RD;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		//dev_err(&isx012_client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	/*
	 * Data comes in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	*data = *(unsigned short *)(&buf);

	return err;
}


static int isx012_i2c_read(unsigned short subaddr, unsigned short *data)
{
	unsigned char buf[2];
	struct i2c_msg msg = {isx012_client->addr, 0, 2, buf};

	int err = 0;

	if (!isx012_client->adapter) {
		//dev_err(&isx012_client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = subaddr>> 8;
	buf[1] = subaddr & 0xff;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		//dev_err(&isx012_client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	msg.flags = I2C_M_RD;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		//dev_err(&isx012_client->dev, "%s: %d register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	/*
	 * Data comes in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	*data = *(unsigned short *)(&buf);

	return err;
}

static int isx012_i2c_write_multi(unsigned short addr, unsigned int w_data, unsigned int w_len)
{
	unsigned char buf[w_len+2];
	struct i2c_msg msg = {isx012_client->addr, 0, w_len+2, buf};

	int retry_count = 5;
	int err = 0;

	if (!isx012_client->adapter) {
		//dev_err(&isx012_client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = addr >> 8;
	buf[1] = addr & 0xff;

	/*
	 * Data should be written in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	if(w_len == 1) {
		buf[2] = (unsigned char)w_data;
	} else if(w_len == 2)	{
		*((unsigned short *)&buf[2]) = (unsigned short)w_data;
	} else {
		*((unsigned int *)&buf[2]) = w_data;
	}

#if 0 //def ISX012_DEBUG
	{
		int j;
		printk("isx012 i2c write W: ");
		for(j = 0; j <= w_len+1; j++)
		{
			printk("0x%02x ", buf[j]);
		}
		printk("\n");
	}
#endif

	while(retry_count--) {
		err  = i2c_transfer(isx012_client->adapter, &msg, 1);
		if (likely(err == 1))
			break;
//		msleep(POLL_TIME_MS);
	}

	return (err == 1) ? 0 : -EIO;
}

static int isx012_i2c_write_multi_temp(unsigned short addr, unsigned int w_data, unsigned int w_len)
{
	unsigned char buf[w_len+2];
	struct i2c_msg msg = {0x1a, 0, w_len+2, buf};

	int retry_count = 5;
	int err = 0;

	if (!isx012_client->adapter) {
		//dev_err(&isx012_client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = addr >> 8;
	buf[1] = addr & 0xff;

	/*
	 * Data should be written in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	if(w_len == 1) {
		buf[2] = (unsigned char)w_data;
	} else if(w_len == 2)	{
		*((unsigned short *)&buf[2]) = (unsigned short)w_data;
	} else {
		*((unsigned int *)&buf[2]) = w_data;
	}

#if 0//def ISX012_DEBUG
	{
		int j;
		printk("isx012 i2c write W: ");
		for(j = 0; j <= w_len+1; j++)
		{
			printk("0x%02x ", buf[j]);
		}
		printk("\n");
	}
#endif

	while(retry_count--) {
		err  = i2c_transfer(isx012_client->adapter, &msg, 1);
		if (likely(err == 1))
			break;
//		msleep(POLL_TIME_MS);
	}

	return (err == 1) ? 0 : -EIO;
}

static int isx012_i2c_write_list(isx012_short_t regs[], int size, char *name)
{
	printk("[isx012] %s, temp_i2c_address = %d/%d\n", __func__, temp_i2c_address, __LINE__);

#ifdef CONFIG_LOAD_FILE
	isx012_regs_table_write(client, name);
#else
	int err = 0;
	int i = 0;

	if (!isx012_client->adapter) {
		printk(KERN_ERR "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	for (i = 0; i < size; i++) {
		if(regs[i].subaddr == 0xFFFF)
		{
		    msleep(regs[i].value);
                    printk("delay 0x%04x, value 0x%04x\n", regs[i].subaddr, regs[i].value);
		}
                else
                {
        		if(temp_i2c_address == 1)
        			err = isx012_i2c_write_multi_temp(regs[i].subaddr, regs[i].value, regs[i].len);
        		else
        			err = isx012_i2c_write_multi(regs[i].subaddr, regs[i].value, regs[i].len);

        		if (unlikely(err < 0)) {
        			printk(KERN_ERR "%s: register set failed\n",  __func__);
        			return -EIO;
        		}
                }
	}
#endif

	return 0;
}


void isx012_mode_transtion_OM(void)
{
	int count = 0;
	int status = 0;

	printk("[isx012] %s/%d\n", __func__, __LINE__);

	for(count = 0; count < 100 ; count++)
	{
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E (1) read : %x\n", status);

		if((status & 0x1) == 0x1)
			break;
		else
			mdelay(1);
	}
	isx012_i2c_write_multi(0x0012, 0x01, 0x01);
	for(count = 0; count < 100 ; count++)
	{
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E (2) read : %x\n", status);

		if((status & 0x1) == 0x0)
			break;
		else
			mdelay(1);
	}
}


void isx012_mode_transtion_CM(void)
{
	int count = 0;
	int status = 0;

	printk("[isx012] %s/%d\n", __func__, __LINE__);

	for(count = 0; count < 100 ; count++)
	{
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E (1) read : %x\n", status);

		if((status & 0x2) == 0x2)
			break;
		else
			mdelay(1);
	}
	isx012_i2c_write_multi(0x0012, 0x02, 0x01);
	for(count = 0; count < 100 ; count++)
	{
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E (2) read : %x\n", status);

		if((status & 0x2) == 0x0)
			break;
		else
			mdelay(1);
	}
}

static int32_t isx012_i2c_write_32bit(unsigned long packet)
{
	int32_t rc = -EFAULT;
	int retry_count = 1;
	int i;
	
	unsigned char buf[4];
	struct i2c_msg msg = {
		.addr = isx012_client->addr,
		.flags = 0,
		.len = 4,
		.buf = buf,
	};
	*(unsigned long *)buf = cpu_to_be32(packet);

	//for(i=0; i< retry_count; i++) {
	rc = i2c_transfer(isx012_client->adapter, &msg, 1); 		
  	//}

	return rc;
}




#if 0


static int32_t isx012_i2c_write(unsigned short subaddr, unsigned short val)
{
	unsigned long packet;
	packet = (subaddr << 16) | (val&0xFFFF);

	return isx012_i2c_write_32bit(packet);
}


static int32_t isx012_i2c_read(unsigned short subaddr, unsigned short *data)
{

	int ret;
	unsigned char buf[2];

	struct i2c_msg msg = {
		.addr = isx012_client->addr,
		.flags = 0,		
		.len = 2,		
		.buf = buf,	
	};

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(isx012_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	
	if (ret == -EIO) 
	    goto error;
	
	msg.flags = I2C_M_RD;

	ret = i2c_transfer(isx012_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
	    goto error;

	*data = ((buf[0] << 8) | buf[1]);
	

error:
	return ret;

}

#endif


#ifdef CONFIG_LOAD_FILE
static inline int isx012_write(struct i2c_client *client,
		u32 packet)
{
	u8 buf[4];
	int err = 0, retry_count = 5;

	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.buf	= buf,
		.len	= 4,
	};

	if (!client->adapter) {
		cam_err("ERR - can't search i2c client adapter");
		return -EIO;
	}

	while (retry_count--) {
		*(u32 *)buf = cpu_to_be32(packet);
		err = i2c_transfer(client->adapter, &msg, 1);
		if (likely(err == 1))
			break;
		mdelay(10);
	}

	if (unlikely(err < 0)) {
		cam_err("ERR - 0x%08x write failed err=%d",(u32)packet, err);
		return err;
	}

	return (err != 1) ? -1 : 0;
}


void isx012_regs_table_init(void)
{
	struct file *fp = NULL;
	struct test *nextBuf = NULL;

	u8 *nBuf = NULL;
	size_t file_size = 0, max_size = 0, testBuf_size = 0;
	ssize_t nread = 0;
	s32 check = 0, starCheck = 0;
	s32 tmp_large_file = 0;
	s32 i = 0;
	int ret = 0;
	loff_t pos;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	BUG_ON(testBuf);

	//fp = filp_open("/mnt/sdcard/external_sd/sec_s5k4ecgx_reg.h", O_RDONLY, 0);
	fp = filp_open("/mnt/sdcard/sec_isx012_reg.h", O_RDONLY, 0);
	if (IS_ERR(fp)) {
		cam_err("failed to open /mnt/sdcard/sec_isx012_reg.h");
		return PTR_ERR(fp);
	}

	file_size = (size_t) fp->f_path.dentry->d_inode->i_size;
	max_size = file_size;

	cam_info("file_size = %d", file_size);

	nBuf = kmalloc(file_size, GFP_ATOMIC);
	if (nBuf == NULL) {
		cam_err("Fail to 1st get memory");
		nBuf = vmalloc(file_size);
		if (nBuf == NULL) {
			cam_err("ERR: nBuf Out of Memory");
			ret = -ENOMEM;
			goto error_out;
		}
		tmp_large_file = 1;
	}

	testBuf_size = sizeof(struct test) * file_size;
	if (tmp_large_file) {
		testBuf = (struct test *)vmalloc(testBuf_size);
		large_file = 1;
	} else {
		testBuf = kmalloc(testBuf_size, GFP_ATOMIC);
		if (testBuf == NULL) {
			cam_err("Fail to get mem(%d bytes)", testBuf_size);
			testBuf = (struct test *)vmalloc(testBuf_size);
			large_file = 1;
		}
	}
	if (testBuf == NULL) {
		cam_err("ERR: Out of Memory");
		ret = -ENOMEM;
		goto error_out;
	}

	pos = 0;
	memset(nBuf, 0, file_size);
	memset(testBuf, 0, file_size * sizeof(struct test));

	nread = vfs_read(fp, (char __user *)nBuf, file_size, &pos);
	if (nread != file_size) {
		cam_err("failed to read file ret = %d", nread);
		ret = -1;
		goto error_out;
	}

	set_fs(fs);

	i = max_size;

	cam_info("i = %d", i);

	while (i) {
		testBuf[max_size - i].data = *nBuf;
		if (i != 1) {
			testBuf[max_size - i].nextBuf = &testBuf[max_size - i + 1];
		} else {
			testBuf[max_size - i].nextBuf = NULL;
			break;
		}
		i--;
		nBuf++;
	}

	i = max_size;
	nextBuf = &testBuf[0];

#if 1
	while (i - 1) {
		if (!check && !starCheck) {
			if (testBuf[max_size - i].data == '/') {
				if (testBuf[max_size-i].nextBuf != NULL) {
					if (testBuf[max_size-i].nextBuf->data
								== '/') {
						check = 1;/* when find '//' */
						i--;
					} else if (testBuf[max_size-i].nextBuf->data == '*') {
						starCheck = 1;/* when find '/ *' */
						i--;
					}
				} else
					break;
			}
			if (!check && !starCheck) {
				/* ignore '\t' */
				if (testBuf[max_size - i].data != '\t') {
					nextBuf->nextBuf = &testBuf[max_size-i];
					nextBuf = &testBuf[max_size - i];
				}
			}
		} else if (check && !starCheck) {
			if (testBuf[max_size - i].data == '/') {
				if(testBuf[max_size-i].nextBuf != NULL) {
					if (testBuf[max_size-i].nextBuf->data == '*') {
						starCheck = 1; /* when find '/ *' */
						check = 0;
						i--;
					}
				} else
					break;
			}

			 /* when find '\n' */
			if (testBuf[max_size - i].data == '\n' && check) {
				check = 0;
				nextBuf->nextBuf = &testBuf[max_size - i];
				nextBuf = &testBuf[max_size - i];
			}

		} else if (!check && starCheck) {
			if (testBuf[max_size - i].data == '*') {
				if (testBuf[max_size-i].nextBuf != NULL) {
					if (testBuf[max_size-i].nextBuf->data == '/') {
						starCheck = 0; /* when find '* /' */
						i--;
					}
				} else
					break;
			}
		}

		i--;

		if (i < 2) {
			nextBuf = NULL;
			break;
		}

		if (testBuf[max_size - i].nextBuf == NULL) {
			nextBuf = NULL;
			break;
		}
	}
#endif

#if 0 // for print
	printk("i = %d\n", i);
	nextBuf = &testBuf[0];
	while (1) {
		//printk("sdfdsf\n");
		if (nextBuf->nextBuf == NULL)
			break;
		printk("%c", nextBuf->data);
		nextBuf = nextBuf->nextBuf;
	}
#endif

error_out:

	if (nBuf)
		tmp_large_file ? vfree(nBuf) : kfree(nBuf);
	if (fp)
		filp_close(fp, current->files);
	return ret;
}


void isx012_regs_table_exit(void)
{
	if (testBuf) {
		large_file ? vfree(testBuf) : kfree(testBuf);
		large_file = 0;
		testBuf = NULL;
	}
}

static int isx012_write_regs_from_sd(char *name)
{
	struct test *tempData = NULL;

	int ret = -EAGAIN;
	u32 temp;
	u32 delay = 0;
	u8 data[11];
	s32 searched = 0;
	size_t size = strlen(name);
	s32 i;

	CAM_DEBUG("E size = %d, string = %s", size, name);
	tempData = &testBuf[0];

	while (!searched) {
		searched = 1;
		for (i = 0; i < size; i++) {
			if (tempData->data != name[i]) {
				searched = 0;
				break;
			}
			tempData = tempData->nextBuf;
		}
		tempData = tempData->nextBuf;
	}
	/* structure is get..*/

	while (1) {
		if (tempData->data == '{')
			break;
		else
			tempData = tempData->nextBuf;
	}

	while (1) {
		searched = 0;
		while (1) {
			if (tempData->data == 'x') {
				/* get 10 strings.*/
				data[0] = '0';
				for (i = 1; i < 11; i++) {
					data[i] = tempData->data;
					tempData = tempData->nextBuf;
				}
				/*CAM_DEBUG("%s\n", data);*/
				temp = simple_strtoul(data, NULL, 16);
				break;
			} else if (tempData->data == '}') {
				searched = 1;
				break;
			} else
				tempData = tempData->nextBuf;

			if (tempData->nextBuf == NULL)
				return -1;
		}

		if (searched)
			break;
		if ((temp & ISX012_DELAY) == ISX012_DELAY) {
			delay = temp & 0xFFFF;
			cam_info("delay(%d)",delay);
			msleep(delay);
			continue;
		}
		ret = isx012_write(isx012_client,temp);

		/* In error circumstances */
		/* Give second shot */
		if (unlikely(ret)) {
			ret = isx012_write(isx012_client,temp);

			/* Give it one more shot */
			if (unlikely(ret)) {
				ret = isx012_write(isx012_client, temp);
			}
		}
	}
	return 0;
}
#endif



static void isx012_set_ae_lock(char value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d",value);
	printk("isx012_set_ae_lock ");
#if 0
    	switch (value) {
        case EXT_CFG_AE_LOCK:
            	isx012_ctrl->setting.ae_lock = EXT_CFG_AE_LOCK;
            	 ISX012_WRITE_LIST(isx012_ae_lock);
        	break;
        case EXT_CFG_AE_UNLOCK:
            	isx012_ctrl->setting.ae_lock = EXT_CFG_AE_UNLOCK;
            	ISX012_WRITE_LIST(isx012_ae_unlock);
        	break;
        case EXT_CFG_AWB_LOCK:
            	isx012_ctrl->setting.awb_lock = EXT_CFG_AWB_LOCK;
               ISX012_WRITE_LIST(isx012_awb_lock);
        	break;
        case EXT_CFG_AWB_UNLOCK:
            	isx012_ctrl->setting.awb_lock = EXT_CFG_AWB_UNLOCK;
            	ISX012_WRITE_LIST(isx012_awb_unlock);
        	break;
        default:
		cam_err("Invalid(%d)", value);
        	break;
    	}
#endif		
     return err;


}

static long isx012_set_effect(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d",value);
	printk("isx012_set_effect ");
	
retry:
	switch (value) {
	case CAMERA_EFFECT_OFF:
		 ISX012_WRITE_LIST(isx012_Effect_Normal);
		break;
	case CAMERA_EFFECT_SEPIA:
		ISX012_WRITE_LIST(isx012_Effect_Sepia);
		break;
	case CAMERA_EFFECT_MONO:
		 ISX012_WRITE_LIST(isx012_Effect_Black_White);
		break;
	case CAMERA_EFFECT_NEGATIVE:
		ISX012_WRITE_LIST(isx012_Effect_Normal);
		break;
	default:
		cam_err("Invalid(%d)", value);
		value = CAMERA_EFFECT_OFF;
		goto retry;
	}
	
	isx012_ctrl->setting.effect = value;
	return err;
}


static void isx012_set_REG_TC_DBG_AutoAlgEnBits(int bit, int set)
{
	printk("isx012_set_REG_TC_DBG_AutoAlgEnBits ");
#if 0
	int REG_TC_DBG_AutoAlgEnBits = 0;
    
   	/* Read 04E6 */
	    isx012_i2c_write(0x002C,0x7000);
    	isx012_i2c_write(0x002E,0x04E6);
    	isx012_i2c_read(0x0F12, (unsigned short*)&REG_TC_DBG_AutoAlgEnBits);

    	if (bit == 3 && set == true) {
        	if (REG_TC_DBG_AutoAlgEnBits & 0x8 == 1)
			return;
		
        	if (isx012_ctrl->setting.scene == EXT_CFG_SCENE_NIGHTSHOT)
			mdelay(250);
        	else
			mdelay(100);
		
	        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits | 0x8;
	        isx012_i2c_write(0x0028, 0x7000);
	        isx012_i2c_write(0x002A, 0x04E6);
	        isx012_i2c_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    	}
    	else if (bit == 3 && set == false) {
	        if (REG_TC_DBG_AutoAlgEnBits & 0x8 == 0)
			return;
	        if (isx012_ctrl->setting.scene == EXT_CFG_SCENE_NIGHTSHOT)
			mdelay(250);
	        else 
			mdelay(100);
		
	        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits & 0xFFF7;
	        isx012_i2c_write(0x0028, 0x7000);
	        isx012_i2c_write(0x002A, 0x04E6);
	        isx012_i2c_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    	}
    	else if (bit == 5 && set == true) {
	        if (REG_TC_DBG_AutoAlgEnBits & 0x20 == 1)
			return;
	        if( isx012_ctrl->setting.scene == EXT_CFG_SCENE_NIGHTSHOT)
			mdelay(250);
	        else 
			mdelay(100);
		
	        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits | 0x20;
	        isx012_i2c_write(0x0028, 0x7000);
	        isx012_i2c_write(0x002A, 0x04E6);
	        isx012_i2c_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    	}
    	else if (bit == 5 && set == false) {
	        if (REG_TC_DBG_AutoAlgEnBits & 0x20 == 0)
			return;
	        if (isx012_ctrl->setting.scene == EXT_CFG_SCENE_NIGHTSHOT)
			mdelay(250);
	        else 
			mdelay(100);
		
	        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits & 0xFFDF;
	        isx012_i2c_write(0x0028, 0x7000);
	        isx012_i2c_write(0x002A, 0x04E6);
	        isx012_i2c_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    	}

   	return;
#endif

}


static int isx012_set_whitebalance(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d",value);
	printk("isx012_set_whitebalance ");

    	switch (value) {
        case WHITE_BALANCE_AUTO :
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,1);
            	ISX012_WRITE_LIST (isx012_WB_Auto);
        	break;
        case WHITE_BALANCE_SUNNY:
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            	 ISX012_WRITE_LIST(isx012_WB_Sunny);
        	break;
        case WHITE_BALANCE_CLOUDY :
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            	 ISX012_WRITE_LIST(isx012_WB_Cloudy);
        	break;
        case WHITE_BALANCE_FLUORESCENT:
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            	 ISX012_WRITE_LIST(isx012_WB_Fluorescent);
        	break;
        case WHITE_BALANCE_INCANDESCENT:
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            	ISX012_WRITE_LIST(isx012_WB_Tungsten);
        	break;
        default :
		cam_err("Invalid(%d)", value);
        	break;
        }

	isx012_ctrl->setting.whiteBalance = value;
	return err;
}

static int  isx012_set_brightness(int8_t value)
{
	int err = -EINVAL;
	
	CAM_DEBUG("%d",value);
	printk("isx012_set_brightness ");
	//if(s5k4ecgx_ctrl->check_dataline)
	//	return 0;

	switch (value) {
	case EV_MINUS_4 :
		 ISX012_WRITE_LIST(isx012_EV_Minus_4);
		break;
	case EV_MINUS_3 :
		 ISX012_WRITE_LIST(isx012_EV_Minus_3);
		break;
	case EV_MINUS_2 :
		 ISX012_WRITE_LIST(isx012_EV_Minus_2);
		break;
	case EV_MINUS_1 :
		 ISX012_WRITE_LIST(isx012_EV_Minus_3);
		break;
	case EV_DEFAULT :
		ISX012_WRITE_LIST(isx012_EV_Default);
		break;
	case EV_PLUS_1 :
		 ISX012_WRITE_LIST(isx012_EV_Plus_1);
		break;
	case EV_PLUS_2 :
		 ISX012_WRITE_LIST(isx012_EV_Plus_2);
		break;
	case EV_PLUS_3 :
		 ISX012_WRITE_LIST(isx012_EV_Plus_3);
		break;
	case EV_PLUS_4 :
		ISX012_WRITE_LIST(isx012_EV_Plus_4);
		break;
	default :
		cam_err("Invalid(%d)", value);
		break;
	}

	isx012_ctrl->setting.brightness = value;
	return err;
}


static int  isx012_set_iso(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d", value);
	printk("isx012_set_iso ");
	
	switch (value) {
        case ISO_AUTO :
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(5,1);
            	 ISX012_WRITE_LIST(isx012_ISO_Auto);
        	break;
        case ISO_50 :
           //	isx012_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            	 ISX012_WRITE_LIST(isx012_ISO_50);
       	 	break;
        case ISO_100 :
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            	ISX012_WRITE_LIST(isx012_ISO_100);
        	break;
        case ISO_200 :
		 		//isx012_set_REG_TC_DBG_AutoAlgEnBits(5,0);
		 		 ISX012_WRITE_LIST(isx012_ISO_200);
		break;
        case ISO_400 :
            	//isx012_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            	 ISX012_WRITE_LIST(isx012_ISO_400);
        	break;
        default :
		cam_err("Invalid(%d)", value);
        	break;
	}

	isx012_ctrl->setting.iso = value;    
	return err;
}


static int isx012_set_metering(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d", value);
	printk("isx012_set_metering ");

retry:
	switch (value) {
	case METERING_MATRIX:
		 ISX012_WRITE_LIST(isx012_Metering_Matrix);
		break;
	case METERING_CENTER:
		 ISX012_WRITE_LIST(isx012_Metering_Center);
		break;
	case METERING_SPOT:
		 ISX012_WRITE_LIST(isx012_Metering_Spot);
		break;
	default:
		cam_err("Invalid(%d)", value);
		value = METERING_CENTER;
		goto retry;
	}

	isx012_ctrl->setting.metering = value;
	return err;
}



static int isx012_set_contrast(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d",value);
	printk("isx012_set_contrast ");

retry:
	switch (value) {
	case CONTRAST_MINUS_2 :
		ISX012_WRITE_LIST(isx012_Contrast_Minus_2);
		break;
        case CONTRAST_MINUS_1 :
		ISX012_WRITE_LIST(isx012_Contrast_Minus_1);
		break;
        case CONTRAST_DEFAULT :
		ISX012_WRITE_LIST(isx012_Contrast_Default);
		break;
        case CONTRAST_PLUS_1 :
		ISX012_WRITE_LIST(isx012_Contrast_Plus_1);
		break;
	case CONTRAST_PLUS_2 :
		ISX012_WRITE_LIST(isx012_Contrast_Plus_2);
        	break;
        default :
		cam_err("Invalid(%d)", value);
		value = METERING_CENTER;
		goto retry;
   	}

	isx012_ctrl->setting.contrast = value;	
	return err;
}


static int isx012_set_saturation(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d",value);
	printk("isx012_set_saturation ");

retry:
	switch (value) {
	case SATURATION_MINUS_2 :
		ISX012_WRITE_LIST (isx012_Saturation_Minus_2);
		break;
        case SATURATION_MINUS_1 :
		ISX012_WRITE_LIST(isx012_Saturation_Minus_1);
		break;
        case SATURATION_DEFAULT :
		 ISX012_WRITE_LIST(isx012_Saturation_Default);
		break;
        case SATURATION_PLUS_1 :
		ISX012_WRITE_LIST(isx012_Saturation_Plus_1);
		break;
	case SATURATION_PLUS_2 :
		 ISX012_WRITE_LIST(isx012_Saturation_Plus_2);
        	break;
        default :
		cam_err("Invalid(%d)", value);
		value = METERING_CENTER;
		goto retry;
   	}

	isx012_ctrl->setting.saturation = value;
	return err;
}


static int isx012_set_sharpness(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d",value);
	printk("isx012_set_sharpness ");

retry:
	switch (value) {
	case SHARPNESS_MINUS_2 :
		 ISX012_WRITE_LIST(isx012_Sharpness_Minus_2);
		break;
        case SHARPNESS_MINUS_1 :
		 ISX012_WRITE_LIST(isx012_Sharpness_Minus_1);
		break;
        case SHARPNESS_DEFAULT :
	      ISX012_WRITE_LIST(isx012_Sharpness_Default);
		break;
        case SHARPNESS_PLUS_1 :
		 ISX012_WRITE_LIST(isx012_Sharpness_Plus_1);
		break;
	case SHARPNESS_PLUS_2 :
		 ISX012_WRITE_LIST(isx012_Sharpness_Plus_2);
        	break;
        default :
		cam_err("Invalid(%d)", value);
		value = METERING_CENTER;
		goto retry;
   	}

	isx012_ctrl->setting.sharpness = value;
	return err;
}




static int  isx012_set_scene(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d",value);
	printk("isx012_set_scene ");
#if 0
	if (value != EXT_CFG_SCENE_OFF) {
	     ISX012_WRITE_LIST(isx012_Scene_Default);

	}
	
	switch (value) {
	case EXT_CFG_SCENE_OFF:
		 ISX012_WRITE_LIST(isx012_Scene_Default);
	    	break;
	case EXT_CFG_SCENE_PORTRAIT: 
		 ISX012_WRITE_LIST(isx012_Scene_Portrait);
		break;
	case EXT_CFG_SCENE_LANDSCAPE:
		 ISX012_WRITE_LIST(isx012_Scene_Landscape);
		err = isx012_set_metering(EXT_CFG_METERING_NORMAL);
		break;
	case EXT_CFG_SCENE_SPORTS:
		ISX012_WRITE_LIST(isx012_Scene_Sports);
	    	break;
	case EXT_CFG_SCENE_PARTY:
		//isx012_set_REG_TC_DBG_AutoAlgEnBits(5,0);
		ISX012_WRITE_LIST(isx012_Scene_Party_Indoor);
	    	break;
	case EXT_CFG_SCENE_BEACH:
		//isx012_set_REG_TC_DBG_AutoAlgEnBits(5,0);
		ISX012_WRITE_LIST(isx012_Scene_Beach_Snow);
		break;
	case EXT_CFG_SCENE_SUNSET:
		//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,0);
		ISX012_WRITE_LIST(isx012_Scene_Sunset);
		break;
	case EXT_CFG_SCENE_DAWN:
		//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,0);
		ISX012_WRITE_LIST(isx012_Scene_Duskdawn);
	    	break;
	case EXT_CFG_SCENE_FALL:
		ISX012_WRITE_LIST(isx012_Scene_Fall_Color);
	    	break;
	case EXT_CFG_SCENE_NIGHTSHOT:
		ISX012_WRITE_LIST(isx012_Scene_Nightshot);
	    	break;
	case EXT_CFG_SCENE_BACKLIGHT:
		 ISX012_WRITE_LIST(isx012_Scene_Backlight);
		//if(s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_ON ||s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_AUTO)s5k4ecgx_set_metering(EXT_CFG_METERING_CENTER);
		//else s5k4ecgx_set_metering(EXT_CFG_METERING_SPOT);
	   	break;
	case EXT_CFG_SCENE_FIREWORK:
		 ISX012_WRITE_LIST(isx012_Scene_Fireworks);
		 ISX012_WRITE_LIST(isx012_ISO_50);
		break;
	case EXT_CFG_SCENE_TEXT:
		 ISX012_WRITE_LIST(isx012_Scene_Text);
		break;
	case EXT_CFG_SCENE_CANDLE:
		//isx012_set_REG_TC_DBG_AutoAlgEnBits(3,0);
		ISX012_WRITE_LIST(isx012_Scene_Candle_Light);
		break;
	default:
		cam_err("Invalid(%d)", value);
	    	break;
	}


	isx012_ctrl->setting.scene = value;
#endif	
	return err;
}



static int isx012_set_preview_size( int8_t value)
{
	CAM_DEBUG("%d",value);
	printk("isx012_set_preview_size ");
		
	#if 0
	if(HD_mode) {
		    HD_mode = 0;
		    S5K4ECGX_WRITE_LIST(s5k4ecgx_1280_Preview_D)
	}

	
	switch (value) {
	case EXT_CFG_PREVIEW_SIZE_640x480_VGA:
	case EXT_CFG_PREVIEW_SIZE_176x144_QCIF:
		ISX012_WRITE_LIST(isx012_640_Preview);
		break;
	case EXT_CFG_PREVIEW_SIZE_800x480_WVGA:	
		ISX012_WRITE_LIST(isx012_800_Preview);
		break;
	case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: 
		ISX012_WRITE_LIST(isx012_176_Preview);
		break;
	case EXT_CFG_PREVIEW_SIZE_720x480_D1: 
		ISX012_WRITE_LIST(isx012_720_Preview);
		break;
	case EXT_CFG_PREVIEW_SIZE_1280x720_D1: 
		//HD_mode = 1;
		ISX012_WRITE_LIST(isx012_1280_Preview_E);
		break;
	default:
		cam_err("Invalid");
		break;
	}
	

	isx012_ctrl->setting.preview_size= value;
	#endif

	return 0;
}




static int isx012_set_picture_size(int value)
{
	CAM_DEBUG("%d",value);
	printk("isx012_set_picture_size ");
#if 0	
	switch (value) {
	case EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M:
		ISX012_WRITE_LIST(isx012_5M_Capture);
		//s5k4ecgx_set_zoom(EXT_CFG_ZOOM_STEP_0);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_2560x1536_4M_WIDE:
		ISX012_WRITE_LIST(isx012_4M_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
		ISX012_WRITE_LIST(isx012_5M_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
		ISX012_WRITE_LIST(isx012_2_4M_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
		ISX012_WRITE_LIST(isx012_5M_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:
		ISX012_WRITE_LIST(isx012_1_5M_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
		ISX012_WRITE_LIST(isx012_1M_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:
		ISX012_WRITE_LIST(isx012_4K_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
		ISX012_WRITE_LIST(isx012_VGA_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_320x240_QVGA:
		ISX012_WRITE_LIST(isx012_QVGA_Capture);
		break;
	default:
		cam_err("Invalid");
		return -EINVAL;
	}


//	if(size != EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M && s5k4ecgx_status.zoom != EXT_CFG_ZOOM_STEP_0)
//		s5k4ecgx_set_zoom(s5k4ecgx_status.zoom);


	isx012_ctrl->setting.snapshot_size = value;
#endif
	return 0;
}



static int isx012_set_movie_mode(int mode)
{

	if ((mode != SENSOR_CAMERA) && (mode != SENSOR_MOVIE)) {
		return -EINVAL;
	}


	return 0;
}


static int isx012_check_dataline(s32 val)
{
	int err = -EINVAL;

	CAM_DEBUG("%s", val ? "ON" : "OFF");
	if (val) {
		 ISX012_WRITE_LIST(isx012_DTP_init);
		
	} else {
		 ISX012_WRITE_LIST(isx012_DTP_stop);
	}

	return err;
}

static int isx012_mipi_mode(int mode)
{
	int rc = 0;
	struct msm_camera_csi_params isx012_csi_params;
	
	CAM_DEBUG("E");

	if (!isx012_ctrl->status.config_csi1) {
		isx012_csi_params.lane_cnt = 2;
		isx012_csi_params.data_format = 0x1E;
		isx012_csi_params.lane_assign = 0xe4;
		isx012_csi_params.dpcm_scheme = 0;
		isx012_csi_params.settle_cnt = 24;// 0x14; //0x7; //0x14;
		rc = msm_camio_csi_config(&isx012_csi_params);
		
		if (rc < 0)
			printk(KERN_ERR "config csi controller failed \n");
		
		isx012_ctrl->status.config_csi1 = 1;
	}
	
	CAM_DEBUG("X");
	return rc;
}


static int isx012_start(void)
{
	int rc=0;
	int err = -EINVAL;

	CAM_DEBUG("E");
	printk("isx012_start ");
	if (isx012_ctrl->status.started) {
		CAM_DEBUG("X : already started");
		return rc;
	}
	isx012_mipi_mode(1);
	msleep(30); //=> Please add some delay 
	
	
	//ISX012_WRITE_LIST(isx012_init_reg1);
	msleep(10);
	
        //ISX012_WRITE_LIST(isx012_init_reg2);

	isx012_ctrl->status.initialized = 1;
	isx012_ctrl->status.started = 1;
	CAM_DEBUG("X");
	return rc;
}



static long isx012_video_config(int mode)
{
	int err = -EINVAL;
	CAM_DEBUG("E");

	ISX012_WRITE_LIST(isx012_Preview_Return);

	return err;

}

static long isx012_snapshot_config(int mode)
{
	int err = -EINVAL;
	CAM_DEBUG("E");

	 ISX012_WRITE_LIST(ISX012_Capture_SizeSetting);
	 
	 ISX012_WRITE_LIST(isx012_Capture_Start);
	 
	 isx012_mode_transtion_CM();

	
	return err;
}



static long isx012_set_sensor_mode(int mode)
{
	int err = -EINVAL;
	printk("isx012_set_sensor_mode ");

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		CAM_DEBUG("SENSOR_PREVIEW_MODE START");
		if(iscapture == 1)
		{
			iscapture = 0;
			ISX012_WRITE_LIST(ISX012_Preview_Mode);		
			isx012_mode_transtion_CM();			
		}
		isx012_start();
		
		//if (s5k4ecgx_ctrl->sensor_mode != SENSOR_MOVIE)
		err= isx012_video_config(SENSOR_PREVIEW_MODE);

		break;

	case SENSOR_SNAPSHOT_MODE:
	case SENSOR_RAW_SNAPSHOT_MODE:	
		CAM_DEBUG("SENSOR_SNAPSHOT_MODE START");
		iscapture = 1;
		err= isx012_snapshot_config(SENSOR_SNAPSHOT_MODE);

		break;

	
	case SENSOR_SNAPSHOT_TRANSFER:
		CAM_DEBUG("SENSOR_SNAPSHOT_TRANSFER START");

		break;
		
	default:
		return 0;//-EFAULT;
	}

	return 0;
}

static int isx012_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	int temp = 0;
	int status = 0;
	int count = 0;

	printk("POWER ON START ");
	printk("isx012_sensor_init_probe ");

	gpio_tlmm_config(GPIO_CFG(50, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //reset
	gpio_tlmm_config(GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //stanby
//	gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//	gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);


	gpio_set_value_cansleep(50, 0);
	temp = gpio_get_value(50);
	printk("[isx012] CAM_5M_RST : %d\n", temp);

	gpio_set_value_cansleep(49, 0);
	temp = gpio_get_value(49);
	printk("[isx012] CAM_5M_STBY : %d\n", temp);

	cam_ldo_power_on();
 
	//msm_camio_clk_rate_set(info->mclk);
	//	mdelay(5); //min 50us
	gpio_set_value_cansleep(50, 1);
	temp = gpio_get_value(50);
	printk("[isx012] CAM_5M_RST : %d\n", temp);
	mdelay(10);


	//cam_ldo_power_off();	
	//mdelay(10);
	printk("[isx012] Mode Trandition 1\n");

	for(count = 0; count < 100 ; count++)
	{
		//isx012_i2c_read_temp(0x000E, (unsigned short*)&status);
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E(1) read : %x\n", status);

		if(status == 1)
			break;
		else
			mdelay(1);
	}
	//isx012_i2c_write_multi_temp(0x0012, 0x01, 0x01);
	isx012_i2c_write_multi(0x0012, 0x01, 0x01);
	for(count = 0; count < 100 ; count++)
	{
		//isx012_i2c_read_temp(0x000E, (unsigned short*)&status);
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E(2) read : %x\n", status);

		if(status == 0)
			break;
		else
			mdelay(1);
	}
	//temp_i2c_address = 1;
	mdelay(10);
	ISX012_WRITE_LIST(ISX012_Pll_Setting_2);
	printk("[isx012] Mode Trandition 2\n");

//	isx012_i2c_read_temp(0x0014, (unsigned short*)&status);
//	printk("[isx012] device status : %x\n", status);

	for(count = 0; count < 100 ; count++)
	{
		//isx012_i2c_read_temp(0x000E, (unsigned short*)&status);
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E(3) read : %x\n", status);

		if(status == 1)
			break;
		else
			mdelay(1);
	}
	//isx012_i2c_write_multi_temp(0x0012, 0x01, 0x01);
	isx012_i2c_write_multi(0x0012, 0x01, 0x01);
	for(count = 0; count < 100 ; count++)
	{
		//isx012_i2c_read_temp(0x000E, (unsigned short*)&status);
		isx012_i2c_read(0x000E, (unsigned short*)&status);
		printk("[isx012] 0x000E(4) read : %x\n", status);

		if(status == 0)
			break;
		else
			mdelay(1);
	}
	printk("[isx012] MIPI write\n");

	//isx012_i2c_write_multi_temp(0x5008, 0x00, 0x01);
	isx012_i2c_write_multi(0x5008, 0x00, 0x01);
	
	ISX012_WRITE_LIST(ISX012_Init_Reg);
	ISX012_WRITE_LIST(ISX012_Preview_SizeSetting);
	ISX012_WRITE_LIST(ISX012_Preview_Mode);
	temp_i2c_address = 0;

	gpio_set_value_cansleep(49, 1); //STBY 0 -> 1
	temp = gpio_get_value(49);
	printk("[isx012] CAM_5M_ISP_STNBY : %d\n", temp);

	mdelay(10);

	isx012_mode_transtion_OM();

	isx012_mode_transtion_CM();

	mdelay(50);


#ifdef CONFIG_LOAD_FILE
	isx012_regs_table_init();
#endif
	
	
	CAM_DEBUG("POWER ON END ");

	return rc;
}




int isx012_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	CAM_DEBUG("E");
	printk("isx012_sensor_open_init ");
	
	isx012_ctrl = kzalloc(sizeof(struct isx012_ctrl_t), GFP_KERNEL);
	if (!isx012_ctrl) {
		cam_err("failed!");
		rc = -ENOMEM;
		goto init_done;
	}	
	
	if (data)
		isx012_ctrl->sensordata = data;
 
	
	rc = isx012_sensor_init_probe(data);
	if (rc < 0) {
		cam_err("isx012_sensor_open_init failed!");
		goto init_fail;
	}


	
	isx012_ctrl->status.started = 0;
	isx012_ctrl->status.initialized = 0;
	isx012_ctrl->status.config_csi1 = 0;
	
	isx012_ctrl->setting.check_dataline = 0;
	isx012_ctrl->setting.camera_mode = SENSOR_CAMERA;

	

	CAM_DEBUG("X");
init_done:
	return rc;

init_fail:
	kfree(isx012_ctrl);
	return rc;
}

static int isx012_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&isx012_wait_queue);
	return 0;
}

static int isx012_sensor_af_status(void)
{
	int ret = 0;
	int status = 0;

	printk(KERN_DEBUG "[isx012] %s/%d\n", __func__, __LINE__);

	isx012_i2c_read(0x8B8A, (unsigned short *)&status);
	if ((status & 0x8) == 0x8) {
		ret = 1;
		printk(KERN_DEBUG "[isx012] success\n");
	}

	return ret;
}

static int isx012_sensor_af_result(void)
{
	int ret = 0;
	int status = 0;

	printk(KERN_DEBUG "[isx012] %s/%d\n", __func__, __LINE__);

	isx012_i2c_read(0x8B8B, (unsigned short *)&status);
	if ((status & 0x1) == 0x1) {
		printk(KERN_DEBUG "[isx012] AF success\n");
		ret = 1;
	} else if ((status & 0x1) == 0x0) {
		printk(KERN_DEBUG "[isx012] AF fail\n");
		ret = 2;
	}
	return ret;
}


int isx012_sensor_ext_config(void __user *argp)
{

	sensor_ext_cfg_data		cfg_data;
	int rc=0;
	printk("isx012_sensor_ext_config ");

	if (copy_from_user((void *)&cfg_data, (const void *)argp, sizeof(cfg_data))){
		cam_err("fail copy_from_user!");
	}

	CAM_DEBUG("cmd = %d , param1 = %d",cfg_data.cmd,cfg_data.value_1);
	#if 0
	if( (cfg_data.cmd != EXT_CFG_SET_DTP)
		&& (cfg_data.cmd != EXT_CFG_SET_VT_MODE)	
		&& (cfg_data.cmd != EXT_CFG_SET_MOVIE_MODE)	
		&& (!s5k4ecgx_ctrl->status.initialized)){
		cam_err("camera isn't initialized\n");
		return 0;
	}
	#endif
	switch (cfg_data.cmd) {
	case 50:
		if (cfg_data.value_1 == 0) {
			ISX012_WRITE_LIST(ISX012_Halfrelease_Mode);
		} else if (cfg_data.value_1 == 1) {
			cfg_data.value_2 = isx012_sensor_af_status();
			if (cfg_data.value_2 == 1)
				isx012_i2c_write_multi(0x0012, 0x10, 0x01);
		} else if (cfg_data.value_1 == 2) {
			cfg_data.value_2 = isx012_sensor_af_result();
		}
		break;
		
		
	case EXT_CFG_SET_BRIGHTNESS:
		rc = isx012_set_brightness(cfg_data.value_1);
		break;

	case EXT_CFG_SET_EFFECT:
		rc = isx012_set_effect(cfg_data.value_1);
		break;	
		
	case EXT_CFG_SET_ISO:
		rc = isx012_set_iso(cfg_data.value_1);
		break;
		
	case EXT_CFG_SET_WB:
		rc = isx012_set_whitebalance(cfg_data.value_1);
		break;

	case EXT_CFG_SET_SCENE:
		rc = isx012_set_scene(cfg_data.value_1);
		break;

	case EXT_CFG_SET_METERING:	// auto exposure mode
		rc = isx012_set_metering(cfg_data.value_1);
		break;

	case EXT_CFG_SET_CONTRAST:
		rc = isx012_set_contrast(cfg_data.value_1);
		break;

	case EXT_CFG_SET_SHARPNESS:
		rc = isx012_set_sharpness(cfg_data.value_1);
		break;

	case EXT_CFG_SET_SATURATION:
		rc = isx012_set_saturation(cfg_data.value_1);
		break;
		
	case EXT_CFG_SET_PREVIEW_SIZE:	
		rc = isx012_set_preview_size(cfg_data.value_1);
		break;

	case EXT_CFG_SET_PICTURE_SIZE:	
		rc = isx012_set_picture_size(cfg_data.value_1);
		break;

	case EXT_CFG_SET_JPEG_QUALITY:	
		//rc = isx012_set_jpeg_quality(cfg_data.value_1);
		break;
		
	case EXT_CFG_SET_FPS:
		//rc = isx012_set_frame_rate(cfg_data.value_1,cfg_data.value_2);
		break;

	case EXT_CFG_SET_DTP:
		break;

 	case EXT_CFG_SET_VT_MODE:
		cam_info("VTCall mode : %d",cfg_data.value_1);
		break;
		
	case EXT_CFG_SET_MOVIE_MODE:
		cam_info("MOVIE mode : %d",cfg_data.value_1);
		isx012_set_movie_mode(cfg_data.value_1);
		break;

	case EXT_CFG_SET_AF_OPERATION:
		cam_info("AF mode : %d",cfg_data.value_1);
#if 0		
		if (ctrl_info.address == 0) {
			ISX012_WRITE_LIST(ISX012_Halfrelease_Mode);
		} else if (ctrl_info.address == 1) {
			ctrl_info.value_1 = isx012_sensor_af_status();
			if (ctrl_info.value_1 == 1)
			 isx012_i2c_write_multi(0x0012, 0x10, 0x01);
		} else if (ctrl_info.address == 2) {
			ctrl_info.value_1 = isx012_sensor_af_result();
		}
#endif		
		break;		

	default:
		break;
	}

	if (copy_to_user((void *)argp, (const void *)&cfg_data, sizeof(cfg_data))){
		cam_err("fail copy_from_user!");
	}
	
	return rc;	
}

int isx012_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,(void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&m5mo_sem); */

	CAM_DEBUG("cfgtype = %d, mode = %d", cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = isx012_set_sensor_mode(cfg_data.mode);
		break;
		
	default:
		rc = 0;//-EFAULT;
		break;
	}

	/* up(&m5mo_sem); */

	return rc;
}

int isx012_sensor_release(void)
{
	int rc = 0;
	int temp = 0;

	/* down(&m5mo_sem); */

	CAM_DEBUG("POWER OFF START");
	gpio_tlmm_config(GPIO_CFG(50, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //reset
	gpio_tlmm_config(GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //stanby
//	gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//	gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	//gpio_set_value_cansleep(CAM_VGA_RST, LOW);
	mdelay(1);

	cam_ldo_power_off();	// have to turn off MCLK before PMIC

	gpio_set_value_cansleep(50, 0);
	temp = gpio_get_value(50);
	printk("[isx012] CAM_5M_RST : %d\n", temp);

	gpio_set_value_cansleep(49, 0);
	temp = gpio_get_value(49);
	printk("[isx012] CAM_5M_STBY : %d\n", temp);

	isx012_ctrl->status.initialized = 1;
	kfree(isx012_ctrl);
	
#ifdef CONFIG_LOAD_FILE
	isx012_regs_table_exit();
#endif
	CAM_DEBUG("POWER OFF END");
	/* up(&m5mo_sem); */

	return rc;
}


 static int isx012_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

	CAM_DEBUG("E");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	isx012_sensorw = kzalloc(sizeof(struct isx012_work_t), GFP_KERNEL);

	if (!isx012_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, isx012_sensorw);
	isx012_init_client(client);
	isx012_client = client;

	CAM_DEBUG("E");


	return 0;

probe_failure:
	kfree(isx012_sensorw);
	isx012_sensorw = NULL;
	cam_err("isx012_i2c_probe failed!");
	return rc;
}


static int __exit isx012_i2c_remove(struct i2c_client *client)
{

	struct isx012_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
//	i2c_detach_client(client);
	isx012_client = NULL;
	isx012_sensorw = NULL;
	kfree(sensorw);
	return 0;

}


static const struct i2c_device_id isx012_id[] = {
    { "isx012_i2c", 0 },
    { }
};

//PGH MODULE_DEVICE_TABLE(i2c, s5k4ecgx);

static struct i2c_driver isx012_i2c_driver = {
	.id_table	= isx012_id,
	.probe  	= isx012_i2c_probe,
	.remove 	= __exit_p(isx012_i2c_remove),
	.driver 	= {
		.name = "isx012",
	},
};


int32_t isx012_i2c_init(void)
{
	int32_t rc = 0;

	CAM_DEBUG("E");

	rc = i2c_add_driver(&isx012_i2c_driver);

	if (IS_ERR_VALUE(rc))
		goto init_failure;

	return rc;



init_failure:
	cam_err("failed to isx012_i2c_init, rc = %d", rc);
	return rc;
}


void isx012_exit(void)
{
	i2c_del_driver(&isx012_i2c_driver); 	
}


//int m5mo_sensor_probe(void *dev, void *ctrl)
int isx012_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = 0;

	printk("############# isx012_sensor_probe ##############\n");
/*	struct msm_camera_sensor_info *info =
		(struct msm_camera_sensor_info *)dev; 

	struct msm_sensor_ctrl *s =
		(struct msm_sensor_ctrl *)ctrl;
*/

 
	rc = isx012_i2c_init();
	if (rc < 0)
		goto probe_done;

 	s->s_init	= isx012_sensor_open_init;
	s->s_release	= isx012_sensor_release;
	s->s_config	= isx012_sensor_config;
	s->s_ext_config	= isx012_sensor_ext_config;
	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = 0;

probe_done:
	cam_err("error, rc = %d", rc);
	return rc;
	
}


static int __sec_isx012_probe(struct platform_device *pdev)
{
	printk("############# __sec_isx012_probe ##############\n");
	return msm_camera_drv_start(pdev, isx012_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sec_isx012_probe,
	.driver = {
		.name = "msm_camera_isx012",
		.owner = THIS_MODULE,
	},
};

static int __init sec_isx012_camera_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

static void __exit sec_isx012_camera_exit(void)
{
	platform_driver_unregister(&msm_camera_driver);
}

module_init(sec_isx012_camera_init);
module_exit(sec_isx012_camera_exit);

