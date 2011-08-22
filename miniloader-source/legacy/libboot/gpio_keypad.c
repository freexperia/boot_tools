/*
 * Copyright (C) 2008 The Android Open Source Project
 * All rights reserved.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the 
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <boot/boot.h>
#include <boot/gpio.h>
#include <boot/gpio_keypad.h>

int gpio_keypad_init(gpio_keypad_info *keypad)
{
	unsigned i;
	for(i = 0; i < keypad->noutputs; i++) {
		gpio_set(keypad->output_gpios[i], keypad->polarity ^ keypad->drive_inactive_outputs);
		gpio_dir(keypad->output_gpios[i], keypad->drive_inactive_outputs);
	}
	for(i = 0; i < keypad->ninputs; i++) {
		gpio_dir(keypad->input_gpios[i], 0);
	}
	keypad->state = 0;
	return 0;
}

void gpio_keypad_scan_keys(gpio_keypad_info *keypad)
{
	unsigned out, in;
	unsigned long long keys;
	unsigned npolarity = !keypad->polarity;
	unsigned int shift;

	keys = 0;
	out = keypad->noutputs;
	shift = keypad->noutputs * keypad->ninputs;
	while(out > 0) {
		out--;
		if(keypad->drive_inactive_outputs)
			gpio_set(keypad->output_gpios[out], !npolarity);
		else	
			gpio_dir(keypad->output_gpios[out], 1);
		udelay(keypad->settle_time);
		in = keypad->ninputs;
		while(in > 0) {
			in--;
			shift--;
			keys = (keys << 1) | (gpio_get(keypad->input_gpios[in]) ^ npolarity);
			if(((unsigned)(keypad->state >> shift) ^ (unsigned)keys) & 1) {
				unsigned int mapped_key = 0;
				if(keypad->key_map)
					mapped_key = keypad->key_map[shift];
				//dprintf("gpio_keypad_scan_keys: %d-%d (%d-%d) %d (%d): %d\n", out, in,
				//        keypad->output_gpios[out], keypad->input_gpios[in],
				//        shift, mapped_key, keys & 1);
				if(mapped_key && key_changed)
					key_changed(mapped_key, keys & 1);
			}
		}
		if(keypad->drive_inactive_outputs)
			gpio_set(keypad->output_gpios[out], npolarity);
		else	
			gpio_dir(keypad->output_gpios[out], 0);
	}
	if(keys != keypad->state) {
		keypad->state = keys;
		//dprintf("gpio_keypad_scan_keys: %x %x\n", (unsigned long)(keys >> 32), (unsigned long)keys);
	}
}

int i2c_ssbi_poll_for_device_ready(void)
{
	unsigned long timeout = SSBI_TIMEOUT_US;

	while (!(readl(MSM_SSBI_BASE + SSBI2_STATUS) & SSBI_STATUS_READY)) {
		if (--timeout == 0) {
			dprintf("In Device ready function:Timeout, status %x\n", readl(MSM_SSBI_BASE + SSBI2_STATUS));
			return 1;
		}
	}

	return 0;
}

int i2c_ssbi_poll_for_read_completed(void)
{
	unsigned long timeout = SSBI_TIMEOUT_US;

	while (!(readl(MSM_SSBI_BASE + SSBI2_STATUS) & SSBI_STATUS_RD_READY)) {
		if (--timeout == 0) {
            dprintf("In read completed function:Timeout, status %x\n", readl(MSM_SSBI_BASE + SSBI2_STATUS));
			return 1;
		}
	}

	return 0;
}

int i2c_ssbi_read_bytes(unsigned char  *buffer, unsigned short length,
                                                unsigned short slave_addr)
{
	int ret = 0;
	unsigned char *buf = buffer;
	unsigned short len = length;
	unsigned short addr = slave_addr;
	unsigned long read_cmd = SSBI_CMD_READ(addr);
	unsigned long mode2 = readl(MSM_SSBI_BASE + SSBI2_MODE2);

	//buf = alloc(len * sizeof(8));
	if (mode2 & SSBI_MODE2_SSBI2_MODE)
		writel(SSBI_MODE2_REG_ADDR_15_8(mode2, addr),
				MSM_SSBI_BASE + SSBI2_MODE2);

	while (len) {
		ret = i2c_ssbi_poll_for_device_ready();
		if (ret) {
			dprintf ("Error: device not ready\n");
			return ret;
		}

		writel(read_cmd, MSM_SSBI_BASE + SSBI2_CMD);

		ret = i2c_ssbi_poll_for_read_completed();
		if (ret) {
			dprintf ("Error: read not completed\n");
			return ret;
		}

		*buf++ = readl(MSM_SSBI_BASE + SSBI2_RD) & SSBI_RD_REG_DATA_MASK;
		len--;
	}
	return 0;
}

int i2c_ssbi_write_bytes(unsigned char  *buffer, unsigned short length,
                                                unsigned short slave_addr)
{
	int ret = 0;
	unsigned long timeout = SSBI_TIMEOUT_US;
	unsigned char *buf = buffer;
	unsigned short len = length;
	unsigned short addr = slave_addr;
	unsigned long mode2 = readl(MSM_SSBI_BASE + SSBI2_MODE2);

	if (mode2 & SSBI_MODE2_SSBI2_MODE)
		writel(SSBI_MODE2_REG_ADDR_15_8(mode2, addr),
				MSM_SSBI_BASE + SSBI2_MODE2);

	while (len) {
		ret = i2c_ssbi_poll_for_device_ready();
		if (ret) {
			dprintf ("Error: device not ready\n");
			return ret;
		}

		writel(SSBI_CMD_WRITE(addr, *buf++), MSM_SSBI_BASE + SSBI2_CMD);

		while (readl(MSM_SSBI_BASE + SSBI2_STATUS) & SSBI_STATUS_MCHN_BUSY) {
		  if (--timeout == 0) {
		    dprintf("In Device ready function:Timeout, status %x\n", readl(MSM_SSBI_BASE + SSBI2_STATUS));
		    return 1;
		  }
		}
		len--;
	}
	return 0;
}

int pm8058_gpio_config(int gpio, struct pm8058_gpio *param)
{
	int	rc;
	unsigned char bank[8];
	static int dir_map[] = {
		PM8058_GPIO_MODE_OFF,
		PM8058_GPIO_MODE_OUTPUT,
		PM8058_GPIO_MODE_INPUT,
		PM8058_GPIO_MODE_BOTH,
	};

	if (param == 0) {
	  dprintf ("pm8058_gpio struct not defined\n");
          return -1;
	}

	/* Select banks and configure the gpio */
	bank[0] = PM8058_GPIO_WRITE |
		((param->vin_sel << PM8058_GPIO_VIN_SHIFT) &
			PM8058_GPIO_VIN_MASK) |
		PM8058_GPIO_MODE_ENABLE;
	bank[1] = PM8058_GPIO_WRITE |
		((1 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((dir_map[param->direction] << PM8058_GPIO_MODE_SHIFT) &
			PM8058_GPIO_MODE_MASK) |
		((param->direction & PM_GPIO_DIR_OUT) ?
			PM8058_GPIO_OUT_BUFFER : 0);
	bank[2] = PM8058_GPIO_WRITE |
		((2 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((param->pull << PM8058_GPIO_PULL_SHIFT) &
			PM8058_GPIO_PULL_MASK);
	bank[3] = PM8058_GPIO_WRITE |
		((3 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((param->out_strength << PM8058_GPIO_OUT_STRENGTH_SHIFT) &
			PM8058_GPIO_OUT_STRENGTH_MASK);
	bank[4] = PM8058_GPIO_WRITE |
		((4 << PM8058_GPIO_BANK_SHIFT) & PM8058_GPIO_BANK_MASK) |
		((param->function << PM8058_GPIO_FUNC_SHIFT) &
			PM8058_GPIO_FUNC_MASK);

	rc = i2c_ssbi_write_bytes(bank, 5, SSBI_REG_ADDR_GPIO(gpio));
	if (rc) {
		dprintf("Failed on 1st ssbi_write(): rc=%d.\n", rc);
		return 1;
	}
	return 0;
}

int pm8058_gpio_config_kypd_drv(int gpio_start, int num_gpios)
{
	int	rc;
	struct pm8058_gpio kypd_drv = {
		.direction	= PM_GPIO_DIR_OUT,
		.pull		= PM_GPIO_PULL_NO,
		.vin_sel	= 2,
		.out_strength	= PM_GPIO_STRENGTH_LOW,
		.function	= PM_GPIO_FUNC_1,
		.inv_int_pol	= 1,
	};

	while (num_gpios--) {
		rc = pm8058_gpio_config(gpio_start++, &kypd_drv);
		if (rc) {
			dprintf("FAIL pm8058_gpio_config(): rc=%d.\n", rc);
			return rc;
		}
	}

	return 0;
}

int pm8058_gpio_config_kypd_sns(int gpio_start, int num_gpios)
{
	int	rc;
	struct pm8058_gpio kypd_sns = {
		.direction	= PM_GPIO_DIR_IN,
		.pull		= PM_GPIO_PULL_UP1,
		.vin_sel	= 2,
		.out_strength	= PM_GPIO_STRENGTH_NO,
		.function	= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol	= 1,
	};

	while (num_gpios--) {
		rc = pm8058_gpio_config(gpio_start++, &kypd_sns);
		if (rc) {
			dprintf("FAIL pm8058_gpio_config(): rc=%d.\n", rc);
			return rc;
		}
	}

	return 0;
}


int ssbi_keypad_init(void)
{
    int loop = 0;
    unsigned char kypd_cntl_init = 0xFC;
    unsigned char kypd_scan_init = 0x00;
    int *modem_stat_check = (SMEM_BASE + 0x14);

    /* Wait for modem to be ready before keypad init */
    while (readl(modem_stat_check) != 1);

    if (i2c_ssbi_write_bytes(&kypd_cntl_init, 1, SSBI_REG_KYPD_CNTL_ADDR))
      dprintf ("Error in initializing SSBI_REG_KYPD_CNTL register\n");

    if (i2c_ssbi_write_bytes(&kypd_scan_init, 1, SSBI_REG_KYPD_SCAN_ADDR))
      dprintf ("Error in initializing SSBI_REG_KYPD_SCAN register\n");

    pm8058_gpio_config_kypd_sns(SSBI_OFFSET_ADDR_GPIO_KYPD_SNS, 8);
    pm8058_gpio_config_kypd_drv(SSBI_OFFSET_ADDR_GPIO_KYPD_DRV, 18);

}


void scan_qwerty_keypad(qwerty_keypad_info  *qwerty_keypad)
{
    int rows = qwerty_keypad->rows;
    int columns = qwerty_keypad->columns;
    unsigned char column_keys = 0x00;
    int shift = 0;
    static int key_detected = 0;

    ssbi_keypad_init();

    if (i2c_ssbi_read_bytes(qwerty_keypad->rec_keys, 18, SSBI_REG_KYPD_REC_DATA_ADDR))
      dprintf ("Error in initializing SSBI_REG_KYPD_CNTL register\n");

    if (i2c_ssbi_read_bytes(qwerty_keypad->old_keys, 18, SSBI_REG_KYPD_OLD_DATA_ADDR))
      dprintf ("Error in initializing SSBI_REG_KYPD_CNTL register\n");

    while (rows--) {
        if (qwerty_keypad->rec_keys[rows] != qwerty_keypad->old_keys[rows]) {
	    while (columns--) {
	        column_keys = (qwerty_keypad->rec_keys[rows]);
	        if ((0x01 << columns) & (~column_keys)) {
	            shift = (rows * qwerty_keypad->columns) + columns;
	            if (qwerty_keypad->key_map[shift]) {
		        if (shift != key_detected) {
                            key_changed(qwerty_keypad->key_map[shift], 1);
			    key_detected = shift;
		        }
		        break;
	            }
		}
	    }
	}
    }
}
