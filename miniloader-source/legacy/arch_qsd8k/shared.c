/*
 * Copyright (c) 2008, Google Inc.
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
#include <qsd8k/shared.h>

static void get_version(char *s, unsigned id)
{
    unsigned *ver = (unsigned*) MSM7K_VERSION;
    unsigned n = ver[id];
    
    snprintf(s, 32, "%d.%d", n >> 16, n & 0xffff);
}

void get_version_modem(char *s)
{
    get_version(s, VERSION_MODEM);
}

void get_version_modem_sbl(char *s)
{
    get_version(s, VERSION_MODEM_SBL);
}

#define MSM_CSR_BASE 0xAC100000

#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)

static inline void notify_other_proc_comm(void)
{
	writel(1, MSM_A2M_INT(6));
}

#define APP_COMMAND (MSM7K_SHARED_PHYS + 0x00)
#define APP_STATUS  (MSM7K_SHARED_PHYS + 0x04)
#define APP_DATA1   (MSM7K_SHARED_PHYS + 0x08)
#define APP_DATA2   (MSM7K_SHARED_PHYS + 0x0C)

#define MDM_COMMAND (MSM7K_SHARED_PHYS + 0x10)
#define MDM_STATUS  (MSM7K_SHARED_PHYS + 0x14)
#define MDM_DATA1   (MSM7K_SHARED_PHYS + 0x18)
#define MDM_DATA2   (MSM7K_SHARED_PHYS + 0x1C)


#if 0
enum
{
	PCOM_CMD_IDLE = 0x0,
	PCOM_CMD_DONE,
	PCOM_RESET_APPS,
	PCOM_RESET_CHIP,
	PCOM_CONFIG_NAND_MPU,
	PCOM_CONFIG_USB_CLKS,
	PCOM_GET_POWER_ON_STATUS,
	PCOM_GET_WAKE_UP_STATUS,
	PCOM_GET_BATT_LEVEL,
	PCOM_CHG_IS_CHARGING,
	PCOM_POWER_DOWN,
	PCOM_USB_PIN_CONFIG,
	PCOM_USB_PIN_SEL,
	PCOM_SET_RTC_ALARM,
	PCOM_NV_READ,
	PCOM_NV_WRITE,
	PCOM_GET_UUID_HIGH,
	PCOM_GET_UUID_LOW,
	PCOM_GET_HW_ENTROPY,
	PCOM_RPC_GPIO_TLMM_CONFIG_REMOTE,
	PCOM_CLKCTL_RPC_ENABLE,
	PCOM_CLKCTL_RPC_DISABLE,
	PCOM_CLKCTL_RPC_RESET,
	PCOM_CLKCTL_RPC_SET_FLAGS,
	PCOM_CLKCTL_RPC_SET_RATE,
	PCOM_CLKCTL_RPC_MIN_RATE,
	PCOM_CLKCTL_RPC_MAX_RATE,
	PCOM_CLKCTL_RPC_RATE,
	PCOM_CLKCTL_RPC_PLL_REQUEST,
	PCOM_CLKCTL_RPC_ENABLED,
	PCOM_VREG_SWITCH,
	PCOM_VREG_SET_LEVEL,
	PCOM_GPIO_TLMM_CONFIG_GROUP,
	PCOM_GPIO_TLMM_UNCONFIG_GROUP,
	PCOM_NV_WRITE_BYTES_4_7,
	PCOM_CONFIG_DISP,
	PCOM_GET_FTM_BOOT_COUNT,
	PCOM_RPC_GPIO_TLMM_CONFIG_EX,
	PCOM_PM_MPP_CONFIG,
	PCOM_GPIO_IN,
	PCOM_GPIO_OUT,
	PCOM_RESET_MODEM,
	PCOM_RESET_CHIP_IMM,
	PCOM_PM_VID_EN,
	PCOM_VREG_PULLDOWN,
	PCOM_GET_MODEM_VERSION,
	PCOM_CLK_REGIME_SEC_RESET,
	PCOM_CLK_REGIME_SEC_RESET_ASSERT,
	PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
	PCOM_CLK_REGIME_SEC_PLL_REQUEST_WRP,
	PCOM_CLK_REGIME_SEC_ENABLE,
	PCOM_CLK_REGIME_SEC_DISABLE,
	PCOM_CLK_REGIME_SEC_IS_ON,
	PCOM_CLK_REGIME_SEC_SEL_CLK_INV,
	PCOM_CLK_REGIME_SEC_SEL_CLK_SRC,
	PCOM_CLK_REGIME_SEC_SEL_CLK_DIV,
	PCOM_CLK_REGIME_SEC_ICODEC_CLK_ENABLE,
	PCOM_CLK_REGIME_SEC_ICODEC_CLK_DISABLE,
	PCOM_CLK_REGIME_SEC_SEL_SPEED,
	PCOM_CLK_REGIME_SEC_CONFIG_GP_CLK_WRP,
	PCOM_CLK_REGIME_SEC_CONFIG_MDH_CLK_WRP,
	PCOM_CLK_REGIME_SEC_USB_XTAL_ON,
	PCOM_CLK_REGIME_SEC_USB_XTAL_OFF,
	PCOM_CLK_REGIME_SEC_SET_QDSP_DME_MODE,
	PCOM_CLK_REGIME_SEC_SWITCH_ADSP_CLK,
	PCOM_CLK_REGIME_SEC_GET_MAX_ADSP_CLK_KHZ,
	PCOM_CLK_REGIME_SEC_GET_I2C_CLK_KHZ,
	PCOM_CLK_REGIME_SEC_MSM_GET_CLK_FREQ_KHZ,
	PCOM_CLK_REGIME_SEC_SEL_VFE_SRC,
	PCOM_CLK_REGIME_SEC_MSM_SEL_CAMCLK,
	PCOM_CLK_REGIME_SEC_MSM_SEL_LCDCLK,
	PCOM_CLK_REGIME_SEC_VFE_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_VFE_RAIL_ON,
	PCOM_CLK_REGIME_SEC_GRP_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_GRP_RAIL_ON,
	PCOM_CLK_REGIME_SEC_VDC_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_VDC_RAIL_ON,
	PCOM_CLK_REGIME_SEC_LCD_CTRL,
	PCOM_CLK_REGIME_SEC_REGISTER_FOR_CPU_RESOURCE,
	PCOM_CLK_REGIME_SEC_DEREGISTER_FOR_CPU_RESOURCE,
	PCOM_CLK_REGIME_SEC_RESOURCE_REQUEST_WRP,
	PCOM_CLK_REGIME_MSM_SEC_SEL_CLK_OWNER,
	PCOM_CLK_REGIME_SEC_DEVMAN_REQUEST_WRP,
	PCOM_GPIO_CONFIG,
	PCOM_GPIO_CONFIGURE_GROUP,
	PCOM_GPIO_TLMM_SET_PORT, 
	PCOM_GPIO_TLMM_CONFIG_EX,
	PCOM_SET_FTM_BOOT_COUNT,
	PCOM_RESERVED0,      /* smem_pcmod.c: used to test proc comm */
	PCOM_RESERVED1,      /* smem_pcmod.c: used to test proc comm */
	PCOM_CUSTOMER_CMD1,  /* reserved for OEM specified commands  */
	PCOM_CUSTOMER_CMD2,
	PCOM_CUSTOMER_CMD3,
	PCOM_CLK_REGIME_ENTER_APPSBL_CHG_MODE,
	PCOM_CLK_REGIME_EXIT_APPSBL_CHG_MODE,
	PCOM_CLK_REGIME_SEC_RAIL_DISABLE,
	PCOM_CLK_REGIME_SEC_RAIL_ENABLE,
	PCOM_CLK_REGIME_SEC_RAIL_CONTROL,
	PCOM_SET_SW_WATCHDOG_STATE,
	PCOM_PM_MPP_CONFIG_DIGITAL_INPUT,
	PCOM_PM_MPP_CONFIG_I_SINK,
	PCOM_RESERVED_101,
	PCOM_MSM_HSUSB_PHY_RESET,
	PCOM_GET_BATT_MV_LEVEL,
	PCOM_CHG_USB_IS_PC_CONNECTED,
	PCOM_CHG_USB_IS_CHARGER_CONNECTED,
	PCOM_CHG_USB_IS_DISCONNECTED,
	PCOM_CHG_USB_I_AVAILABLE,
	PCOM_CLK_REGIME_SEC_MSM_SEL_FREQ,
	PCOM_CLK_REGIME_SEC_SET_PCLK_AXI_POLICY,
	PCOM_NUM_CMDS,
};

enum
{
	 PCOM_INVALID_STATUS = 0x0,
	 PCOM_READY,
	 PCOM_CMD_RUNNING,
	 PCOM_CMD_SUCCESS,
	 PCOM_CMD_FAIL,
};
#else
enum {
        PCOM_CMD_IDLE = 0x0,
        PCOM_CMD_DONE,
        PCOM_RESET_APPS,
        PCOM_RESET_CHIP,
        PCOM_CONFIG_NAND_MPU,
        PCOM_CONFIG_USB_CLKS,
        PCOM_GET_POWER_ON_STATUS,
        PCOM_GET_WAKE_UP_STATUS,
        PCOM_GET_BATT_LEVEL,
        PCOM_CHG_IS_CHARGING,
        PCOM_POWER_DOWN,
        PCOM_USB_PIN_CONFIG,
        PCOM_USB_PIN_SEL,
        PCOM_SET_RTC_ALARM,
        PCOM_NV_READ,
        PCOM_NV_WRITE,
        PCOM_GET_UUID_HIGH,
        PCOM_GET_UUID_LOW,
        PCOM_GET_HW_ENTROPY,
        PCOM_RPC_GPIO_TLMM_CONFIG_REMOTE,
        PCOM_CLKCTL_RPC_ENABLE,
        PCOM_CLKCTL_RPC_DISABLE,
        PCOM_CLKCTL_RPC_RESET,
        PCOM_CLKCTL_RPC_SET_FLAGS,
        PCOM_CLKCTL_RPC_SET_RATE,
        PCOM_CLKCTL_RPC_MIN_RATE,
        PCOM_CLKCTL_RPC_MAX_RATE,
        PCOM_CLKCTL_RPC_RATE,
        PCOM_CLKCTL_RPC_PLL_REQUEST,
        PCOM_CLKCTL_RPC_ENABLED,
        PCOM_VREG_SWITCH,
        PCOM_VREG_SET_LEVEL,
        PCOM_GPIO_TLMM_CONFIG_GROUP,
        PCOM_GPIO_TLMM_UNCONFIG_GROUP,
        PCOM_NV_WRITE_BYTES_4_7,
        PCOM_CONFIG_DISP,
        PCOM_GET_FTM_BOOT_COUNT,
        PCOM_RPC_GPIO_TLMM_CONFIG_EX,
        PCOM_PM_MPP_CONFIG,
        PCOM_GPIO_IN,
        PCOM_GPIO_OUT,
        PCOM_RESET_MODEM,
        PCOM_RESET_CHIP_IMM,
        PCOM_PM_VID_EN,
        PCOM_VREG_PULLDOWN,
        PCOM_GET_MODEM_VERSION,
        PCOM_CLK_REGIME_SEC_RESET,
        PCOM_CLK_REGIME_SEC_RESET_ASSERT,
        PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
        PCOM_CLK_REGIME_SEC_PLL_REQUEST_WRP,
        PCOM_CLK_REGIME_SEC_ENABLE,
        PCOM_CLK_REGIME_SEC_DISABLE,
        PCOM_CLK_REGIME_SEC_IS_ON,
        PCOM_CLK_REGIME_SEC_SEL_CLK_INV,
        PCOM_CLK_REGIME_SEC_SEL_CLK_SRC,
        PCOM_CLK_REGIME_SEC_SEL_CLK_DIV,
        PCOM_CLK_REGIME_SEC_ICODEC_CLK_ENABLE,
        PCOM_CLK_REGIME_SEC_ICODEC_CLK_DISABLE,
        PCOM_CLK_REGIME_SEC_SEL_SPEED,
        PCOM_CLK_REGIME_SEC_CONFIG_GP_CLK_WRP,
        PCOM_CLK_REGIME_SEC_CONFIG_MDH_CLK_WRP,
        PCOM_CLK_REGIME_SEC_USB_XTAL_ON,
        PCOM_CLK_REGIME_SEC_USB_XTAL_OFF,
        PCOM_CLK_REGIME_SEC_SET_QDSP_DME_MODE,
        PCOM_CLK_REGIME_SEC_SWITCH_ADSP_CLK,
        PCOM_CLK_REGIME_SEC_GET_MAX_ADSP_CLK_KHZ,
        PCOM_CLK_REGIME_SEC_GET_I2C_CLK_KHZ,
        PCOM_CLK_REGIME_SEC_MSM_GET_CLK_FREQ_KHZ,
        PCOM_CLK_REGIME_SEC_SEL_VFE_SRC,
        PCOM_CLK_REGIME_SEC_MSM_SEL_CAMCLK,
        PCOM_CLK_REGIME_SEC_MSM_SEL_LCDCLK,
        PCOM_CLK_REGIME_SEC_VFE_RAIL_OFF,
        PCOM_CLK_REGIME_SEC_VFE_RAIL_ON,
        PCOM_CLK_REGIME_SEC_GRP_RAIL_OFF,
        PCOM_CLK_REGIME_SEC_GRP_RAIL_ON,
        PCOM_CLK_REGIME_SEC_VDC_RAIL_OFF,
        PCOM_CLK_REGIME_SEC_VDC_RAIL_ON,
        PCOM_CLK_REGIME_SEC_LCD_CTRL,
        PCOM_CLK_REGIME_SEC_REGISTER_FOR_CPU_RESOURCE,
        PCOM_CLK_REGIME_SEC_DEREGISTER_FOR_CPU_RESOURCE,
        PCOM_CLK_REGIME_SEC_RESOURCE_REQUEST_WRP,
        PCOM_CLK_REGIME_MSM_SEC_SEL_CLK_OWNER,
        PCOM_CLK_REGIME_SEC_DEVMAN_REQUEST_WRP,
        PCOM_GPIO_CONFIG,
        PCOM_GPIO_CONFIGURE_GROUP,
        PCOM_GPIO_TLMM_SET_PORT,
        PCOM_GPIO_TLMM_CONFIG_EX,
        PCOM_SET_FTM_BOOT_COUNT,
        PCOM_RESERVED0,
        PCOM_RESERVED1,
        PCOM_CUSTOMER_CMD1,
        PCOM_CUSTOMER_CMD2,
        PCOM_CUSTOMER_CMD3,
        PCOM_CLK_REGIME_ENTER_APPSBL_CHG_MODE,
        PCOM_CLK_REGIME_EXIT_APPSBL_CHG_MODE,
        POCM_CLK_REGIME_SEC_RAIL_DISABLE,
        POCM_CLK_REGIME_SEC_RAIL_ENABLE,
        POCM_CLK_REGIME_SEC_RAIL_CONTROL,
        POCM_SET_SW_WATCHDOG_STATE,
        POCM_PM_MPP_CONFIG_DIGITAL_INPUT,
        POCM_PM_MPP_CONFIG_I_SINK,
        POCM_RESERVED_101,
        POCM_MSM_HSUSB_PHY_RESET,
        PCOM_GET_BATT_MV_LEVEL,
        PCOM_CHG_USB_IS_PC_CONNECTED,
        PCOM_CHG_USB_IS_CHARGER_CONNECTED,
        PCOM_CHG_USB_IS_DISCONNECTED,
        PCOM_CHG_USB_IS_AVAILABLE,
        PCOM_CLK_REGIME_SEC_MSM_SEL_FREQ,
        PCOM_CLK_REGIME_SEC_SET_PCLK_AXI_POLICY,
        PCOM_CLKCTL_RPC_RESET_ASSERT,
        PCOM_CLKCTL_RPC_RESET_DEASSERT,
        PCOM_CLKCTL_RPC_RAIL_ON,
        PCOM_CLKCTL_RPC_RAIL_OFF,
        PCOM_CLKCTL_RPC_RAIL_ENABLE,
        PCOM_CLKCTL_RPC_RAIL_DISABLE,
        PCOM_CLKCTL_RPC_RAIL_CONTROL,
        PCOM_CLKCTL_RPC_MIN_MSMC1,
};

enum {
        PCOM_OEM_FIRST_CMD = 0x10000000,
        PCOM_OEM_TEST_CMD = PCOM_OEM_FIRST_CMD,

        /* add OEM PROC COMM commands here */
/* SEMC: - start */
    PCOM_OEM_POWER_DOWN,
    PCOM_OEM_RESET_CHIP,
        PCOM_SET_HEADSET_ADC_OPT,
        PCOM_SET_HEADSET_ADC_THRESHOLD_OPT,
        PCOM_HEADSET_PLUGGED,
        PCOM_HEADSET_UNPLUGGED,
        PCOM_GET_HEADSET_ADC_VALUE,
        PCOM_REG_HEADSET_EVENT,
        PCOM_HEADSET_SUSPEND,
        PCOM_HEADSET_RESUME,
        PCOM_OEM_LAST = PCOM_HEADSET_RESUME,
/* SEMC: - end */
};

enum {
        PCOM_INVALID_STATUS = 0x0,
        PCOM_READY,
        PCOM_CMD_RUNNING,
        PCOM_CMD_SUCCESS,
        PCOM_CMD_FAIL,
        PCOM_CMD_FAIL_FALSE_RETURNED,
        PCOM_CMD_FAIL_CMD_OUT_OF_BOUNDS_SERVER,
        PCOM_CMD_FAIL_CMD_OUT_OF_BOUNDS_CLIENT,
        PCOM_CMD_FAIL_CMD_UNREGISTERED,
        PCOM_CMD_FAIL_CMD_LOCKED,
        PCOM_CMD_FAIL_SERVER_NOT_YET_READY,
        PCOM_CMD_FAIL_BAD_DESTINATION,
        PCOM_CMD_FAIL_SERVER_RESET,
        PCOM_CMD_FAIL_SMSM_NOT_INIT,
        PCOM_CMD_FAIL_PROC_COMM_BUSY,
        PCOM_CMD_FAIL_PROC_COMM_NOT_INIT,
};
#endif


int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2)
{
	int ret = -1;
	int retry = 0;

#ifndef ENABLE_PROC_COMM
	return 0;
#endif

	while (readl(MDM_STATUS) != PCOM_READY) {
		if(retry++ > 1000){
			DISPLAY_MSG("wait PCOM_READY timeout..\n");
                        break;
//			return ret;
		}
		/* XXX check for A9 reset */
	}

	writel(cmd, APP_COMMAND);
	if (data1)
		writel(*data1, APP_DATA1);
	if (data2)
		writel(*data2, APP_DATA2);

	notify_other_proc_comm();
	retry = 0;
	while (readl(APP_COMMAND) != PCOM_CMD_DONE) {
		if(retry++ > 1000){
			DISPLAY_MSG("wait PCOM_CMD_DONE timeout..\n");
			return ret;
		}
		/* XXX check for A9 reset */
	}

	if (readl(APP_STATUS) != PCOM_CMD_FAIL) {
		if (data1)
			*data1 = readl(APP_DATA1);
		if (data2)
			*data2 = readl(APP_DATA2);
		ret = 0;
	}

	return ret;
}

int clock_enable(unsigned id)
{
    return msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &id, 0);
}

int clock_disable(unsigned id)
{
    return msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, 0);
}

int clock_set_rate(unsigned id, unsigned rate)
{
    return msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &rate);
}

int clock_get_rate(unsigned id)
{
    if (msm_proc_comm(PCOM_CLKCTL_RPC_RATE, &id, 0)) {
        return -1;
    } else {
        return (int) id;
    }
}

void reboot(void)
{
//    msm_proc_comm(PCOM_RESET_CHIP, 0, 0);
    msm_proc_comm(PCOM_OEM_RESET_CHIP, 0, 0);
//    for (;;) ;
}

int vreg_enable(unsigned id)
{
    unsigned n = 1;
    return msm_proc_comm(PCOM_VREG_SWITCH, &id, &n);
}

int vreg_disable(unsigned id)
{
    unsigned n = 0;
    return msm_proc_comm(PCOM_VREG_SWITCH, &id, &n);
}

int vreg_set_level(unsigned id, unsigned level)
{
    return msm_proc_comm(PCOM_VREG_SET_LEVEL, &id, &level);
}


int get_battery_level(unsigned *batt_level)
{
   //disabled as of now. Waiting for reply from the vbatt team (APU)
#if 0
   unsigned n = 0;
   return msm_proc_comm(PCOM_GET_BATT_LEVEL, batt_level, &n);
#endif
}

/* checks if the charger is actively charging the battery */
int charger_is_charging(unsigned *is_charging)
{
   unsigned charging_supported = FALSE;
   msm_proc_comm(PCOM_CHG_IS_CHARGING, is_charging, &charging_supported);
   return charging_supported;
}

/* Apps processor calls this API to tell modem processor that a PC USB
 * is connected return true if the USB HOST PC charger charging is
 * supported */
int charger_usb_is_pc_connected(void)
{
   unsigned charging_supported = FALSE;
   unsigned m = 0;
   msm_proc_comm(PCOM_CHG_USB_IS_PC_CONNECTED, &charging_supported, &m);
   return charging_supported;
}

/* Apps processor calls this API to tell modem processor that a USB Wall
 * charger is connected returns true if the USB WALL charger charging is
 * supported */
int charger_usb_is_charger_connected(void)
{
   unsigned charging_supported = FALSE;
   unsigned m = 0;
   msm_proc_comm(PCOM_CHG_USB_IS_CHARGER_CONNECTED, &charging_supported, &m);
   return charging_supported;
}

/* Apps processor calls this API to tell modem processor that a USB cable is
 * disconnected return true is charging is supported in the system */
int charger_usb_disconnected(void)
{
   unsigned charging_supported = FALSE;
   unsigned m = 0;
   msm_proc_comm(PCOM_CHG_USB_IS_DISCONNECTED, &charging_supported, &m);
   return charging_supported;
}

/* current parameter passed is the amount of current that the charger needs
 * to draw from USB */
int charger_usb_i(unsigned current)
{
   unsigned charging_supported = FALSE;
   //msm_proc_comm(PCOM_CHG_USB_I_AVAILABLE, &current, &charging_supported);
   msm_proc_comm(PCOM_CHG_USB_IS_AVAILABLE, &current, &charging_supported);
   return charging_supported;
}

int get_uuid_high(void)
{
   unsigned batt_level = 0;
   unsigned n = 0;
   msm_proc_comm(PCOM_GET_BATT_LEVEL, &batt_level, &n);
   return batt_level;
   //unsigned uuid_high = 0;
   //unsigned m = 0;
   //msm_proc_comm(PCOM_GET_UUID_HIGH, &uuid_high, &m);
   //return m;
}

int get_uuid_low(void)
{
   //unsigned uuid_low = 0;
   //unsigned m = 0;
   //msm_proc_comm(PCOM_GET_UUID_LOW, &uuid_low, &m);
   //return m;
   unsigned batt_level = 0;
   unsigned n = 0;
   msm_proc_comm(PCOM_GET_BATT_LEVEL, &batt_level, &n);
   return batt_level;
}

