/* touch_sw82906.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
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

#ifndef LGE_TOUCH_SW82906_H
#define LGE_TOUCH_SW82906_H

#include <linux/pm_qos.h>

#define __SW82906_STILL_BRING_UP

//#define __SUPPORT_NOTIFY_LCD_EVENT_REG
//#define __SUPPORT_ABT
//#define __SUPPORT_NOTIFY_CALL
//#define __SUPPORT_LONGPRESS

#define PALM_ID					15

/* ic info offset*/
#define tc_version				(0)
//#define tc_product_code		(0x1)
#define tc_product_id1			(0x2)
//#define tc_product_id2		(0x3)
//#define tc_conf_version		(0x4)

#define pt_info_channel			(0x5)
#define pt_info_lot_um			(0x7)
#define pt_info_fpc_type		(0x8)
#define pt_info_pt_date			(0x9)
#define pt_info_pt_time			(0xa)

/* HW resister */
#define INFO_PTR_ADDR			(0x020)	//(0x01B)
#define ENGINE_BUF_ADDR         (0x64D)
#define SYS_BUF_ADDR            (0x64E)


#define SERIAL_SPI_EN			(0xFE4)
#define SERIAL_I2C_EN			(0xFE5)
#define SPI_TATTN_OPT			(0xFF3)
#define SYS_GPIO_FUNC			(0x008)
#define SYS_LDO_CTL				(0x006)
#define SYS_CLK_CTL				(0x003)
#define SYS_PLL_CTL1			(0x007)
#define LDO15_CTL				(0x457)
#define pllosc_sel				(1 << 5)
#define xe_en					(1 << 0)
#define r_ldo15_bias_mute		(3 << 4)
#define ldo_bias				(3 << 3)
#define pll_pd_n				(1 << 1)
#define osc_pd_n				(1 << 0)

#define SPR_CHIP_ID				(0)
#define TC_CMD					(0xC00)
//#define RSVD1					(0xC10)
#define TEST_CMD				(0xC20)
#define ABT_CMD					(0xF20)
//#define RSVD2					(0xC40)
#define CFG_S_SRAM_OFT			(0xCC0)
#define TC_IC_STATUS			(0x600)
#define TC_STATUS				(0x601)
#define ABT_REPORT				(0x602)
//#define RSVD3					(0)
#define CHIP_INFO				(0x642)
#define REG_INFO				(0x64A)
#define PT_INFO					(0x656)
#define TC_STS					(0x666)
#define ABT_STS					(0)
#define TUNE_CODE				(0x6A6)
#define SYS_BUF_OFT		  		(0x106A)
#define ENGINE_BUF_OFT			(0x1482)
#define R_ADDR					(0x7DC)

#define DEBUG_INFO				(0x63E)

/* device control offset */
#define tc_device_ctl			(0x0)
#define tc_interrupt_ctl		(0x1)
#define tc_interrupt_status		(0x2)
#define tc_driving_ctl			(0x3)
//#define tc_flash_dn_ctl		(0x5)

#define command_start_addr		(0xC00)
#define command_end_addr		(0xEFF)

//#define tc_rtc_te_interval_cnt	(56)
//#define config_s_info1			(0x480)
//#define config_s_info2			(0x481)

/* F/W Upgrade */
#define FLASH_PAGE_OFFSET		(0x200)
#define FLASH_PAGE_SIZE			(2<<10)
#define FLASH_MAX_RW_SIZE		(1<<10)

#define GDMA_SADDR				0x085	//0x056
#define GDMA_CTL				0x087	//0x058
#define GDMA_CTL_READONLY_EN	BIT(17)
#define GDMA_CTL_GDMA_EN		BIT(26)
#define GDMA_START				0x088	//0x059
#define GDMA_CRC_RESULT			0x08C	//0x05D
#define GDMA_CRC_PASS			0x08D	//0x05E

#define FC_CTRL					0x09A	//0x06B
#define FC_STAT					0x09B	//0x06C
#define FC_ADDR					0x09C	//0x06D
#define SPI_FLASH_STATUS		0xFE2

#define SYS_OSC_CTL				(0x005)
#define AVERAGE_TRIM_VAL		(0x3C)

#define __FC_CTRL_PAGE_ERASE	BIT(0)
#define __FC_CTRL_MASS_ERASE	BIT(1)
#define __FC_CTRL_WR_EN			BIT(2)

#define FC_CTRL_PAGE_ERASE		(__FC_CTRL_PAGE_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_MASS_ERASE		(__FC_CTRL_MASS_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_WR_EN			(__FC_CTRL_WR_EN)

#define FC_ERASE_WAIT_CNT		200
#define FC_ERASE_WAIT_TIME		5
/* F/W Upgrade END */

#define FAIL_REASON_MAX_CNT		8

/*
 * Engine control resister Offset
 * base:r_abt_cmd_addr(0xF20)
 */
//#define SENSELESS_PIXEL						(0x48)	// 72 pixel == 5mm
#define SENSELESS_PIXEL						(0x73)  // 115 pixel

/* KNOCK_ON  */
#define KNOCK_ON_INDEX_AND_ENABLE			(0x10)
#define KNOCK_ON_TOTAL_TAP_COUNT			(0x11)
#define KNOCK_ON_INTER_MIN_INTERTAP			(0x12)
#define KNOCK_ON_INTER_MAX_INTERTAP			(0x13)
#define KNOCK_ON_INNER_TOUCH_SLOP			(0x14)
#define KNOCK_ON_INTER_TAP_DISTANCE			(0x15)
#define KNOCK_ON_INTERRUPT_DELAY_TIME		(0x16)
#define KNOCK_ON_ACTIVE_AREA_X1				(0x17)
#define KNOCK_ON_ACTIVE_AREA_Y1				(0x18)
#define KNOCK_ON_ACTIVE_AREA_X2				(0x19)
#define KNOCK_ON_ACTIVE_AREA_Y2				(0x1A)

/* Swipe U, R, D, L - LG_PAY */
#define SWIPE_INDEX_AND_ENABLE				(0x1B)
#define SWIPE_DIST_THRESHOLD				(0x1C)
#define SWIPE_RATIO_THRESHOLD				(0x1D)
#define SWIPE_TIME_MIN_HORIZONTAL			(0x1E)
#define SWIPE_TIME_MIN_VERTICAL				(0x1F)
#define SWIPE_TIME_MAX_HORIZONTAL			(0x20)
#define SWIPE_TIME_MAX_VERTIAL				(0x21)
#define SWIPE_ACTIVE_AREA_HORIZONTAL_X1		(0x22)
#define SWIPE_ACTIVE_AREA_HORIZONTAL_Y1		(0x23)
#define SWIPE_ACTIVE_AREA_HORIZONTAL_X2		(0x24)
#define SWIPE_ACTIVE_AREA_HORIZONTAL_Y2		(0x25)
#define SWIPE_ACTIVE_AREA_VERTICAL_X1		(0x26)
#define SWIPE_ACTIVE_AREA_VERTICAL_Y1		(0x27)
#define SWIPE_ACTIVE_AREA_VERTICAL_X2		(0x28)
#define SWIPE_ACTIVE_AREA_VERTICAL_Y2		(0x29)
#define SWIPE_START_AREA_HORIZONTAL_X1		(0x2A)
#define SWIPE_START_AREA_HORIZONTAL_Y1		(0x2B)
#define SWIPE_START_AREA_HORIZONTAL_X2		(0X2C)
#define SWIPE_START_AREA_HORIZONTAL_Y2		(0x2D)
#define SWIPE_START_AREA_VERTICAL_X1		(0x2E)
#define SWIPE_START_AREA_VERTICAL_Y1		(0x2F)
#define SWIPE_START_AREA_VERTICAL_X2		(0X30)
#define SWIPE_START_AREA_VERTICAL_Y2		(0x31)
#define SWIPE_WRONG_DIRECTION_THD			(0x32)
#define SWIPE_INIT_RATIO_CHK_DIST			(0x33)
#define SWIPE_INIT_RATIO_THD				(0x34)

/* Swipe2 R, L Only - QUICK_TOOL */
#define SWIPE2_INDEX_AND_ENABLE				(0x35)
#define SWIPE2_DIST_THRESHOLD				(0x36)
#define SWIPE2_RATIO_THRESHOLD				(0x37)
#define SWIPE2_TIME_MIN_HORIZONTAL			(0x38)
#define SWIPE2_TIME_MAX_HORIZONTAL			(0x39)
#define SWIPE2_ACTIVE_AREA_HORIZONTAL_X1	(0x3A)
#define SWIPE2_ACTIVE_AREA_HORIZONTAL_Y1	(0x3B)
#define SWIPE2_ACTIVE_AREA_HORIZONTAL_X2	(0x3C)
#define SWIPE2_ACTIVE_AREA_HORIZONTAL_Y2	(0x3D)
#define SWIPE2_START_AREA_HORIZONTAL_X1		(0x3E)
#define SWIPE2_START_AREA_HORIZONTAL_Y1		(0x3F)
#define SWIPE2_START_AREA_HORIZONTAL_X2		(0X40)
#define SWIPE2_START_AREA_HORIZONTAL_Y2		(0x41)
#define SWIPE2_WRONG_DIRECTION_THD			(0x42)
#define SWIPE2_INIT_RATIO_CHK_DIST			(0x43)
#define SWIPE2_INIT_RATIO_THD				(0x44)

/* ABS */
#define LPWG_ABS_INDEX_AND_ENABLE			(0x45)
#define LPWG_ABS_ACTIE_AREA_START			(0x46)
#define LPWG_ABS_ACTIE_AREA_END				(0x47)

/* LongPress */
#define LONG_PRESS_INDEX_AND_ENABLE			(0x48)
#define LONG_PRESS_ACT_AREA_START			(0x49)
#define LONG_PRESS_ACT_AREA_END				(0x4A)
#define LONG_PRESS_SLOPE					(0x4B)
#define LONG_PRESS_TIME						(0x4C)
#define LONG_PRESS_CONTACT_SIZE				(0x4D)

/* 1Tap */
#define ONE_TAP_INDEX_AND_ENABLE			(0x4E)
#define ONE_TAP_ACTIVE_AREA_START			(0x4F)
#define ONE_TAP_ACTIVE_AREA_END				(0x50)
#define ONE_TAP_TOUCH_SLOPE					(0x51)
#define ONE_TAP_PRESS_TIME					(0x52)
#define ONE_TAP_INT_DELAY_TIME				(0x53)

/* Fail Reason */
#define LPWG_FAILREASON_REPORT_CTRL2		(0x54)
#define LPWG_FAILREASON_COUNT2				(0x55)
#define ONETAP_FAILREASON_BUF				(0x56)

#define LPWG_FAILREASON_REPORT_CTRL1		(0x58)
#define LPWG_FAILREASON_COUNT1				(0x59)
#define KNOCK_ON_FRONT_FAILREASON_BUF		(0x5A)
#define KNOCK_ON_REAR_FAILREASON_BUF		(0x5C)
#define SWIPE_FAILREASON_BUF				(0x5E)
#define LONGPRESS_FAILREASON_BUF			(0x60)

/* QuickCover */
#define COVER_SENSITIVITY					(0x62)
#define COVER_ACTIVE_AREA_START_XY			(0x63)
#define COVER_ACTIVE_AREA_END_XY			(0x64)

/* SPECIAL Resister */
#define SPECIAL_CHARGER_INFO				(0x65)
#define GLOVE_TOUCH_CTRL					(0x66)
#define GRAB_TOUCH_CTRL						(0x67)
#define SPECIAL_IME_STATUS					(0x68)
#define SPECIAL_CALL_INFO					(0x69)
#define SPECIAL_SENSITIVE_INFO				(0x6A)
#define SPECIAL_TEMPERATURE_INFO			(0x90)

#define ROTATION_ANGLE						(0x84)
#define PORTRAIT_VERTICAL_A_B_WIDTH			(0x85)
#define PORTRAIT_VERTICAL_C_WIDTH_HEIGHT	(0x86)
#define LANDSCAPE_VERTICAL_A_B_WIDTH		(0x87)
#define LANDSCAPE_VERTICAL_C_WIDTH_HEIGH	(0x88)
#define LANDSCAPE_HORIZONTAL_A_B_HEIGHT		(0x89)
#define GRIP_EXCEPTION_ENABLE				(0x62)
#define GRIP_EXCEPTION_START				(0x63)
#define GRIP_EXCEPTION_END					(0x64)




#if 1
#define R_HEADER_SIZE_SPI			(6)
#define W_HEADER_SIZE_SPI			(6)
#else
#define R_HEADER_SIZE_SPI			(2)
#define W_HEADER_SIZE_SPI			(4)
#endif

#define R_HEADER_SIZE_I2C			(0)
#define W_HEADER_SIZE_I2C			(2)

/* SPR control */
//#define SPI_RST_CTL				0xFE0
//#define SPI_CLK_CTL				0xFE1
#define SPI_OSC_CTL					(0xFE1) // 0xFE2


#define CONNECT_NONE				(0x00)
#define CONNECT_USB					(0x01)
#define CONNECT_WIRELESS			(0x10)

#define CFG_MAGIC_CODE				0xCACACACA
#define CFG_CHIP_ID					42000

#define CHIP_POW_C_CONF				10
#define	CHIP_POW_S_CONF				10

#define CHIP_NUM_C_CONF				0
#define CHIP_MIN_S_CONF				1
#define CHIP_MAX_S_CONF				10

#define CFG_C_SIZE					(CHIP_NUM_C_CONF<<CHIP_POW_C_CONF)
#define CFG_S_SIZE					(1<<10)

#define CRC_FIXED_VALUE				0x800D800D

/* Interrupt Status for check_status func */
/* (1<<1)|(1<<2) */
#define IC_STATUS_NORMAL_MASK		0x00000006
#define IC_STATUS_GLOBAL_RESET_BIT	0x00000000	/* NULL */
/* (1<<3) | (1<<5) */
#define IC_STATUS_HW_RESET_BIT		0x00000028
#define IC_STATUS_SW_RESET_BIT		0x00000000	/* NULL */
/* (1<<1) */
#define IC_STATUS_FW_UPGRADE_BIT	0x00000002
/* (1<<2) */
#define IC_STATUS_LOGGING_BIT		0x00000004

/* (1<<5)|(1<<6)|(1<<15)|(1<<20)|(1<<22) */
#define TC_STATUS_NORMAL_MASK		0x00508060
#define TC_STATUS_GLOBAL_RESET_BIT	0x00000000	/* NULL */
/* (1<<6)|(1<<9)|(1<<10) */
#define TC_STATUS_HW_RESET_BIT		0x00000640
#define TC_STATUS_SW_RESET_BIT		0x00000000	/* NULL */
#define TC_STATUS_FW_UPGRADE_BIT	0x00000000	/* NULL */
/* (1<<5)|(1<<6)|(1<<9)|(1<<10)|(1<<15)|(1<<20)|(1<<22) */
#define TC_STATUS_LOGGING_BIT		0x00508660

#define STATUS_NORMAL_MASK			(((u64)IC_STATUS_NORMAL_MASK << 32) | (u64)TC_STATUS_NORMAL_MASK)
#define STATUS_GLOBAL_RESET_BIT		(((u64)IC_STATUS_GLOBAL_RESET_BIT << 32) | (u64)TC_STATUS_GLOBAL_RESET_BIT)
#define STATUS_HW_RESET_BIT			(((u64)IC_STATUS_HW_RESET_BIT << 32) | (u64)TC_STATUS_HW_RESET_BIT)
#define STATUS_SW_RESET_BIT			(((u64)IC_STATUS_SW_RESET_BIT << 32) | (u64)TC_STATUS_SW_RESET_BIT)
#define STATUS_FW_UPGRADE_BIT		(((u64)IC_STATUS_FW_UPGRADE_BIT << 32) | (u64)TC_STATUS_FW_UPGRADE_BIT)
#define STATUS_LOGGING_BIT			(((u64)IC_STATUS_LOGGING_BIT << 32) | (u64)TC_STATUS_LOGGING_BIT)

#define GRAB_TOUCH_CTRL_BIT								0x00000001
#define GRAB_NOTI_CTRL_BIT								0x00010000

/* ic bus test */
#define BUS_TEST_REG									(0x021)
#define IC_TEST_ADDR_NOT_VALID							0x8000

/* interrupt type */
#define INTR_TYPE_BOOT_UP_DONE							0x01
#define INTR_TYPE_INIT_COMPLETE							0x02
#define INTR_TYPE_ABNORMAL_ERROR_REPORT					0x03
#define INTR_TYPE_DEBUG_REPORT							0x04
#define INTR_TYPE_REPORT_PACKET							0x05

/* seperate control in each split display area */
#define SPLIT_DISPLAY_AREA_MOVING						0XFA0
#define SPLIT_DISPLAY_AREA_0							0XFA1
#define SPLIT_DISPLAY_AREA_INTERSPACE_0_1				0XFA2
#define SPLIT_DISPLAY_AREA_1							0XFA3

#define SPLIT_DISPLAY_AREA_ACTIVATION_DISABLE			0
#define SPLIT_DISPLAY_AREA_ACTIVATION_ENABLE			1

#define SPLIT_DISPLAY_AREA_MODE_LPWG					0	// 1 (before v0.08)
#define SPLIT_DISPLAY_AREA_MODE_ABS			    		1	// 2 (before v0.08)

#define SPLIT_DISPLAY_AREA_ENABLE_SHIFT_BIT				(31)
#define SPLIT_DISPLAY_AREA_PEN_ENABLE_SHIFT_BIT			(25)
#define SPLIT_DISPLAY_AREA_FINGER_ENABLE_SHIFT_BIT		(24)
#define SPLIT_DISPLAY_AREA_MAGIC_SHIFT_BIT				(20)
#define SPLIT_DISPLAY_AREA_DISPLAY_MODE_SHIFT_BIT		(16)
#define SPLIT_DISPLAY_AREA_RESOLUTION_SHIFT_BIT			(0)

#define SPLIT_DISPLAY_AREA_ENABLE_COMPARE_BIT			(0x1 << 31)
#define SPLIT_DISPLAY_AREA_PEN_ENABLE_COMPARE_BIT		(0x1 << 25)
#define SPLIT_DISPLAY_AREA_FINGER_ENABLE_COMPARE_BIT	(0x1 << 24)
#define SPLIT_DISPLAY_AREA_MAGIC_COMPARE_BIT			(0x1 << 20)
#define SPLIT_DISPLAY_AREA_DISPLAY_MODE_COMPARE_BIT		(0x3 << 16)
#define SPLIT_DISPLAY_AREA_RESOLUTION_COMPARE_BIT		(0xFFFF)

struct sw82906_display_area_split_info {
	u32 area_activation:1;
	u32 reserve1:5;
	u32 pen_en:1;
	u32 finger_en:1;
	u32 reserve2:3;
	u32 magic:1;
	u32 reserve3:2;
	u32 display_mode:2; //0:LPWG, 1:ABS
	int resolustion:16;
} __packed;

struct sw82906_lg_pay_swipe_info {
	u32 enable;
	u8 distance[4];
	u8 ratio_thres[4];
	u16 min_time[4];
	u16 max_time[4];
	u16 area_hori[8];
	u16 area_verti[8];
	u16 start_hori[8];
	u16 start_verti[8];
	u8 wrong_dir_thres[4];
	u8 init_ratio_chk_dist[4];
	u8 init_ratio_thres[4];
} __packed;

struct sw82906_quick_tool_swipe_info {
	u32 enable;
	u8 distance[4];
	u8 ratio_thres[4];
	u16 min_time[2];
	u16 max_time[2];
	u16 area[8];
	u16 start[8];
	u8 wrong_dir_thres[4];
	u8 init_ratio_chk_dist[4];
	u8 init_ratio_thres[4];
} __packed;

enum {
	ALL_OFF = 0,
	ALL_ON,
	FRONT_ON_BACK_OFF,
	FRONT_OFF_BACK_ON,
};

enum {
	FUNC_OFF = 0,
	FUNC_ON,
};

enum {
	RESUME_CTRL_NONE = 0,
	RESUME_CTRL_POWER,
	RESUME_CTRL_RESET,
};

enum {
	U3_MODE_2_ONLY = 0,
	U3_MODE_1_2_SWING,
	U3_MODE_2_ONLY_SYNC,
	U3_MODE_1_2_SWING_SYNC,
};

enum {
	SW_RESET = 0,
	HW_RESET_ASYNC,
	HW_RESET_SYNC,
	SW_RESET_CODE_DUMP,
	HW_RESET_POWER,
	HW_RESET_ONLY,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	IRQ_TYPE_ABS_MODE = 0,
	IRQ_TYPE_KNOCK_ON = 1,
	IRQ_TYPE_KNOCK_CODE = 2,
	IRQ_TYPE_LG_PAY_SWIPE_LEFT = 3,
	IRQ_TYPE_LG_PAY_SWIPE_RIGHT = 4,
	IRQ_TYPE_LG_PAY_SWIPE_UP = 5,
	IRQ_TYPE_SWIPE_DOWN = 6,
	IRQ_TYPE_LONG_PRESS = 7,
	IRQ_TYPE_QUICK_TOOL_SWIPE_LEFT = 8,
	IRQ_TYPE_QUICK_TOOL_SWIPE_RIGHT = 9,
	IRQ_TYPE_ONE_TAP = 50,
	IRQ_TYPE_LONG_PRESS_DOWN = 51,
	IRQ_TYPE_LONG_PRESS_UP = 52,
	IRQ_TYPE_CUSTOM_DEBUG = 200,
	IRQ_TYPE_KNOCK_OVERTAP = 201,
	IRQ_TYPE_PEN_WAKEUP = 101,
	IRQ_TYPE_PEN_WAKEUP_BTN = 102,
	IRQ_TYPE_PEN_WAKEUP_BTN_B = 103,
};

enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_STOP,
	LCD_MODE_UNKNOWN,
	LCD_MODE_NUM,
};

enum {
	SW82906_SWIPE_L = 0,
	SW82906_SWIPE_R = 1,
	SW82906_SWIPE_U = 2,
	SW82906_SWIPE_D = 3,
	SW82906_SWIPE_NUM = 4,
};

enum {
	SW82906_SWIPE2_L = 0,
	SW82906_SWIPE2_R = 1,
	SW82906_SWIPE2_NUM = 2,
};

enum {
	LPWG_FAILREASON_DISABLE = 0,
	LPWG_FAILREASON_ENABLE,
};

enum {
	LPWG_DEBUG_KNOCK_ON_FRONT = 0,
	LPWG_DEBUG_KNOCK_ON_REAR,
	LPWG_DEBUG_SWIPE,
	LPWG_DEBUG_LONGPRESS,
	LPWG_DEBUG_ONETAP,
	LPWG_DEBUG_NUM,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	LOG_WRITE_DONE = 0,
	DO_WRITE_LOG,
};

enum {
	UPGRADE_NOT_INITIALIZED = -1,
	UPGRADE_NO_NEED = 0,
	UPGRADE_NEED = 1,
	UPGRADE_DONE = 2,
	UPGRADE_FORCE = 3,
};

enum {
	FLIP_BACK = -1,
	FLIP_NONE = 0,
	FLIP_FRONT = 1,
};

//#define __SW82906_SUPPORT_CFG

#define FLASH_TPROG				0x65
#define serial_data_offset		(0x045)
#define MAX_FLASH_SIZE			(128 * 1024)

#define BDMA_TRANS_SIZE			(0x1000)
#define DATASRAM_ADDR			(0x20000000)

#if IS_ENABLED(__SW82906_SUPPORT_CFG)
typedef union
{
	struct {
		unsigned	common_cfg_size : 16;
		unsigned	specific_cfg_size : 16;
	} b;
	uint32_t w;
} t_cfg_size;

typedef union
{
	struct {
		unsigned	chip_rev : 8;
		unsigned	model_id : 8;
		unsigned	reserved : 16;
	} b;
	uint32_t w;
} t_cfg_specific_info1;


typedef union
{
	struct {
		unsigned	lcm_id : 8;
		unsigned	fpcb_id : 8;
		unsigned	lot_id : 8;
		unsigned	reserved : 8;
	} b;
	uint32_t w;
} t_cfg_specific_info2;

typedef struct
{
	uint32_t                        cfg_magic_code;
	uint32_t                        cfg_chip_id;
	uint32_t                        cfg_specific_index;
	t_cfg_size                      cfg_size;
	t_cfg_specific_info1            cfg_specific_info1;
	t_cfg_specific_info2            cfg_specific_info2;
	uint32_t                        cfg_header_reserved1;
	uint32_t                        cfg_header_reserved2;
} CFG_S_HEADER_CONTROL_TypeDef;

union cfg_size {
	struct {
		u32 common_size:16;
		u32 specific_size:16;
	} b;
	u32 w;
};

union s_cfg_info_1 {
	struct {
		u32 chip_rev:8;
		u32 model_id:8;
		u32 lcm_id:8;
		u32 fpc_id:8;
	} b;
	u32 w;
};

union s_cfg_info_2 {
	struct {
		u32 lot_id:8;
		u32 rsvd:24;
	} b;
	u32 w;
};

struct s_cfg_head {
	union s_cfg_info_1	info_1;
	union s_cfg_info_2	info_2;
	u32 version;
	u32 model_name;
};

struct cfg_head {
	u32 magic_code;
	u32 chip_id;
	u32 s_index;
	union cfg_size c_size;
	struct s_cfg_head s_cfg_head;
	u32 rsvd1;
	u32 rsvd2;
	u32 rsvd3;
	u32 rsvd4;
};
#endif	/* __SW82906_SUPPORT_CFG */

#define BDMA_SADDR						0xA1	//0x72
#define BDMA_DADDR						0xA3	//0x73
#define BDMA_CAL_OP						0xA5
#define BDMA_CTL						0xA6	//0x74
#define BDMA_START						0xA7	//0x75
#define BDMA_STS						0xA9	//0x77

#define BDMA_CTL_BDMA_EN				(BIT(16) | (2<<23) | (2<<25) | (2<<27))
#define BDMA_CTL_BDMA_BST_EN			(0x001E0000)
#define BDMA_STS_TR_BUSY				(0x00000040)
#define BDMA_CAL_OP_CTRL				(3 | (1024<<2) | (1024<<13))

/* SPR control */

/* Firmware control */
#define spr_rst_ctl				(0x004)	//(0x006)
#define SYS_RST_CTL				spr_rst_ctl

#define spr_boot_ctl			(0x00F)
#define fw_boot_code_addr		(0x01A)	//(0x044)
#define spr_sram_ctl			(0x010)
#define spr_flash_offset			(0x03C)	//(0x07D)
#define spr_data_offset			(0x082)
#define rst_cnt_addr			(0xFF5)
#define tc_flash_dn_sts			(0)

#define flash_access_addr		(0xFD8)	//(0xFD0)
#define data_access_addr		(0xFD1)

#define MAX_RW_SIZE				(60 * 1024)
#define FLASH_SIZE				(128 * 1024)
#define FLASH_CONF_SIZE			(1 * 1024)

#define FLASH_KEY_CODE_CMD		0xDFC1
#define FLASH_KEY_CONF_CMD		0xE87B
#define FLASH_BOOTCHK_VALUE		0x0A0A0000
#define FW_BOOT_LOADER_INIT		0x74696E69
#define FW_BOOT_LOADER_CODE		0x544F4F42
#define FLASH_CODE_DNCHK_VALUE	0x42
#define FLASH_CONF_DNCHK_VALUE	0x8C
#define FLIP_INFO_ADDR		0xF86

enum {
	E_FW_CODE_SIZE_ERR = 1,
	E_FW_CODE_ONLY_VALID = 2,
	E_FW_CODE_AND_CFG_VALID = 3,
	E_FW_CODE_CFG_ERR = 4
};

/* report packet */
struct sw82906_touch_data {
	u32 track_id:5;
	u32 tool_type:3;
	u32 angle:8;
	u32 event:2;
	u32 x:14;
	u32 y:14;
	u32 pressure:8;
	u32 reserve1:10;
	u32 reserve2:4;
	u32 width_major:14;
	u32 width_minor:14;
} __packed;

struct sw82906_pen_data {
	u16 in_range:1; 	//0:None, 1:Tip & Hover
	u16 contact:1;		//0:None, 1:Tip
	u16 swit1:1;
	u16 swit2:1;
	u16 pressure:12;	//0 ~ 0xFFF
	//
	u16 batt:2;
	u16 rsvd0:14;
	//
	u32 id_upper:24;
	u32 rsvd1:8;
	//
	u32 id_lower:28;
	u32 rsvd2:4;
	//
	u16 x;
	u16 y;
	//
	u16 tilt;
	u16 azimuth;
	//
	u32 rsvd4;
} __packed;

struct sw82906_touch_info {
	u32 ic_status;
	u32 tc_status;
	//
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 pen_cnt:1;
	u32 rsvd1:7;
	u32 curr_mode:3;
	u32 rsvd2:4;
	u32 palm_bit:1;
	struct sw82906_pen_data pen;
	struct sw82906_touch_data data[10];
	/* debug info */
} __packed;

struct sw82906_version_bin {
	u8 major : 4;
	u8 build : 4;
	u8 minor;
};

struct sw82906_version {
	u8 minor;
	u8 major:4;
	u8 build:4;
	u8 chip_id;
	u8 protocol_ver:4;
	u8 reverved:4;
};

struct sw82906_pt_info {
	u8 channel;
	u32 reserved_1:24;
	u8 chip_rev;
	u32 reserved_2:24;
	u8 sensor_ver;
	u32 reserved_3:24;
	u8 fpc_ver;
	u32 reserved_4:24;
	u8 pt_date_year;
	u8 pt_date_month;
	u8 pt_date_day;
	u8 reserved_5;
	u8 pt_time_hour;
	u8 pt_time_min;
	u8 pt_time_sec;
	u8 reserved_6;
} __packed;

struct sw82906_ic_info {
	struct sw82906_version version;
	u8 product_id[8+4];
	struct sw82906_pt_info pt_info;
	struct sw82906_version_bin img_version;
};

struct sw82906_chip_info {
	u32 r_version;
	u32 r_product_code;
	u32 r_product_id1;
	u32 r_product_id2;
	u32 r_conf_dn_index;
	u32 r_conf_version;
};

struct sw82906_reg_info {
    u16 sys_buf_addr;
    u16 engine_buf_addr;
};

struct project_param {
	u8 wa_trim_wr;
	u8 wa_suspend_reset_ctl;
	u8 dfc_resume_power_ctl;
	u8 sys_driving_ctrl;
	u32 sys_tc_stop_delay;
	u32 sys_tc_driving_delay;
	u8 used_mode;
	u8 reg_debug;
	u8 dynamic_reg;
	u32 flash_fw_size;
	u8 dfc_bus_test;
	u8 esd_recovery_when_rw;
	u8 tcl_off_via_mipi;
	u8 touch_power_control_en;
	u8 flex_report;
	u8 need_serial_control;
};

struct sw82906_touch_debug_info {
	u32 info[3];
	u32 type:24;
	u32 length:8;
} __packed;

struct sw82906_fb_blank {
	int prev;
	int curr;
};

struct sw82906_active_area {
	s16 x1;
	s16 y1;
	s16 x2;
	s16 y2;
} __packed;

struct sw82906_lpwg_abs_buf {
	u32 enable;
	u16 start_x;
	u16 start_y;
	u16 end_x;
	u16 end_y;
} __packed;

struct sw82906_lpwg_abs_ctrl {
	bool enable;
	struct sw82906_active_area area;
	struct sw82906_active_area border;
	s16 offset_y;
};
struct sw82906_lpwg_longpress_ctrl {
	bool enable;
	struct sw82906_active_area area;
	struct sw82906_active_area border;
	int slop;
	int press_time;
	int contact_size;
};

struct sw82906_lpwg_onetap_ctrl {
	bool enable;
	struct sw82906_active_area area;
	struct sw82906_active_area border;
	int slop;
	int press_time;
	int delay_time;
};

struct sw82906_data {
	struct device *dev;
	struct kobject kobj;

	struct project_param p_param;

	struct sw82906_touch_info info;
	struct sw82906_touch_debug_info debug_info;
	struct sw82906_ic_info ic_info;
	struct sw82906_chip_info chip_info;
	struct sw82906_reg_info reg_info;
	struct workqueue_struct *wq_log;
	struct sw82906_fb_blank fb_blank;

	struct sw82906_lpwg_abs_ctrl lpwg_abs;
	struct sw82906_lpwg_longpress_ctrl lpwg_longpress;
	struct sw82906_lpwg_onetap_ctrl lpwg_onetap;

	bool split_display_area_moving;
	struct sw82906_display_area_split_info display_area_split_info[DISPLAY_AREA_ID_MAX];
	struct sw82906_display_area_split_info display_area_interspace_split_info[DISPLAY_AREA_INTERSPACE_ID_MAX];

	void *prd;

	u8 lcd_mode;
	u8 prev_lcd_mode;
	u8 driving_mode;
	u8 u3fake;

	struct mutex io_lock;
	struct delayed_work fb_notify_work;
	u32 charger;
	u32 earjack;
	u32 frame_cnt;
	u8 intr_type;
	atomic_t block_watch_cfg;
	atomic_t init;
	struct pm_qos_request pm_qos_req;
	u32 q_sensitivity;
	char te_test_log[64];
	int te_ret;
	u8 te_write_log;
	u8 lpwg_failreason_ctrl;
	u32 lpwg_failreason_data;
	int check_fwup;

	u8 err_cnt;
	u8 boot_err_cnt;

	int pen_status;
	int flip_info;
	int thermal_noti;
};

#define KNOCK_ON_MAX_NUM					2
#define SWIPE_MAX_NUM				2
#define KNOCK_ON_DEBUG_MAX_NUM			16
#define SWIPE_DEBUG_MAX_NUM			8
#define DISTANCE_INTER_TAP			(0x1 << 1) /* 2 */
#define DISTANCE_TOUCHSLOP			(0x1 << 2) /* 4 */
#define TIMEOUT_INTER_TAP_LONG		(0x1 << 3) /* 8 */
#define MULTI_FINGER				(0x1 << 4) /* 16 */
#define DELAY_TIME					(0x1 << 5) /* 32 */
#define TIMEOUT_INTER_TAP_SHORT		(0x1 << 6) /* 64 */
#define PALM_STATE					(0x1 << 7) /* 128 */
#define TAP_TIMEOVER				(0x1 << 8) /* 256 */
#define KNOCK_ON_DEBUG_ALL (DISTANCE_INTER_TAP | DISTANCE_TOUCHSLOP |\
		TIMEOUT_INTER_TAP_LONG | MULTI_FINGER | DELAY_TIME |\
		TIMEOUT_INTER_TAP_SHORT | PALM_STATE | TAP_TIMEOVER)

extern int dlkm_sw82906_reg_read(struct device *dev, u16 addr, void *data, int size);
extern int dlkm_sw82906_reg_write(struct device *dev, u16 addr, void *data, int size);
extern int dlkm_sw82906_ic_info(struct device *dev);
extern int dlkm_sw82906_te_info(struct device *dev, char *buf);
extern int dlkm_sw82906_tc_driving(struct device *dev, int mode);
extern int dlkm_sw82906_irq_abs(struct device *dev);
extern int dlkm_sw82906_irq_abs_data(struct device *dev);
extern int dlkm_sw82906_irq_lpwg(struct device *dev);
extern int dlkm_sw82906_irq_handler(struct device *dev);
extern int dlkm_sw82906_check_status(struct device *dev);
extern int dlkm_sw82906_debug_info(struct device *dev);
extern int dlkm_sw82906_sleep_ctrl(struct device *dev, int new_status);
/*
extern int sw82906_chip_info_load(struct device *dev);
*/
extern int dlkm_sw82906_reset_ctrl(struct device *dev, int ctrl);
#if !IS_ENABLED (CONFIG_LGE_TOUCH_DLKM)
extern bool is_ddic_name(char *ddic_name);
#endif
#if IS_ENABLED(CONFIG_LGE_TOUCH_CORE_QCT)
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD) || IS_ENABLED(CONFIG_LGE_TOUCH_PANEL_GLOBAL_RESET)
extern void lge_mdss_report_panel_dead(void);
#endif
#endif
#if IS_ENABLED(CONFIG_LGE_TOUCH_CORE_MTK)
extern void mtkfb_esd_recovery(void);
#endif

static inline struct sw82906_data *to_sw82906_data(struct device *dev)
{
	return (struct sw82906_data *)touch_get_device(to_touch_core(dev));
}

static inline struct sw82906_data *to_sw82906_data_from_kobj(struct kobject *kobj)
{
	return (struct sw82906_data *)container_of(kobj,
			struct sw82906_data, kobj);
}

static inline u32 dlkm_sw82906_tc_sts_irq_type(int status)
{
	return ((status >> 16) & 0x0F);
}

static inline u32 dlkm_sw82906_tc_sts_running_sts(int status)
{
	return (status & 0x1F);
}

static inline int dlkm_sw82906_read_value(struct device *dev,
		u16 addr, u32 *value)
{
	return dlkm_sw82906_reg_read(dev, addr, value, sizeof(*value));
}

static inline int dlkm_sw82906_write_value(struct device *dev,
		u16 addr, u32 value)
{
	return dlkm_sw82906_reg_write(dev, addr, &value, sizeof(value));
}

void dlkm_sw82906_irq_runtime_engine_debug(struct device *dev);
#if IS_ENABLED(CONFIG_LGE_TOUCH_COMMON_CONTROL)
/* To Do */
static char *sw82906_cmd_lists[MAX_CMD_COUNTS][2] = {
	[0] = {"flip_info", "[en:1 dis:0]"},
	{NULL},
};
enum {
	FLIP_INFO,
};
#endif
#endif /* LGE_TOUCH_SW82906_H */

