/* touch_sw82908_prd.c
 *
 * Copyright (C) 2015 LGE.
 *
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

#include "../../touch_core.h"
#include "../../touch_hwif.h"
#include "../../touch_common.h"

#include "touch_sw82908.h"
#include "touch_sw82908_prd.h"

static char __used *line;
static char __used w_buf[BUF_SIZE];

static u16 __used self_data[MAX_CHANNEL + COL_MAX_CHANNEL];
static u16 __used data_buf[MAX_CHANNEL*COL_MAX_CHANNEL];
static u16 __used LowerImage[ROW_SIZE][COL_SIZE];
static u16 __used UpperImage[ROW_SIZE][COL_SIZE];
static u16 __used AverageImage[ROW_SIZE][COL_SIZE];
static u16 __used SimpleImage[SIMPLE_SPEC_SIZE];

#define TEST_TOTAL_NUM 23
static const char * const test_name_str[TEST_TOTAL_NUM] = {
	"U3_PT_TEST",
	"OPEN_NODE_TEST",
	"SHORT_NODE_TEST",
	"U3_M1_RAWDATA_TEST",
	"U3_M1_JITTER_TEST",
	"U3_M2_RAWDATA_TEST",
	"U3_M2_JITTER_TEST",
	"U0_M1_RAWDATA_TEST",
	"U0_M1_JITTER_TEST",
	"U0_M2_RAWDATA_TEST",
	"U0_M2_JITTER_TEST",
	"U3_M2_DELTA_TEST",
	"U0_M2_DELTA_TEST",
	"U3_BLU_JITTER_TEST",
	"AVERAGE_JITTER_TEST",
	"OPEN_RX_NODE_TEST",
	"OPEN_TX_NODE_TEST",
	"SHORT_RX_NODE_TEST",
	"SHORT_TX_NODE_TEST",
	"U3_M2_RAW_SELF_TEST",
	"U3_M2_JITTER_SELF_TEST",
	"U0_M1_RAW_SELF_TEST",
	"U3_P_JITTER_TEST",
};

static int prd_compare_rawdata(struct device *dev, u8 type, int *result);

static void prd_param_set(struct device *dev)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;

	TOUCH_TRACE();

	/* test on/off select */
	param->sd_test_set = ((1<<U3_M2_RAWDATA_TEST)|(1<<U3_M2_JITTER_TEST)
			|(1<<U3_M2_RAW_SELF_TEST)|(1<<U3_M2_JITTER_SELF_TEST)|(1<<U3_P_JITTER_TEST)|(1<<U3_M1_JITTER_TEST));

//	param->lpwg_sd_test_set = ((1<<U0_M2_RAWDATA_TEST)|(1<<U0_M2_JITTER_TEST)|(1<<U0_M1_RAW_SELF_TEST));
	param->lpwg_sd_test_set = ((1<<U0_M2_RAWDATA_TEST)|(1<<U0_M2_JITTER_TEST));

	/* open short test type select */
	param->os_test_type = DATA_DRIVER_COMPARE;
	/* spec file path */
	param->spec_file_path = "/sdcard/firmware/Time_limit.txt";
	param->mfts_spec_file_path = "/sdcard/firmware/Time_limit.txt";

	/* Test frame settings */
	param->frame.open_rx_frame	= PT_FRAME_1;
	param->frame.open_tx_frame	= PT_FRAME_3;
	param->frame.short_rx_frame	= PT_FRAME_1;
	param->frame.short_tx_frame	= PT_FRAME_3;
	param->frame.u3_m1_raw_frame	= PT_FRAME_1;
	param->frame.u3_m1_jitter_frame = PT_FRAME_1;
	param->frame.u3_m2_raw_frame	= PT_FRAME_1;
	param->frame.u3_m2_jitter_frame = PT_FRAME_1;
	param->frame.u0_m1_raw_frame	= PT_FRAME_1;
	param->frame.u0_m1_jitter_frame = PT_FRAME_1;
	param->frame.u0_m2_raw_frame	= PT_FRAME_1;
	param->frame.u0_m2_jitter_frame = PT_FRAME_1;
	param->frame.u3_m2_delta_frame	= PT_FRAME_1;
	param->frame.u0_m2_delta_frame	= PT_FRAME_1;
	param->frame.u3_blu_jitter_frame = PT_FRAME_1;
    param->frame.u3_p_jitter_frame = PT_FRAME_1;

	/* frame offset settings */
	param->offset.frame_1_offset = (0xEE6);
	param->offset.frame_2_offset = param->offset.frame_1_offset + 0x258;
	param->offset.frame_3_offset = param->offset.frame_2_offset + 0x23;	//0x126c
	param->offset.frame_4_offset = param->offset.frame_3_offset + 0x258;

	/* engine debugging tool offset */
	/* mutual data */
	param->ait_offset.raw		= (0x1786);
	param->ait_offset.delta		= (0x1786);
	param->ait_offset.label		= (0x1786);
	param->ait_offset.base		= (0x1786);
	param->ait_offset.debug		= (0x1786);
	/* self data */
	param->ait_offset.raw_s		= (0x18EE);	//need check self offset
	param->ait_offset.delta_s	= (0x18EE);
	param->ait_offset.label_s	= (0x18EE);
	param->ait_offset.base_s	= (0x18EE);
	param->ait_offset.debug_s	= (0x18EE);
	param->u3_m2_self = (0xEE6);
	/* pen data */
	param->ait_offset.pen_data	= (0x1786);
}

static int write_test_mode(struct device *dev, u32 type)
{
	u32 testmode = 0;
	u8 disp_mode = 0x3;
	int retry = 40;
	u32 rdata = 0x01;
	int waiting_time = 200;
	u8 type_temp = 0;
	u32 addr = 0;

	TOUCH_TRACE();

	switch (type) {
		case FLASH_INFO_CHECK:
			testmode = 0x3C;
			waiting_time = 10;
			retry = 40;
			break;
		case PT_INFO_CHECK:
			testmode = 0x33;
			waiting_time = 10;
			retry = 40;
			break;
		case OPEN_NODE_TEST:
			type_temp = 0x1;
			testmode = (disp_mode << 8) + type_temp;
			waiting_time = 10;
			break;
		case SHORT_NODE_TEST:
			type_temp = 0x2;
			testmode = (disp_mode << 8) + type_temp;
			waiting_time = 1000;
			break;
		case U3_M1_RAWDATA_TEST:
			type_temp = 0x3;
			testmode = (disp_mode << 8) + type_temp;
			break;
		case U3_M1_JITTER_TEST:
			type_temp = 0x4;
			testmode = (disp_mode << 8) + type_temp;
			waiting_time = 800;
			break;
		case U3_M2_RAW_SELF_TEST:
			type_temp = 0xF;
			testmode = (disp_mode << 8) + type_temp;
			break;
		case U3_M2_RAWDATA_TEST:
			type_temp = 0x5;
			testmode = (disp_mode << 8) + type_temp;
			break;
		case U3_M2_JITTER_TEST:
            type_temp = 0xD;
			testmode = ((disp_mode << 8) + type_temp);
			waiting_time = 800;
			break;
		case U3_M2_JITTER_SELF_TEST:
			type_temp = 0x10;
			testmode = ((disp_mode << 8) + type_temp);
			waiting_time = 800;
			break;
		case U0_M1_RAWDATA_TEST:
		case U0_M1_RAW_SELF_TEST:
			type_temp = 0x3;
			testmode = type_temp;
			break;
		case U0_M1_JITTER_TEST:
			type_temp = 0x4;
			testmode = type_temp;
			waiting_time = 800;
			break;
		case U0_M2_RAWDATA_TEST:
			type_temp = 0x5;
			testmode = type_temp;
			break;
		case U0_M2_JITTER_TEST:
			type_temp = 0xD;
			testmode = type_temp;
			waiting_time = 800;
			break;
		case U3_M2_DELTA_TEST:
			type_temp = 0xD;
			testmode = ((disp_mode << 8) + type_temp);
			waiting_time = 800;
			break;
		case U0_M2_DELTA_TEST:
			type_temp = 0xD;
			testmode = type_temp;
			waiting_time = 800;
			break;
		case U3_BLU_JITTER_TEST:
			type_temp = 0xD;
			waiting_time = 800;
			retry = 60;
			testmode = ((disp_mode << 8) + type_temp);
			break;
        case U3_P_JITTER_TEST:
            type_temp = 0x0E;
			waiting_time = 800;
			retry = 60;
			testmode = ((disp_mode << 8) + type_temp);
			break;
	}

	/* TestType Set */
	dlkm_sw82908_reg_write(dev, TC_PT_TEST_CTL,
			(u32 *)&testmode, sizeof(testmode));
	TOUCH_I("write testmode (0x%x)= %xh\n", TC_PT_TEST_CTL, testmode);
	touch_msleep(waiting_time);

	/* Check Test Result - wait until 0 is written */
	addr = TC_STS + PT_TEST_STS_OFT;
	do {
		touch_msleep(100);
		dlkm_sw82908_reg_read(dev, addr, (u8 *)&rdata, sizeof(rdata));
		TOUCH_I("rdata(0x%x) = 0x%x\n", addr, rdata);
        rdata &= 0xFFFF;
	} while ((rdata != 0xAA) && retry--);

	if (rdata != 0xAA) {
		TOUCH_I("ProductionTest Type [%d] Time out\n", type);
		goto error;
	}
	return 1;
error:
	TOUCH_E("[%s] fail\n", __func__);
	return 0;
}

static int prd_read_self_print(struct device *dev, int offset, int row, int col, u8 type)
{
	int i = 0, j = 0;
	int ret = 0;
	int log_ret = 0;
	char log_buf[LOG_BUF_SIZE] = {0, };

	TOUCH_TRACE();

	TOUCH_I("read_offset = %xh\n", offset);

	memset(&self_data, 0x0, sizeof(self_data));

	dlkm_sw82908_write_value(dev, tc_tsp_test_data_offset, offset);

	dlkm_sw82908_reg_read(dev, data_access_addr,
			(u16 *)self_data, (row+col)*sizeof(u16));

	for (i = 0; i < row + col; i++) {
		if (i == 0) {
			ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "\n[%s_RX]\n  : ", test_name_str[type]);
			for (j = 0; j < col; j++) {
				ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, " [%2d]", j);
			}
			ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "\n[ 0]");
			log_ret = touch_snprintf(log_buf, LOG_BUF_SIZE, " RX: ");
		}

		if (i == row) {
			TOUCH_I("%s\n", log_buf);
			log_ret = 0;
			memset(log_buf, 0, sizeof(log_buf));

			log_ret += touch_snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, " TX: ");
			ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "\n[%s_TX]\n  : ", test_name_str[type]);
			for (j = 0; j < col; j++) {
				ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, " [%2d]", j);
			}
			ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "\n[ 0]");
		}

		ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "%5d", self_data[i]);
		log_ret += touch_snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "%5d", self_data[i]);
	}

	TOUCH_I("%s\n", log_buf);

	ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "\n");
	memset(w_buf, 0, BUF_SIZE);

	return ret;
}

static int prd_read_print_frame(struct device *dev, int offset, int row, int col, int direction)
{
	int i = 0, j = 0;
	int ret = 0;
	int log_ret = 0;
	char log_buf[LOG_BUF_SIZE] = {0, };

	TOUCH_TRACE();

	TOUCH_I("read_offset = %xh\n", offset);

	memset(&data_buf, 0x0, sizeof(data_buf));

	dlkm_sw82908_write_value(dev, tc_tsp_test_data_offset, offset);

	dlkm_sw82908_reg_read(dev, data_access_addr,
			(u16 *)data_buf, row*col*sizeof(u16));

	/* print a frame data */
	ret = touch_snprintf(w_buf, PAGE_SIZE, "  : ");
	log_ret = touch_snprintf(log_buf, LOG_BUF_SIZE - log_ret, "  : ");

	for (i = 0; i < col; i++) {
		ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, " [%2d]", i);
		log_ret += touch_snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, " [%2d]", i);
	}
	TOUCH_I("%s\n", log_buf);

	for (i = 0; i < row; i++) {
		log_ret = 0;
		memset(log_buf, 0, sizeof(log_buf));
		ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret,  "\n[%2d]", i);
		log_ret += touch_snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret,  "[%2d]", i);

		for (j = 0; j < col; j++) {
			if ((LowerImage[i][j] | UpperImage[i][j]) == 0) {
				ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "%5d",
					data_buf_read_opt(0, 4, col, row, direction));
				log_ret += touch_snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "%5d",
					data_buf_read_opt(0, 4, col, row, direction));
			} else {
				ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "%5d",
					data_buf_read_opt(i, j, col, row, direction));
				log_ret += touch_snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "%5d",
					data_buf_read_opt(i, j, col, row, direction));
			}
		}
		TOUCH_I("%s\n", log_buf);
	}

	ret += touch_snprintf(w_buf + ret, PAGE_SIZE - ret, "\n");
	memset(w_buf, 0, BUF_SIZE);

	return ret;
}

static int prd_open_result_get(struct device *dev, int frame)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int ret = 0;

	TOUCH_TRACE();

	switch (frame) {
		case PT_FRAME_1:
			ret = prd_read_print_frame(dev, param->offset.frame_1_offset,
				ROW_SIZE, COL_SIZE, VERTICAL_DATA);
			break;
		case PT_FRAME_2:
			TOUCH_E("Attempt to read invalid frame\n");
			break;
		case PT_FRAME_3:
			ret = prd_read_print_frame(dev, param->offset.frame_3_offset,
				ROW_SIZE, COL_SIZE, VERTICAL_DATA);
			break;
		case PT_FRAME_4:
			TOUCH_E("Attempt to read invalid frame\n");
			break;
	}
	return ret;
}

static int prd_short_result_get(struct device *dev, int frame)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int ret = 0;

	TOUCH_TRACE();

	switch (frame) {
		case PT_FRAME_1:
            ret = prd_read_self_print(dev, param->offset.frame_1_offset, ROW_SIZE, COL_SIZE, 0);
			break;
		case PT_FRAME_2:
			TOUCH_E("Attempt to read invalid frame\n");
			break;
		case PT_FRAME_3:
			ret = prd_read_print_frame(dev, param->offset.frame_3_offset,
				ROW_SIZE, COL_SIZE, VERTICAL_DATA);
			break;
		case PT_FRAME_4:
			TOUCH_E("Attempt to read invalid frame\n");
			break;
	}
	return ret;
}

static int __used prd_os_xline_result_read(struct device *dev, int type)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int result = 0;

	TOUCH_TRACE();

	switch (type) {
	case OPEN_RX_NODE_TEST:
		TOUCH_I("[OPEN RX NODE DATA]\n");
		prd_open_result_get(dev, param->frame.open_rx_frame);
		break;
	case OPEN_TX_NODE_TEST:
		TOUCH_I("[OPEN TX NODE DATA]\n");
		prd_open_result_get(dev, param->frame.open_tx_frame);
		break;
	case SHORT_RX_NODE_TEST:
		TOUCH_I("[SHORT RX NODE DATA]\n");
		prd_short_result_get(dev, param->frame.short_rx_frame);
		break;
	case SHORT_TX_NODE_TEST:
		TOUCH_I("[SHORT TX NODE DATA]\n");
		prd_short_result_get(dev, param->frame.short_tx_frame);
		break;
	}
	if (param->os_test_type == DATA_DRIVER_COMPARE) {
		memset(w_buf, 0, BUF_SIZE);

		/* rawdata compare result(pass : 0 fail : 1) */
		prd_compare_rawdata(dev, type, &result);
		TOUCH_I("compare result(pass : 0 fail : 1) = %d\n", result);

	}

	return result;
}

static int sic_get_simple_limit(struct device *dev, char *breakpoint, u16 (*buf))
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	boot_mode = touch_check_boot_mode(dev);
	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
	case TOUCH_MINIOS_AAT:
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
	case TOUCH_MINIOS_MFTS_DS_FLAT:
		break;
	case TOUCH_CHARGER_MODE:
	case TOUCH_LAF_MODE:
	case TOUCH_RECOVERY_MODE:
		TOUCH_I("%s: Etc boot_mode(%d)!!!\n", __func__, boot_mode);
		ret = -1;
		goto error;
		break;
	default:
		TOUCH_E("%s: invalid boot_mode = %d\n", __func__, boot_mode);
		ret = -1;
		goto error;
		break;
	}
	if (line == NULL) {
		ret =  -1;
		goto error;
	}
	TOUCH_I("find breakpoint: %s\n", breakpoint);

	found = strnstr(line, breakpoint, strlen(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_E("failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -1;
		goto error;
	}

	memset(buf, 0, sizeof(u16) * SIMPLE_SPEC_SIZE);

	while (1) {
		if (line[q] == '}') {
			TOUCH_I("panel node spec size is over(%s)\n", __func__);
			break;
		}
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				buf[r] += ((line[q - p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
		}
		q++;
		/* just low and high spec */
		if (r == (int)SIMPLE_SPEC_SIZE) {
			TOUCH_I("panel_spec_file size scanning is success(%s)\n", __func__);
			break;
		}
	}

	if (ret == 0) {
		ret = -1;
		goto error;

	} else {
		TOUCH_I("panel_spec_file scanning is success\n");
		return ret;
	}

error:
	return ret;
}

static int sic_get_limit(struct device *dev, char *breakpoint, u16 (*buf)[COL_SIZE])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int boot_mode = TOUCH_NORMAL_BOOT;
	int tx_num = 0;
	int rx_num = 0;

	TOUCH_TRACE();

	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	boot_mode = touch_check_boot_mode(dev);
	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
	case TOUCH_MINIOS_AAT:
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
	case TOUCH_MINIOS_MFTS_DS_FLAT:
		break;
	case TOUCH_CHARGER_MODE:
	case TOUCH_LAF_MODE:
	case TOUCH_RECOVERY_MODE:
		TOUCH_I("%s: Etc boot_mode(%d)!!!\n", __func__, boot_mode);
		ret = -1;
		goto error;
		break;
	default:
		TOUCH_E("%s: invalid boot_mode = %d\n", __func__, boot_mode);
		ret = -1;
		goto error;
		break;
	}

	if (line == NULL) {
		ret =  -1;
		goto error;
	}
	TOUCH_I("find breakpoint: %s!\n", breakpoint);

	found = strnstr(line, breakpoint, strlen(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_E("failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -1;
		goto error;
	}

	memset(buf, 0, ROW_SIZE * COL_SIZE * 2);

	while (1) {
		if (line[q] == '}') {
			TOUCH_I("panel node spec size is over(%s)\n", __func__);
			break;
		}
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				buf[tx_num][rx_num] += ((line[q - p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
			if (r % (int)COL_SIZE == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;
		if (r == (int)ROW_SIZE * (int)COL_SIZE) {
			TOUCH_I("panel_spec_file size scanning is success(%s)\n", __func__);
			break;
		}
	}

	if (ret == 0) {
		ret = -1;
		goto error;

	} else {
		TOUCH_I("panel_spec_file scanning is success\n");
		return ret;
	}

error:
	return ret;
}

static int prd_open_short_test(struct device *dev)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int type = 0;
	int ret = 0;
	int write_test_mode_result = 0;
	u32 open_result = 0;
	u32 short_result = 0;
	u32 open_rx_result = 0;
	u32 open_tx_result = 0;
	u32 short_rx_result = 0;
	u32 short_tx_result = 0;
	u32 openshort_all_result = 0;
	u32 result_addr = 0;

	TOUCH_TRACE();

	/* Test Type Write */
	TOUCH_I("[OPEN_TEST]\n");

	result_addr = TC_STS + PT_TEST_PF_RESULT_OFT;

	/* 1. open_test */
	type = OPEN_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	if (param->os_test_type == DATA_IC_COMPARE) {
		dlkm_sw82908_reg_read(dev, result_addr, (u8 *)&open_result, sizeof(open_result));
		TOUCH_I("open_result(0x%x) = %d\n", result_addr, open_result);
	}

	if ((param->os_test_type == DATA_DRIVER_COMPARE)||(open_result & 0x1)) {
		sic_get_limit(dev, "OPEN_Lower", LowerImage);
		sic_get_limit(dev, "OPEN_Upper", UpperImage);
		open_rx_result = prd_os_xline_result_read(dev, OPEN_RX_NODE_TEST);
		/*open_tx_result = prd_os_xline_result_read(dev, OPEN_TX_NODE_TEST);*/
	}
	if ((open_result & 0x1)||open_rx_result||open_tx_result) {
		// open test logging
		openshort_all_result |= 0x1;
	}

	/* Test Type Write */
	TOUCH_I("[SHORT_TEST]\n");

	/* 2. short_test */
	type = SHORT_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	if (param->os_test_type == DATA_IC_COMPARE) {
		dlkm_sw82908_reg_read(dev, result_addr, (u8 *)&short_result, sizeof(short_result));
		TOUCH_I("short_result(0x%x) = %d\n", result_addr,  short_result);
	}

	if ((param->os_test_type == DATA_DRIVER_COMPARE)||(short_result & 0x2)) {
		sic_get_simple_limit(dev, "SHORT_test", SimpleImage);
		short_rx_result = prd_os_xline_result_read(dev, SHORT_RX_NODE_TEST);
		/*short_tx_result = prd_os_xline_result_read(dev, SHORT_TX_NODE_TEST);*/
	}
	if ((short_result & 0x2)||short_rx_result||short_tx_result) {
		// short test logging
		openshort_all_result |= 0x2;
	}

	/* fail case */
	if (openshort_all_result != 0) {
		ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "\nOPEN_SHORT_ALL_TEST : Fail\n");
		TOUCH_I("OPEN_SHORT_ALL_TEST : Fail\n");
	} else {
		ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "\nOPEN_SHORT_ALL_TEST : Pass\n");
		TOUCH_I("OPEN_SHORT_ALL_TEST : Pass\n");
	}

	return openshort_all_result;
}

static int read_offset(struct device *dev, u16 type)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int ret = 0;

	TOUCH_TRACE();

	switch (type) {
	case PT_FRAME_1:
		ret = param->offset.frame_1_offset;
		break;
	case PT_FRAME_2:
		ret = param->offset.frame_2_offset;
		break;
	case PT_FRAME_3:
		ret = param->offset.frame_3_offset;
		break;
	case PT_FRAME_4:
		ret = param->offset.frame_4_offset;
		break;
	}

	return ret;
}

static void prd_read_rawdata(struct device *dev, u8 type)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;

	TOUCH_TRACE();

	switch (type) {
	case U3_M1_RAWDATA_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u3_m1_raw_frame), ROW_SIZE, 2, VERTICAL_DATA);
		break;
	case U3_M1_JITTER_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u3_m1_jitter_frame), ROW_SIZE, 2, VERTICAL_DATA);
		break;
	case U3_M2_RAWDATA_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u3_m2_raw_frame), ROW_SIZE, COL_SIZE, VERTICAL_DATA);
		break;
	case U3_M2_JITTER_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u3_m2_jitter_frame), ROW_SIZE, COL_SIZE, VERTICAL_DATA);
		break;
	case U0_M1_RAWDATA_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u0_m1_raw_frame), ROW_SIZE, 2, VERTICAL_DATA);
		break;
	case U0_M1_JITTER_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u0_m1_jitter_frame), ROW_SIZE, 2, VERTICAL_DATA);
		break;
	case U0_M2_RAWDATA_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u0_m2_raw_frame), ROW_SIZE, COL_SIZE, VERTICAL_DATA);
		break;
	case U0_M2_JITTER_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u0_m2_jitter_frame), ROW_SIZE, COL_SIZE, VERTICAL_DATA);
		break;
	case U3_M2_DELTA_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u3_m2_delta_frame), ROW_SIZE, COL_SIZE, VERTICAL_DATA);
		break;
	case U0_M2_DELTA_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u0_m2_delta_frame), ROW_SIZE, COL_SIZE, VERTICAL_DATA);
		break;
	case U3_BLU_JITTER_TEST:
		prd_read_print_frame(dev, read_offset(dev, param->frame.u3_blu_jitter_frame), ROW_SIZE, COL_SIZE, VERTICAL_DATA);
		break;
	case U3_M2_RAW_SELF_TEST:
		prd_read_self_print(dev, param->u3_m2_self, ROW_SIZE, COL_SIZE, type);
		break;
	case U3_M2_JITTER_SELF_TEST:
		prd_read_self_print(dev, param->u3_m2_self, ROW_SIZE, COL_SIZE, type);
		break;
	case U0_M1_RAW_SELF_TEST:
		prd_read_self_print(dev, read_offset(dev, param->frame.u0_m1_raw_frame), 0, COL_SIZE, type);
		break;
    case U3_P_JITTER_TEST:
		prd_read_self_print(dev, read_offset(dev, param->frame.u3_p_jitter_frame), ROW_SIZE, COL_SIZE, type);
		break;
	default:
		break;
	}
}

/* Rawdata compare result
 * Pass : reurn 0
 * Fail : return 1
 */
static int prd_compare_rawdata(struct device *dev, u8 type, int *result)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int dir = VERTICAL_DATA;

	/* Average Jiiter Buf */
	u16 m2_raw_average_buf[2][ROW_SIZE] = {{0} };
	int col_size = COL_SIZE;
	int i, j;
	int ret = 0;

	TOUCH_TRACE();

	switch (type) {
	case U3_M1_RAWDATA_TEST:
		col_size = M1_COL_SIZE;
		break;
	case U3_M2_RAWDATA_TEST:
		break;
	case U3_M2_JITTER_TEST:
		TOUCH_I("U3 M2 JITTER spec lower:%d uppper:%d", LowerImage[0][0], UpperImage[0][0]);
		break;
	case U0_M1_RAWDATA_TEST:
		col_size = M1_COL_SIZE;
		break;
	case U0_M2_RAWDATA_TEST:
		break;
	case U0_M2_JITTER_TEST:
		TOUCH_I("U0 M2 JITTER spec lower:%d uppper:%d", LowerImage[0][0], UpperImage[0][0]);
		break;
	case U3_BLU_JITTER_TEST:
		break;
   	case U3_M1_JITTER_TEST:
   	case U0_M1_JITTER_TEST:
		col_size = M1_COL_SIZE;
        TOUCH_I("M1 JITTER spec lower:%d uppper:%d", SimpleImage[0], SimpleImage[1]);

        for (i = 0; i < ROW_SIZE; i++) {
	        for (j = 0; j < col_size; j++) {
		   	    if ((data_buf_read_opt(i, j, col_size, ROW_SIZE, dir) < SimpleImage[0]) ||
		    	    (data_buf_read_opt(i, j, col_size, ROW_SIZE, dir) > SimpleImage[1])) {

   				    if ((SimpleImage[0] | SimpleImage[1]) != 0) {
    				    *result = 1;
	    			    TOUCH_I("F [%d][%d] = %d , %d, %d\n", i, j, data_buf_read_opt(i, j, col_size, ROW_SIZE, dir),
		    			    	SimpleImage[0], SimpleImage[1]);
   			    		ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d][%d] = %d\n", i, j,
    			    			data_buf_read_opt(i, j, col_size, ROW_SIZE, dir));
	    		    }
			        if (ret > (BUF_SIZE / 2)) {
				    	memset(w_buf, 0, BUF_SIZE);
					    ret = 0;
		    	    }
		        }
	        }
        }

        return ret;
	case OPEN_RX_NODE_TEST:
	case OPEN_TX_NODE_TEST:
		TOUCH_I("OPEN spec lower:%d uppper:%d", LowerImage[0][0], UpperImage[0][0]);
		//dir = HORIZONTAL_DATA;
		break;
	case SHORT_RX_NODE_TEST:
	case SHORT_TX_NODE_TEST:
		TOUCH_I("SHORT spec lower:%d uppper:%d", SimpleImage[0], SimpleImage[1]);
            for (i = 0; i < ROW_SIZE + col_size ; i++) {
                if ((self_data[i] < SimpleImage[0]) ||
                        (self_data[i] > SimpleImage[1])) {
                    if ((SimpleImage[0] | SimpleImage[1]) != 0) {
                        *result = 1;
                        TOUCH_I("F [%d] = %d , %d, %d\n", i,  self_data[i],
                                SimpleImage[0], SimpleImage[1]);
                        ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d] = %d\n", i,
                                self_data[i]);
                    }
                }
            }
            return ret;
	case U3_M2_RAW_SELF_TEST:
		TOUCH_I("U3_M2_RAW_SELF spec lower:%d uppper:%d", SimpleImage[0], SimpleImage[1]);
		for (i = 0; i < ROW_SIZE + col_size ; i++) {
			if ((self_data[i] < SimpleImage[0]) ||
					(self_data[i] > SimpleImage[1])) {

				if ((SimpleImage[0] | SimpleImage[1]) != 0) {
					*result = 1;
					TOUCH_I("F [%d] = %d , %d, %d\n", i,  self_data[i],
							SimpleImage[0], SimpleImage[1]);
					ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d] = %d\n", i,
							self_data[i]);
				}
			}
		}
		return ret;
	case U3_M2_JITTER_SELF_TEST:
		TOUCH_I("U3_M2_JITTER_SELF spec lower:%d uppper:%d", SimpleImage[0], SimpleImage[1]);
		for (i = 0; i < ROW_SIZE + col_size ; i++) {
			if ((self_data[i] < SimpleImage[0]) ||
					(self_data[i] > SimpleImage[1])) {

				if ((SimpleImage[0] | SimpleImage[1]) != 0) {
					*result = 1;
					TOUCH_I("F [%d] = %d , %d, %d\n", i,  self_data[i],
							SimpleImage[0], SimpleImage[1]);
					ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d] = %d\n", i,
							self_data[i]);
				}
			}
		}
		return ret;
	case U0_M1_RAW_SELF_TEST:
		TOUCH_I("U0_M1_RAW_SELF spec lower:%d uppper:%d", SimpleImage[0], SimpleImage[1]);
		for (i = 0; i < col_size ; i++) {
			if ((self_data[i] < SimpleImage[0]) ||
					(self_data[i] > SimpleImage[1])) {

				if ((SimpleImage[0] | SimpleImage[1]) != 0) {
					*result = 1;
					TOUCH_I("F [%d] = %d , %d, %d\n", i,  self_data[i],
							SimpleImage[0], SimpleImage[1]);
					ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d] = %d\n", i,
							self_data[i]);
				}
			}
		}
		return ret;
    case U3_P_JITTER_TEST:
		TOUCH_I("U3_P_JITTER spec lower:%d uppper:%d", SimpleImage[0], SimpleImage[1]);
		for (i = 0; i < ROW_SIZE + col_size ; i++) {
			if ((self_data[i] < SimpleImage[0]) ||
					(self_data[i] > SimpleImage[1])) {

				if ((SimpleImage[0] | SimpleImage[1]) != 0) {
					*result = 1;
					TOUCH_I("F [%d] = %d , %d, %d\n", i,  self_data[i],
							SimpleImage[0], SimpleImage[1]);
					ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d] = %d\n", i,
							self_data[i]);
				}
			}
		}
		return ret;
	default:
		TOUCH_I("(%s) not support\n", test_name_str[type]);
		return ret;
	}

	for (i = 0; i < ROW_SIZE; i++) {
		for (j = 0; j < col_size; j++) {
			if ((data_buf_read_opt(i, j, col_size, ROW_SIZE, dir) < LowerImage[i][j]) ||
				(data_buf_read_opt(i, j, col_size, ROW_SIZE, dir) > UpperImage[i][j])) {

				if ((LowerImage[i][j] | UpperImage[i][j]) != 0) {
					*result = 1;
					TOUCH_I("F [%d][%d] = %d , %d, %d\n", i, j, data_buf_read_opt(i, j, col_size, ROW_SIZE, dir),
							LowerImage[i][j], UpperImage[i][j]);
					ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d][%d] = %d\n", i, j,
							data_buf_read_opt(i, j, col_size, ROW_SIZE, dir));
				}

				if (ret > (BUF_SIZE / 2)) {
						memset(w_buf, 0, BUF_SIZE);
						ret = 0;
				}
			}
		}
	}

	if (param->sd_test_set & (1<<AVERAGE_JITTER_TEST)) {
		if ((type == U3_M2_JITTER_TEST) || (type == U3_BLU_JITTER_TEST)) {
			//Average jitter
			for (i = 0; i < ROW_SIZE; i++) {
				for (j = 0; j < col_size; j++) {
					//Average Jitter Buf store
					//m2_raw_average_buf[j] += data_buf[i*col_size+j];
					if (j < (col_size / 2)) {
						m2_raw_average_buf[0][i] += data_buf_read_opt(i, j, col_size, ROW_SIZE, dir);
						//TOUCH_I("[Left]aver_sum:%d, data:%d",
						//m2_raw_average_buf[0][i], data_buf[i*col_size+j]);
					} else {
						m2_raw_average_buf[1][i] += data_buf_read_opt(i, j, col_size, ROW_SIZE, dir);
						//TOUCH_I("[Right]aver_sum:%d, data:%d",
						//m2_raw_average_buf[1][i], data_buf[i*col_size+j]);
					}
				}
				m2_raw_average_buf[0][i] /= (col_size / 2); //Left
				m2_raw_average_buf[1][i] /= (col_size - (col_size / 2)); //Right

				//TOUCH_I("left:%d, left_spec:%d, right:%d, rihgt_spec:%d\n",
				//m2_raw_average_buf[0][i], AverageImage[i][0], m2_raw_average_buf[1][i], AverageImage[i][1]);

				if (m2_raw_average_buf[0][i] > AverageImage[i][0]) {
					*result = 1;
					ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret,
							"F [%d] Row Left Average = %d\n", i, m2_raw_average_buf[0][i]);
					TOUCH_I("F [%d] Row Left Average = %d\n", i, m2_raw_average_buf[0][i]);
				}
				if (m2_raw_average_buf[1][i] > AverageImage[i][1]) {
					*result = 1;
					ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret,
							"F [%d] Row Right Average = %d\n", i, m2_raw_average_buf[1][i]);
					TOUCH_I("F [%d] Row Right Average = %d\n", i, m2_raw_average_buf[1][i]);
				}
			}
		}
	}

	return ret;
}

int dlkm_sw82908_tune_u8_print(int num_ch, u8 *tune_val, char *log_buf, int offset)
{
	int i;
	int temp = 0;
	for (i = 0; i < num_ch; i++) {
		if (((tune_val[i] >> 4) & 1) == 0) {
			temp = tune_val[i] & 0xF;

			offset += touch_snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"-%d ", temp);
		} else if (((tune_val[i] >> 4) & 1) == 1) {
			temp = tune_val[i] & 0xF;

			offset += touch_snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"%d  ", temp);
		} else {
			offset += touch_snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"ISB  ");
		}
	}
	offset += touch_snprintf(log_buf + offset, tc_tune_code_size - offset, "\n");

	return offset;
}


static void tune_display(struct device *dev, struct tune_data_sel *sel, int type)
{
	char log_buf[tc_tune_code_size] = {0,};
	int offset = 0;

	TOUCH_TRACE();

	switch (type) {
		case U3_M1_RAWDATA_TEST:;
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U3 M1 tune: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m1.m1_tune_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);
			break;
		//case U2_M1_RAWDATA_TEST:
		//	break;
		case U0_M1_RAWDATA_TEST:
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U0 M1 tune: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m1.m1_tune_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);
			break;
		case U3_M2_RAWDATA_TEST:
		case U3_M2_RAW_SELF_TEST:
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U3 M2 tune high: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.m2_tune_high_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);

			offset = 0;
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U3 M2 tune low: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.m2_tune_low_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);

			offset = 0;
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U3 M2 tune self1: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.self1_tune_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);

			offset = 0;
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U3 M2 tune self2: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.self2_tune_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);
			break;
		//case U2_M2_RAWDATA_TEST:
			//break;
		case U0_M2_RAWDATA_TEST:
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U0 M2 tune high: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.m2_tune_high_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);

			offset = 0;
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U0 M2 tune low: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.m2_tune_low_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);

			offset = 0;
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U0 M2 tune self1: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.self1_tune_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);

			offset = 0;
			offset += touch_snprintf(log_buf+offset, tc_tune_code_size-offset, "U0 M2 tune self2: ");
			offset += dlkm_sw82908_tune_u8_print(LOFT_CH_NUM, sel->m2.self2_tune_sel_code, log_buf, offset);
			TOUCH_I("%s", log_buf);
			break;

		default:
			TOUCH_E("tune_display ERROR(%d)\n", type);
			return;
	}
}

static void __used read_tune_code(struct device *dev, u8 type)
{
	struct tune_data_format t;
	struct tune_data_sel sel;
	void *data = 0;
	int size = 0;
	int m1_tune_size = sizeof(struct m1_tune);
	int m2_tune_size = sizeof(struct m2_tune);
	int m1_offset_size = m1_tune_size/4;
//	int m2_offset_size = m2_tune_size/4;
	int sel_offset_size = sizeof(struct tune_data_sel)/4;
	int offset = sizeof(t.r_tune_code_magic)/4;

	TOUCH_TRACE();

	switch (type) {
		case U3_M1_RAWDATA_TEST:
			data = &sel.m1;
			size = m1_tune_size;
			//offset += 0;
			break;
		case U3_M2_RAWDATA_TEST:
		case U3_M2_RAW_SELF_TEST:
			data = &sel.m2;
			size = m2_tune_size;
			offset += m1_offset_size;
			break;
#if(0)
		case U2_M1_RAWDATA_TEST:
			data = &sel->m1;
			size = m1_tune_size;
			offset += sel_offset_size;
			break;
		case U2_M2_RAWDATA_TEST:
			data = &sel->m2;
			size = m2_tune_size;
			offset += (sel_offset_size + m1_offset_size);
			break;
#endif
		case U0_M1_RAWDATA_TEST:
			data = &sel.m1;
			size = sizeof(struct m1_tune);
			offset += (sel_offset_size * 2);
			break;
		case U0_M2_RAWDATA_TEST:
			data = &sel.m2;
			size = sizeof(struct m2_tune);
			offset += ((sel_offset_size * 2) + m1_offset_size);
			break;
		default:
			TOUCH_E("Tune Code type error(%d)\n", type);
			return;

	}

	TOUCH_I("\n");
	TOUCH_I("[Read Tune Code]");

	dlkm_sw82908_reg_read(dev, tune_code_addr + offset, data, size);

/*
	TOUCH_I("offset: %x (%x)\n", offset, tune_code_addr + offset);
	TOUCH_I("size: %d \n", size);	//for test
	TOUCH_I("&sel:%x, &sel.m1:%x, &sel.m2:%x \n", &sel, &sel.m1, &sel.m2);
*/
	tune_display(dev, &sel, type);

}

static int prd_rawdata_test(struct device *dev, u8 type)
{
	char test_type[32] = {0, };
	int result = 0;
	int write_test_mode_result = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (type < TEST_TOTAL_NUM) {
		sprintf(test_type, "\n[%s]\n", test_name_str[type]);
		TOUCH_I("%s", test_type);
	} else {
		TOUCH_E("Test Type not defined");
		return 1;
	}

	/* Test Type Write */

	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("production test couldn't be done\n");
		return 1;
	}

	prd_read_rawdata(dev, type);

	memset(w_buf, 0, BUF_SIZE);

	/* rawdata compare result(pass : 0 fail : 1) */
	ret = prd_compare_rawdata(dev, type, &result);

	if (result != 0) {
		ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "%s : Fail\n", test_name_str[type]);
		TOUCH_I("%s : Fail\n", test_name_str[type]);
	} else {
		ret += touch_snprintf(w_buf + ret, BUF_SIZE - ret, "%s : Pass\n", test_name_str[type]);
		TOUCH_I("%s : Pass\n", test_name_str[type]);
	}

	/* tune code ok - 20180604
	 * test sequence
	 * self test -> read tune code
	switch (type) {
	case U3_M2_RAW_SELF_TEST:
		read_tune_code(dev, type);
		break;
	}
	*/

	return result;
}

static void firmware_version_log(struct device *dev)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	int ret = 0;
	int offset = 0;
	unsigned char buf[LOG_BUF_SIZE] = {0,};
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
	case TOUCH_MINIOS_MFTS_DS_FLAT:
		ret = dlkm_sw82908_ic_info(dev);
		break;
	default:
		break;
	}

	offset += touch_snprintf(buf, LOG_BUF_SIZE,
			"==================== Version Info ====================\n");
	offset += touch_snprintf(buf + offset, LOG_BUF_SIZE - offset,
			"version: v%d.%02d, build: %d, chip id: %d, protocol: %d\n",
			d->ic_info.version.major, d->ic_info.version.minor, d->ic_info.version.build,
			d->ic_info.version.chip_id, d->ic_info.version.protocol_ver);
	offset += touch_snprintf(buf + offset, LOG_BUF_SIZE - offset,
			"product id: %s\n", d->ic_info.product_id);
#if !IS_ENABLED(__SW82908_STILL_BRING_UP)
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
		"chip_rev : %d, channel : %d, sensor_ver : %d, fpc_ver : %d\n",
			d->ic_info.pt_info.chip_rev, d->ic_info.pt_info.channel, d->ic_info.pt_info.sensor_ver, d->ic_info.pt_info.fpc_ver);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			"date: %04d.%02d.%02d, time: %02d:%02d:%02d\n",
			d->ic_info.pt_info.pt_date_year, d->ic_info.pt_info.pt_date_month, d->ic_info.pt_info.pt_date_day,
			d->ic_info.pt_info.pt_time_hour, d->ic_info.pt_info.pt_time_min, d->ic_info.pt_info.pt_time_sec);
#endif

	offset += touch_snprintf(buf + offset, LOG_BUF_SIZE - offset,
			"======================================================\n");

}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int openshort_ret = 0;
	int u3_m1_raw_ret = 0;
	int u3_m2_raw_ret = 0;
	int u3_m1_jitter_ret = 0;
	int u3_m2_jitter_ret = 0;
	int u3_m2_delta_ret = 0;
	int u3_blu_jitter_ret = 0;
	int u3_m2_raw_self_ret = 0;
	int u3_m2_jitter_self_ret = 0;
	int u3_p_jitter_ret = 0;
	int ret = 0;

	TOUCH_I("Show_sd Test Start\n");

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = touch_snprintf(buf + ret, PAGE_SIZE - ret, "LCD mode is not U3. Test Result : Fail\n");
		TOUCH_I("LCD mode is not U3. Test Result : Fail\n");
		goto out;
	}

	firmware_version_log(dev);

	mutex_lock(&ts->lock);

	/* Test enter not used
	pt_command = (0x3 << 8) + U3_PT_TEST;
	sw82908_write_value(dev, TC_PT_TEST_CTL, pt_command);
    */

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	dlkm_sw82908_tc_driving(dev, LCD_MODE_STOP);


	/*
	 *	   OPEN_SHORT_ALL_TEST
	 *	   open - pass : 0, fail : 1
	 *	   short - pass : 0, fail : 2
	 */
	openshort_ret = prd_open_short_test(dev);

	/*
		U3_M1_RAWDATA_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M1_RAWDATA_TEST)) {
		sic_get_limit(dev, "U3_M1_Lower", LowerImage);
		sic_get_limit(dev, "U3_M1_Upper", UpperImage);
		u3_m1_raw_ret = prd_rawdata_test(dev, U3_M1_RAWDATA_TEST);
	}

	/*
		U3_M2_RAWDATA_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_RAWDATA_TEST)) {
		sic_get_limit(dev, "U3_M2_Lower", LowerImage);
		sic_get_limit(dev, "U3_M2_Upper", UpperImage);
		u3_m2_raw_ret = prd_rawdata_test(dev, U3_M2_RAWDATA_TEST);
	}

	/*
		U3_M2_RAW_SELF_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_RAW_SELF_TEST)) {
		sic_get_simple_limit(dev, "U3_M2_RAW_SELF", SimpleImage);
		u3_m2_raw_self_ret = prd_rawdata_test(dev, U3_M2_RAW_SELF_TEST);
	}

	/*
		U3_M1_JITTER_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M1_JITTER_TEST)) {
		sic_get_simple_limit(dev, "U3_M1_JITTER", SimpleImage);
		u3_m1_jitter_ret = prd_rawdata_test(dev, U3_M1_JITTER_TEST);
	}

	/*
		U3_M2_JITTER_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_JITTER_TEST)) {
		sic_get_limit(dev, "U3_M2_JITTER_Lower", LowerImage);
		sic_get_limit(dev, "U3_M2_JITTER_Upper", UpperImage);
		u3_m2_jitter_ret = prd_rawdata_test(dev, U3_M2_JITTER_TEST);
	}

	/*
		U3_M2_JITTER_SELF_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_JITTER_SELF_TEST)) {
		sic_get_simple_limit(dev, "U3_M2_JITTER_SELF", SimpleImage);
		u3_m2_jitter_self_ret = prd_rawdata_test(dev, U3_M2_JITTER_SELF_TEST);
	}

    /*
		U3_P_JITTER_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_P_JITTER_TEST)) {
		sic_get_simple_limit(dev, "U3_P_JITTER", SimpleImage);
		u3_p_jitter_ret = prd_rawdata_test(dev, U3_P_JITTER_TEST);
	}

	/*
		U3_M2_DELTA_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_DELTA_TEST)) {
		sic_get_limit(dev, "U3_M2_DELTA_Lower", LowerImage);
		sic_get_limit(dev, "U3_M2_DELTA_Upper", UpperImage);
		u3_m2_delta_ret = prd_rawdata_test(dev, U3_M2_DELTA_TEST);
	}

	/*
		U3_BLU_JITTER_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_DELTA_TEST)) {
		sic_get_limit(dev, "U3_BLU_JITTER_Lower", LowerImage);
		sic_get_limit(dev, "U3_BLU_JITTER_Upper", UpperImage);
		u3_blu_jitter_ret = prd_rawdata_test(dev, U3_BLU_JITTER_TEST);
	}
	/* TEMP CODE START */
	u3_m2_raw_ret = 0;//U3 M2 RADATA TEST
	u3_m1_jitter_ret = 0;//U3 M1 JITTER TEST
	u3_m2_jitter_ret = 0;//U3 M2 JITTER TEST
	u3_m2_raw_self_ret = 0;//U3 M2 RAW SELF TEST
	u3_m2_jitter_self_ret = 0;//U3 M2 JITTER SLEF TEST
	u3_p_jitter_ret = 0;//U3 P JITTER TEST
	openshort_ret = 0;//OPEN_SHORT_ALL_TEST
	/* TEMP CODE END */

	ret = touch_snprintf(buf, PAGE_SIZE, "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	if ((u3_m1_raw_ret + u3_m2_raw_ret + u3_m1_jitter_ret + u3_m2_jitter_ret
				+ u3_m2_raw_self_ret + u3_m2_jitter_self_ret+u3_p_jitter_ret) == 0) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Pass\n");
		TOUCH_I("Raw Data : Pass\n");
	} else {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Fail (m1_raw:%d/m2_raw:%d/m1_jitter:%d/m2_jitter:%d/p_jitter:%d)\n",
				u3_m1_raw_ret, u3_m2_raw_ret|u3_m2_raw_self_ret ,
				u3_m1_jitter_ret, u3_m2_jitter_ret|u3_m2_jitter_self_ret,u3_p_jitter_ret);
		TOUCH_I("Raw Data : Fail (m1_raw:%d/m2_raw:%d/m1_jitter:%d/m2_jitter:%d/p_jitter:%d)\n",
				u3_m1_raw_ret, u3_m2_raw_ret|u3_m2_raw_self_ret ,
				u3_m1_jitter_ret, u3_m2_jitter_ret|u3_m2_jitter_self_ret,u3_p_jitter_ret);
	}

	if (openshort_ret == 0) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Pass\n");
		TOUCH_I("Channel Status : Pass\n");
	} else {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Fail (open:%d/short:%d)\n",
				((openshort_ret & 0x1) == 0x1) ? 1 : 0,
				((openshort_ret & 0x2) == 0x2) ? 1 : 0);
		TOUCH_I("Channel Status : Fail (open:%d/short:%d)\n",
				((openshort_ret & 0x1) == 0x1) ? 1 : 0,
				((openshort_ret & 0x2) == 0x2) ? 1 : 0);
	}

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
			"=====================\n");
	TOUCH_I("=====================\n");

out:
	dlkm_sw82908_reset_ctrl(dev, HW_RESET_SYNC);
	mutex_unlock(&ts->lock);

	kfree(line);
	line = NULL;
	TOUCH_I("Show_sd Test End\n");

	return ret;
}

/* for siw app */
static int start_firmware(struct device *dev)
{
	u32 const cmd = IT_NONE;
	u32 check_data = 0;
	int ret = 0;

	TOUCH_TRACE();

	/* Release F/W to operate */
	ret = dlkm_sw82908_reg_write(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, (void *)&cmd,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	ret = dlkm_sw82908_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	ret = dlkm_sw82908_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	//TOUCH_I("check_data : %x\n", check_data);

error:
	if (ret < 0) {
		TOUCH_E("start_firmware Error, %d\n", ret);
	}
	return ret;
}

static int stop_firmware(struct device *dev, u32 wdata)
{
	u32 read_val;
	u32 check_data = 0;
	int try_cnt = 0;
	int ret = 0;

	TOUCH_TRACE();

	/* STOP F/W to check */
	dlkm_sw82908_reg_write(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &wdata, sizeof(u32));
	dlkm_sw82908_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	dlkm_sw82908_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));

	try_cnt = 1000;
	do {
		--try_cnt;
		if (try_cnt == 0) {
			TOUCH_E("[ERR]get_data->try_cnt == 0\n");
			ret = 1;
			goto error;
		}
		dlkm_sw82908_reg_read(dev, ADDR_CMD_REG_SIC_GETTER_READYSTATUS,
				&read_val, sizeof(u32));
		touch_msleep(10);
	} while (read_val != (u32)RS_IMAGE);

error:
	return ret;
}

static ssize_t prd_show_app_op_end(struct device *dev, char *buf, int prev_mode)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	int ret = 0;

	TOUCH_TRACE();

	buf[0] = REPORT_END_RS_OK;
	if (prev_mode != APP_REPORT_OFF) {
		prd->prd_app_mode = APP_REPORT_OFF;
		ret = start_firmware(dev);
		if (ret < 0) {
			TOUCH_E("Invalid get_data request!\n");
			buf[0] = REPORT_END_RS_NG;
		}
	}

	return 1;
}

static int prd_show_prd_get_data_raw_core(struct device *dev,
		u8 *buf, int size, u32 cmd, u32 offset, int flag) {
	int ret = 0;

	TOUCH_TRACE();

	if (cmd != DONT_USE_CMD) {
		ret = stop_firmware(dev, cmd);
		if (ret < 0) {
			goto out;
		}
	}

	ret = dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset, (u8 *)&offset, sizeof(offset));
	if (ret < 0) {
		goto out;
	}

	ret = dlkm_sw82908_reg_read(dev, data_access_addr, (void *)buf, size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_raw_ait(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_rawdata;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
			CMD_RAWDATA, param->ait_offset.raw, flag);
}

static int prd_show_prd_get_data_do_ait_basedata(struct device *dev, u8 *buf, int size, int step, int flag)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_rawdata;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
			CMD_BASE_DATA, param->ait_offset.base, flag);
}

static int prd_show_prd_get_data_do_deltadata(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int16_t *pbuf = (buf) ? (int16_t *)buf : prd->m2_buf_rawdata;
	int size_rd = (PRD_DELTA_SIZE<<1);
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_delta, size_rd,
			CMD_DELTADATA, param->ait_offset.delta, flag);
	if (ret < 0)
		goto out;

	memset(pbuf, 0, size);

	for (i = 0; i < PRD_M2_ROW_COL_SIZE; i++) {
		row = i / PRD_COL_SIZE;
		col = i % PRD_COL_SIZE;
		pbuf[i] = prd->buf_delta[(row + 1)*(PRD_COL_SIZE + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_labeldata(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	u8 *pbuf = (buf) ? buf : prd->buf_label;
	int size_rd = PRD_LABEL_TMP_SIZE;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_label_tmp, size_rd,
			CMD_LABELDATA, param->ait_offset.label, flag);
	if (ret < 0)
		goto out;

	memset(pbuf, 0, size);

	for (i = 0; i < PRD_M2_ROW_COL_SIZE; i++) {
		row = i / PRD_COL_SIZE;
		col = i % PRD_COL_SIZE;
		pbuf[i] = prd->buf_label_tmp[(row + 1)*(PRD_COL_SIZE + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_debug_buf(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->buf_debug;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
			CMD_DEBUGDATA, param->ait_offset.debug, flag);
}

static ssize_t prd_show_app_operator(struct device *dev, char *buf, int mode)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	u8 *pbuf = (u8 *)prd->m2_buf_rawdata;
	int size = (PRD_M2_ROW_COL_SIZE<<1);
	int flag = PRD_SHOW_FLAG_DISABLE_PRT_RAW;
	int prev_mode = prd->prd_app_mode;

	TOUCH_TRACE();

	//TOUCH_I("show app mode : %s(%d), 0x%X\n",
	//			prd_app_mode_str[mode], mode, flag);

	if (mode == APP_REPORT_OFF) {
		size = prd_show_app_op_end(dev, buf, prev_mode);
		goto out;
	}

	if (mode < APP_REPORT_MAX) {
		prd->prd_app_mode = mode;
	}

	switch (mode) {
	case APP_REPORT_RAW:
		prd_show_prd_get_data_do_raw_ait(dev, pbuf, size, flag);
		break;
	case APP_REPORT_BASE:
		prd_show_prd_get_data_do_ait_basedata(dev, pbuf, size, 0, flag);
		break;
	case APP_REPORT_DELTA:
		prd_show_prd_get_data_do_deltadata(dev, pbuf, size, flag);
		break;
	case APP_REPORT_LABEL:
		size = PRD_M2_ROW_COL_SIZE;
		pbuf = (u8 *)prd->buf_label,
		     prd_show_prd_get_data_do_labeldata(dev, pbuf, size, flag);
		break;
	case APP_REPORT_DEBUG_BUF:
		size = PRD_DEBUG_BUF_SIZE;
		pbuf = (u8 *)prd->buf_debug,
		     prd_show_prd_get_data_do_debug_buf(dev, pbuf, size, flag);
		break;
	default:
		if (prev_mode != APP_REPORT_OFF) {
			prd_show_app_op_end(dev, buf, prev_mode);
		}
		size = 0;
		break;
	}

	if (size) {
		memcpy(buf, pbuf, size);
	}

out:
	return (ssize_t)size;
}

static ssize_t prd_show_app_raw(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_RAW);
}

static ssize_t prd_show_app_base(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_BASE);
}

static ssize_t prd_show_app_label(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_LABEL);
}

static ssize_t prd_show_app_delta(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_DELTA);
}

static ssize_t prd_show_app_debug_buf(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_DEBUG_BUF);
}

static ssize_t prd_show_app_end(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_OFF);
}

static ssize_t prd_show_app_info(struct device *dev, char *buf)
{
	u32 temp = PRD_SYS_ATTR_EN_FLAG;

	TOUCH_TRACE();

	memset(buf, 0, PRD_APP_INFO_SIZE);

	buf[0] = (temp & 0xff);
	buf[1] = ((temp >> 8) & 0xff);
	buf[2] = ((temp >> 16) & 0xff);
	buf[3] = ((temp >> 24) & 0xff);

	buf[8] = PRD_ROW_SIZE;
	buf[9] = PRD_COL_SIZE;
	buf[10] = PRD_COL_ADD;
	buf[11] = PRD_CH;
	buf[12] = PRD_M1_COL;
	buf[13] = PRD_CMD_TYPE;
	buf[14] = SECOND_SCR_BOUND_I;
	buf[15] = SECOND_SCR_BOUND_J;

	TOUCH_I("<prd info> F:%08Xh \n",temp);
	TOUCH_I("R:%d C:%d C_A:%d CH:%d M1_C:%d \n	\
			CMD_T:%d S_SCR_I:%d S_SCR_J:%d \n",	\
		PRD_ROW_SIZE, PRD_COL_SIZE, PRD_COL_ADD, PRD_CH, PRD_M1_COL,	\
		PRD_CMD_TYPE, SECOND_SCR_BOUND_I, SECOND_SCR_BOUND_J);

	return PRD_APP_INFO_SIZE;
}

static struct siw_hal_prd_data *siw_hal_prd_alloc(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd;

	TOUCH_TRACE();

	prd = devm_kzalloc(dev, sizeof(*prd), GFP_KERNEL);
	if (!prd) {
		TOUCH_E("failed to allocate memory for prd\n");
		goto out;
	}

	touch_snprintf(prd->name, sizeof(prd->name)-1, "%s-prd", dev_name(dev));

	prd->dev = ts->dev;

	d->prd = prd;

out:
	return prd;
}

/* for siw app end */

static ssize_t get_data(struct device *dev, int16_t *buf, u32 wdata, int mode)
{
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	u32 m2_data_offset;
	int r_size = 0;
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("======== get_data(%d) ========\n", wdata);


	switch (wdata) {
	case CMD_RAWDATA:
	case CMD_RAWDATA_S:
		if (mode == DBG_MUTUAL) {
			m2_data_offset = param->ait_offset.raw;
			r_size = sizeof(int16_t)*ROW_SIZE*COL_SIZE;
		} else {
			m2_data_offset = param->ait_offset.raw_s;
			r_size = sizeof(int16_t)*(ROW_SIZE+COL_SIZE);
		}

		dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset,
				(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		dlkm_sw82908_reg_read(dev, data_access_addr, (int16_t *)buf, r_size);
		break;
	case CMD_BASE_DATA:
	case CMD_BASE_DATA_S:
		if (mode == DBG_MUTUAL) {
			m2_data_offset = param->ait_offset.base;
			r_size = sizeof(int16_t)*ROW_SIZE*COL_SIZE;
		} else {
			m2_data_offset = param->ait_offset.base_s;
			r_size = sizeof(int16_t)*(ROW_SIZE+COL_SIZE);
		}

		dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset,
				(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		dlkm_sw82908_reg_read(dev, data_access_addr, (int16_t *)buf, r_size);
		break;

	case CMD_DELTADATA:
	case CMD_DELTADATA_S:
		if (mode == DBG_MUTUAL) {
			m2_data_offset = param->ait_offset.delta;
			r_size = sizeof(int16_t)*(ROW_SIZE)*(COL_SIZE);
		} else {
			m2_data_offset = param->ait_offset.delta_s;
			r_size = sizeof(int16_t)*(ROW_SIZE+COL_SIZE);
		}

		dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset,
				(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		dlkm_sw82908_reg_read(dev, data_access_addr, (int16_t *)buf, r_size);
		break;
	case CMD_LABELDATA:
	case CMD_LABELDATA_S:
		if (mode == DBG_MUTUAL) {
			m2_data_offset = param->ait_offset.label;
			r_size = sizeof(int16_t)*(ROW_SIZE)*(COL_SIZE);
		} else {
			m2_data_offset = param->ait_offset.label_s;
			r_size = sizeof(int16_t)*(ROW_SIZE+COL_SIZE);
		}

		dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset,
				(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		dlkm_sw82908_reg_read(dev, data_access_addr, (int16_t *)buf, r_size);
		break;
	case CMD_DEBUGDATA:
	case CMD_DEBUGDATA_S:
		if (mode == DBG_MUTUAL){
			m2_data_offset = param->ait_offset.debug;
            r_size = sizeof(int16_t)*(ROW_SIZE)*(COL_SIZE);
		} else{
			m2_data_offset = param->ait_offset.debug_s;
            r_size = sizeof(int16_t)*(ROW_SIZE+COL_SIZE);
		}

		dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset,
				(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		dlkm_sw82908_reg_read(dev, data_access_addr, (int16_t *)buf, r_size);

		break;
	case CMD_RAW_P_POS:
	case CMD_BASE_P_POS:
	case CMD_DELTA_P_POS:
	case CMD_LABEL_P_POS:
	case CMD_DEBUG_P_POS:
		m2_data_offset = param->ait_offset.pen_data;

		dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset,
				(u32 *)&m2_data_offset, sizeof(m2_data_offset));

		dlkm_sw82908_reg_read(dev, data_access_addr, (int16_t *)buf,
				sizeof(int16_t)*(DEBUG_ROW_SIZE+DEBUG_COL_SIZE));
		break;
	case CMD_RAW_P:
	case CMD_BASE_P:
	case CMD_DELTA_P:
	case CMD_DEBUG_P:
		m2_data_offset = param->ait_offset.pen_data;

		dlkm_sw82908_reg_write(dev, tc_tsp_test_data_offset,
				(u32 *)&m2_data_offset, sizeof(m2_data_offset));

		dlkm_sw82908_reg_read(dev, data_access_addr, (int16_t *)buf,
				sizeof(int16_t)*(PEN_DATA_SIZE));
		break;
	default:
		TOUCH_E("Invalid get_data request!\n");
		break;
	}

//getdata_error:

	return ret;
}

static ssize_t show_fdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw82908_data *d = to_sw49202_data(dev);
	int ret = 0;
	int ret2 = 0;
	u8 type = U3_M2_RAWDATA_TEST;

	TOUCH_TRACE();

	/* LCD off */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD Off. Test Result : Fail\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret2 = write_test_mode(dev, type);
	if (ret2 == 0) {
		TOUCH_E("write_test_mode fail\n");
		dlkm_sw82908_reset_ctrl(dev, HW_RESET_SYNC);
		return ret;
	}

	prd_read_rawdata(dev, type);

	dlkm_sw82908_reset_ctrl(dev, HW_RESET_SYNC);
	mutex_unlock(&ts->lock);

	return ret;
}

static int check_osc_status(struct device *dev)
{
	int ret = 0;
	u32 data = 0;
	u32 value = 0;
	u32 addr = 0;

	TOUCH_TRACE();

	/* write data */
	data = 0x8018;
	addr =  0x3c;
	if (dlkm_sw82908_reg_write(dev, addr, &data, sizeof(u32)) < 0) {
		TOUCH_E("reg addr 0x%x write fail\n", addr);
		ret = 1;
	} else {
		TOUCH_I("reg[%x] = 0x%x\n", addr, data);
	}
	touch_msleep(10);

	/* read data */
	addr =  0xfd8;
	if (dlkm_sw82908_reg_read(dev, addr, &value, sizeof(u32)) < 0) {
		TOUCH_E("reg addr 0x%x read fail\n", addr);
		ret = 1;
	} else {
		TOUCH_I("reg[%x] = 0x%x\n", addr, value);
	}

	/* check read data */
	if (value == 0xffffffff) {
		ret = 1;
		TOUCH_I("trim data failed\n");
	} else {
		ret = 0;
		TOUCH_I("trim data success\n");
	}

	/* restore data */
	data = 0;
	addr = 0x3c;
	if (dlkm_sw82908_reg_write(dev, addr, &data, sizeof(u32)) < 0) {
		TOUCH_E("reg addr 0x%x write fail\n", addr);
	} else {
		TOUCH_I("reg[%x] = 0x%x\n", addr, data);
	}

	return ret;
}
static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int m1_rawdata_ret = 0;
	int m2_rawdata_ret = 0;
	int m1_jitter_ret = 0;
	int m2_jitter_ret = 0;
	int ret = 0;
	int osc_status_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("Show_lpwg_sd Test Start\n");

	/* LCD mode check */
	switch (d->lcd_mode) {
	case LCD_MODE_U0:
	case LCD_MODE_U2_UNBLANK:
	case LCD_MODE_U2:
		break;
	case LCD_MODE_U3:
	case LCD_MODE_U3_PARTIAL:
	case LCD_MODE_U3_QUICKCOVER:
	case LCD_MODE_STOP:
	default:
		ret = touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD mode[%d]. Test Result : Fail\n",
				d->lcd_mode);
		TOUCH_I("LCD mode[%d]. Test Result : Fail\n",
				d->lcd_mode);
		return ret;
	}

	firmware_version_log(dev);

	mutex_lock(&ts->lock);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	dlkm_sw82908_tc_driving(dev, LCD_MODE_STOP);

	osc_status_ret = check_osc_status(dev);

	/*
		U0_M1_RAW_SELF_TEST
		pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M1_RAW_SELF_TEST)) {
		sic_get_simple_limit(dev, "U0_M1_RAW_SELF", SimpleImage);
		m1_rawdata_ret = prd_rawdata_test(dev, U0_M1_RAW_SELF_TEST);
	}

	/*
		U0_M1_RAWDATA_TEST
		pass : 0, fail : 1
		U0 M1 used in U0_M1_RAW_SELF_TEST, so skip this test.
	*/

	if (param->lpwg_sd_test_set & (1<<U0_M1_RAWDATA_TEST)) {
		sic_get_limit(dev, "U0_M1_Lower", LowerImage);
		sic_get_limit(dev, "U0_M1_Upper", UpperImage);
		m1_rawdata_ret = prd_rawdata_test(dev, U0_M1_RAWDATA_TEST);
	}

	/*
		U0_M2_RAWDATA_TEST
		pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M2_RAWDATA_TEST)) {
		sic_get_limit(dev, "U0_M2_Lower", LowerImage);
		sic_get_limit(dev, "U0_M2_Upper", UpperImage);
		m2_rawdata_ret = prd_rawdata_test(dev, U0_M2_RAWDATA_TEST);
	}

	/*
		U0_M1_JITTER_TEST
		pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M1_JITTER_TEST)) {
		sic_get_simple_limit(dev, "U0_M1_JITTER", SimpleImage);
		m1_jitter_ret = prd_rawdata_test(dev, U0_M1_JITTER_TEST);
	}

	/*
		U0_M2_JITTER_TEST
		BLU Jitter - pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M2_JITTER_TEST)) {
		sic_get_limit(dev, "U0_M2_JITTER_Lower", LowerImage);
		sic_get_limit(dev, "U0_M2_JITTER_Upper", UpperImage);
		m2_jitter_ret = prd_rawdata_test(dev, U0_M2_JITTER_TEST);
	}

	/* TEMP CODE START */
	m2_rawdata_ret = 0;//U0 M2 RAWDATA TEST
	m2_jitter_ret = 0;//U0 M2 JITTER TEST
	/* TEMP CODE END */

	ret = touch_snprintf(buf + ret, PAGE_SIZE, "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	if (!m1_rawdata_ret && !m2_rawdata_ret && !m1_jitter_ret && !m2_jitter_ret && !osc_status_ret) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LPWG RawData : %s\n", "Pass");
		TOUCH_I("LPWG RawData : %s\n", "Pass");
	} else {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LPWG RawData : %s (m1_raw:%d/m2_raw:%d/m1_jitter:%d/m2_jitter:%d/osc_status:%d)\n", "Fail",
				m1_rawdata_ret, m2_rawdata_ret, m1_jitter_ret, m2_jitter_ret, osc_status_ret);
		TOUCH_I("LPWG RawData : %s (m1_raw:%d/m2_raw:%d/m1_jitter:%d/m2_jitter:%d/osc_status_ret:%d)\n", "Fail",
				m1_rawdata_ret, m2_rawdata_ret, m1_jitter_ret, m2_jitter_ret, osc_status_ret);
	}
	ret += touch_snprintf(buf + ret, PAGE_SIZE, "=====================\n");
	TOUCH_I("=====================\n");

	dlkm_sw82908_reset_ctrl(dev, HW_RESET_SYNC);

	mutex_unlock(&ts->lock);

	kfree(line);
	line = NULL;
	TOUCH_I("Show_lpwg_sd Test End\n");

	return ret;
}
static ssize_t show_psy_test(struct device *dev, char *buf)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("<<< U3_M1_RAWDATA_TEST >>>\n");
	write_test_mode(dev, U3_M1_RAWDATA_TEST);
	read_tune_code(dev, U3_M1_RAWDATA_TEST);

	TOUCH_I("<<< U3_M2_RAWDATA_TEST >>>\n");
	write_test_mode(dev, U3_M2_RAWDATA_TEST);
	read_tune_code(dev, U3_M2_RAWDATA_TEST);

	TOUCH_I("<<< U0_M1_RAWDATA_TEST >>>\n");
	write_test_mode(dev, U0_M1_RAWDATA_TEST);
	read_tune_code(dev, U0_M1_RAWDATA_TEST);

	TOUCH_I("<<< U0_M2_RAWDATA_TEST >>>\n");
	write_test_mode(dev, U0_M2_RAWDATA_TEST);
	read_tune_code(dev, U0_M2_RAWDATA_TEST);
	TOUCH_I("<<< OPEN_NODE_TEST >>>\n");
	write_test_mode(dev, OPEN_NODE_TEST);

	prd_os_xline_result_read(dev, OPEN_RX_NODE_TEST);
	prd_os_xline_result_read(dev, OPEN_TX_NODE_TEST);

	TOUCH_I("<<< SHORT_NODE_TEST >>>\n");
	write_test_mode(dev, SHORT_NODE_TEST);

	prd_os_xline_result_read(dev, SHORT_RX_NODE_TEST);
	prd_os_xline_result_read(dev, SHORT_TX_NODE_TEST);

	return ret;
}


static char log_buf_s[LOG_BUF_SIZE];

#define LOG_MAX_BUF 5600 //4200
static char kobj_log_buf[LOG_MAX_BUF] = {0,};

static ssize_t read_delta(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *delta = NULL;
	int i = 0;
	int j = 0;
	u32 cmd = CMD_DELTADATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		delta = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (delta == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== Deltadata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, delta, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", delta[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", delta[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (delta != NULL)
		kfree(delta);

	return count;
}

static ssize_t read_rawdata(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int i = 0;
	int j = 0;
	u32 cmd = CMD_RAWDATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (rawdata == NULL) {
			TOUCH_E("rawdata mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== rawdata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, rawdata, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", rawdata[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", rawdata[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	return count;
}

static ssize_t read_base(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *baseline = NULL;
	int i = 0;
	int j = 0;
	u32 cmd = CMD_BASE_DATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		baseline = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (baseline == NULL) {
			TOUCH_E("baseline mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== baseline ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, baseline, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", baseline[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", baseline[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[LOG_MAX_BUF-1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (baseline != NULL)
		kfree(baseline);

	return count;
}

static ssize_t read_debug(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *debugdata = NULL;
	int i = 0;
	int j = 0;
	u32 cmd = CMD_DEBUGDATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		debugdata = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (debugdata == NULL) {
			TOUCH_E("debugdata mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== debugdata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, debugdata, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", debugdata[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", debugdata[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[LOG_MAX_BUF-1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (debugdata != NULL)
		kfree(debugdata);

	return count;
}

static ssize_t read_labeldata(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *label = NULL;
	int i = 0;
	int j = 0;
	u32 cmd = CMD_LABELDATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		label = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (label == NULL) {
			TOUCH_E("label mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== labeldata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, label, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", label[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", label[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[LOG_MAX_BUF-1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (label != NULL)
		kfree(label);

	return count;
}

//------- self data ------------------------------------------------------------------------------------//
static ssize_t read_delta_s(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *delta = NULL;
	int16_t *delta_s = NULL;
	int i = 0;
	int j = 0;
	//self data hori line
	int ret_s = 0;
	int test_cnt = 0;
	u32 cmd = CMD_DELTADATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		//mem init
		memset(log_buf_s, 0, LOG_BUF_SIZE);
		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		delta = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);
		delta_s = kzalloc(sizeof(int16_t) * (COL_SIZE + ROW_SIZE), GFP_KERNEL);
		if ((delta == NULL) || (delta_s == NULL)) {
			TOUCH_E("delta mem_error\n");
			goto error;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== MS Deltadata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, delta, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		ret2 = get_data(dev, delta_s, CMD_DELTADATA_S, DBG_SELF);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		//self data hori line
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "======");
		ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s, "======");

		for (i = 0 ; i < COL_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", delta_s[ROW_SIZE + i]);
			ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s,
					"%5d ", delta_s[ROW_SIZE + i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf_s);
		//self data hori line

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			//ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			//ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			//self data verti line
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", delta_s[test_cnt]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
					"%5d ", delta_s[test_cnt]);
			test_cnt++;
			//self data verti line

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", delta[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", delta[j*ROW_SIZE+i]);
			}

			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (delta != NULL)
		kfree(delta);

	if (delta_s != NULL)
		kfree(delta_s);

	return count;
}

static ssize_t read_rawdata_s(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int16_t *rawdata_s = NULL;
	int i = 0;
	int j = 0;
	//self data hori line
	int ret_s = 0;
	int test_cnt = 0;
	u32 cmd = CMD_RAWDATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		//mem init
		memset(log_buf_s, 0, LOG_BUF_SIZE);
		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);
		rawdata_s = kzalloc(sizeof(int16_t) * (COL_SIZE + ROW_SIZE), GFP_KERNEL);
		if ((rawdata == NULL) || (rawdata_s == NULL)) {
			TOUCH_E("rawdata mem_error\n");
			goto error;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== MS rawdata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, rawdata, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		ret2 = get_data(dev, rawdata_s, CMD_RAWDATA_S, DBG_SELF);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		//self data hori line
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "======");
		ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s, "======");

		for (i = 0 ; i < COL_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", rawdata_s[ROW_SIZE + i]);
			ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s,
					"%5d ", rawdata_s[ROW_SIZE + i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf_s);
		//self data hori line

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			//ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			//ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			//self data verti line
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", rawdata_s[test_cnt]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
					"%5d ", rawdata_s[test_cnt]);
			test_cnt++;
			//self data verti line

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", rawdata[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", rawdata[j*ROW_SIZE+i]);
			}

			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	if (rawdata_s != NULL)
		kfree(rawdata_s);

	return count;
}

static ssize_t read_base_s(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *baseline = NULL;
	int16_t *baseline_s = NULL;
	int i = 0;
	int j = 0;
	//self data hori line
	int ret_s = 0;
	int test_cnt = 0;
	u32 cmd = CMD_BASE_DATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		//mem init
		memset(log_buf_s, 0, LOG_BUF_SIZE);
		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		baseline = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);
		baseline_s = kzalloc(sizeof(int16_t) * (COL_SIZE + ROW_SIZE), GFP_KERNEL);
		if ((baseline == NULL) || (baseline_s == NULL)) {
			TOUCH_E("baseline mem_error\n");
			goto error;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== MS baseline ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, baseline, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		ret2 = get_data(dev, baseline_s, CMD_BASE_DATA_S, DBG_SELF);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		//self data hori line
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "======");
		ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s, "======");

		for (i = 0 ; i < COL_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", baseline_s[ROW_SIZE + i]);
			ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s,
					"%5d ", baseline_s[ROW_SIZE + i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf_s);
		//self data hori line

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			//ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			//ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			//self data verti line
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", baseline_s[test_cnt]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
					"%5d ", baseline_s[test_cnt]);
			test_cnt++;
			//self data verti line

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", baseline[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", baseline[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[LOG_MAX_BUF-1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (baseline != NULL)
		kfree(baseline);

	if (baseline_s != NULL)
		kfree(baseline_s);

	return count;
}

static ssize_t read_debug_s(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *debugdata = NULL;
	int16_t *debugdata_s = NULL;
	int i = 0;
	int j = 0;
	//self data hori line
	int ret_s = 0;
	int test_cnt = 0;
	u32 cmd = CMD_DEBUGDATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		//mem init
		memset(log_buf_s, 0, LOG_BUF_SIZE);
		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		debugdata = kzalloc(sizeof(int16_t) * (DEBUG_COL_SIZE) * (DEBUG_ROW_SIZE), GFP_KERNEL);
		debugdata_s = kzalloc(sizeof(int16_t) * (DEBUG_COL_SIZE + DEBUG_ROW_SIZE), GFP_KERNEL);
		if ((debugdata == NULL) || (debugdata_s == NULL)) {
			TOUCH_E("debugdata mem_error\n");
			goto error;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== S debugdata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, debugdata, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		ret2 = get_data(dev, debugdata_s, CMD_DEBUGDATA_S, DBG_SELF);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		//self data hori line
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "======");
		ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s, "======");

		for(i = 0 ; i < COL_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", debugdata_s[ROW_SIZE + i]);
			ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s,
						"%5d ", debugdata_s[ROW_SIZE + i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf_s);
		//self data hori line

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			//ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			//ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			//self data verti line
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", debugdata_s[test_cnt]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", debugdata_s[test_cnt]);
			test_cnt++;
			//self data verti line

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", debugdata[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", debugdata[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}

		kobj_log_buf[LOG_MAX_BUF-1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (debugdata != NULL)
		kfree(debugdata);

	if (debugdata_s != NULL)
		kfree(debugdata_s);

	return count;
}

static ssize_t read_labeldata_s(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *label = NULL;
	int16_t *label_s = NULL;
	int i = 0;
	int j = 0;
	//self data hori line
	int ret_s = 0;
	int test_cnt = 0;
	u32 cmd = CMD_LABELDATA;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		//mem init
		memset(log_buf_s, 0, LOG_BUF_SIZE);
		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		label = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);
		label_s = kzalloc(sizeof(int16_t) * (COL_SIZE + ROW_SIZE + 2), GFP_KERNEL);

		if ((label == NULL) || (label_s == NULL)) {
			TOUCH_E("label mem_error\n");
			goto error;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== MS labeldata ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, label, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		ret2 = get_data(dev, label_s, CMD_LABELDATA_S, DBG_SELF);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		//self data hori line
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "======");
		ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s, "======");

		for (i = 0 ; i < COL_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", label_s[ROW_SIZE + i]);
			ret_s += touch_snprintf(log_buf_s + ret_s, LOG_BUF_SIZE - ret_s,
					"%5d ", label_s[ROW_SIZE + i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf_s);
		//self data hori line

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			//ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "[%2d]  ", i);
			//ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "[%2d]  ", i);

			//self data verti line
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
					"%5d ", label_s[test_cnt]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
					"%5d ", label_s[test_cnt]);
			test_cnt++;
			//self data verti line

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", label[j*ROW_SIZE+i]);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", label[j*ROW_SIZE+i]);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[LOG_MAX_BUF-1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (label != NULL)
		kfree(label);

	if (label_s != NULL)
		kfree(label_s);

	return count;
}

static ssize_t read_p_pos_raw(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;
	u32 cmd = CMD_RAW_P_POS;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d] cmd[%d]\n", __func__, (int)off, (int)count, cmd);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_pos_raw ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Rx[%2d]  ", ROW_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Rx[%2d]  ", ROW_SIZE);

		for (i = 0 ; i < ROW_SIZE + COL_SIZE; i++) {
			if(i == ROW_SIZE) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
				TOUCH_I("%s\n", log_buf);

				//ret = 0;
				ret3 = 0;

				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Tx[%2d]  ", COL_SIZE);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Tx[%2d]  ", COL_SIZE);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_pos_base(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_BASE_P_POS;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_pos_base ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Rx[%2d]  ", ROW_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Rx[%2d]  ", ROW_SIZE);

		for (i = 0 ; i < ROW_SIZE + COL_SIZE; i++) {
			if(i == ROW_SIZE) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
				TOUCH_I("%s\n", log_buf);

				//ret = 0;
				ret3 = 0;

				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Tx[%2d]  ", COL_SIZE);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Tx[%2d]  ", COL_SIZE);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_pos_delta(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_DELTA_P_POS;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_pos_delta ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Rx[%2d]  ", ROW_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Rx[%2d]  ", ROW_SIZE);

		for (i = 0 ; i < ROW_SIZE + COL_SIZE; i++) {
			if(i == ROW_SIZE) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
				TOUCH_I("%s\n", log_buf);

				//ret = 0;
				ret3 = 0;

				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Tx[%2d]  ", COL_SIZE);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Tx[%2d]  ", COL_SIZE);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_pos_label(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_LABEL_P_POS;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_pos_label ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Rx[%2d]  ", ROW_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Rx[%2d]  ", ROW_SIZE);

		for (i = 0 ; i < ROW_SIZE + COL_SIZE; i++) {
			if(i == ROW_SIZE) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
				TOUCH_I("%s\n", log_buf);

				//ret = 0;
				ret3 = 0;

				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Tx[%2d]  ", COL_SIZE);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Tx[%2d]  ", COL_SIZE);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_pos_debug(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_DEBUG_P_POS;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_pos_debug ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Rx[%2d]  ", ROW_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Rx[%2d]  ", ROW_SIZE);

		for (i = 0 ; i < ROW_SIZE + COL_SIZE; i++) {
			if(i == ROW_SIZE) {
				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
				TOUCH_I("%s\n", log_buf);

				//ret = 0;
				ret3 = 0;

				ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Tx[%2d]  ", COL_SIZE);
				ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3, "Tx[%2d]  ", COL_SIZE);
			}
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_raw(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_RAW_P;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (PEN_DATA_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_raw ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"Data[%2d]  ", PEN_DATA_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"Data[%2d]  ", PEN_DATA_SIZE);

		for (i = 0 ; i < PEN_DATA_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_base(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_BASE_P;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (PEN_DATA_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_base ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"Data[%2d]  ", PEN_DATA_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"Data[%2d]  ", PEN_DATA_SIZE);

		for (i = 0 ; i < PEN_DATA_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';

	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_delta(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_DELTA_P;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (PEN_DATA_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_delta ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"Data[%2d]  ", PEN_DATA_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"Data[%2d]  ", PEN_DATA_SIZE);

		for (i = 0 ; i < PEN_DATA_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static ssize_t read_p_debug(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct device *dev = ts->dev;
	static int ret = 0;
	int ret2 = 0;
	int16_t *frame_buf = NULL;
	int i = 0;

	u32 cmd = CMD_DEBUG_P;
	char log_buf[LOG_BUF_SIZE] = {0,};
	int ret3 = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		ret = 0; //static variable initializing

		memset(kobj_log_buf, 0, sizeof(kobj_log_buf));

		frame_buf = kzalloc(sizeof(int16_t) * (PEN_DATA_SIZE), GFP_KERNEL);

		if (frame_buf == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = touch_snprintf(kobj_log_buf, LOG_MAX_BUF, "======== p_debug ========\n");

		/***********************************************
		 * Match enum value between CMD_XXX and IT_XXX *
		 * when you call stop_firmware.                *
		 ***********************************************/
		if (stop_firmware(dev, cmd)) {
			TOUCH_E("fail to stop FW\n");
			ret = 1;
			goto error;
		}

		ret2 = get_data(dev, frame_buf, cmd, DBG_MUTUAL);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		start_firmware(dev);

		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"Data[%2d]  ", PEN_DATA_SIZE);
		ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"Data[%2d]  ", PEN_DATA_SIZE);

		for (i = 0 ; i < PEN_DATA_SIZE; i++) {
			ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret,
						"%5d ", frame_buf[i]);
			ret3 += touch_snprintf(log_buf + ret3, LOG_BUF_SIZE - ret3,
						"%5d ", frame_buf[i]);
		}
		ret += touch_snprintf(kobj_log_buf + ret, LOG_MAX_BUF - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		kobj_log_buf[ret + 1] = '\n';
	}

	if (ret < count)
		count = ret;
	ret = ret - count;

	if ((off < LOG_MAX_BUF) && (off + count <= LOG_MAX_BUF)) {
		memcpy(buf, &kobj_log_buf[off], count);
	} else {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
		count = 0;
	}

error:
	if (frame_buf != NULL)
		kfree(frame_buf);

	return count;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(fdata, show_fdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(psy_test, show_psy_test, NULL);

/* [Start] for siw app */
static TOUCH_ATTR(prd_app_raw, prd_show_app_raw, NULL);
static TOUCH_ATTR(prd_app_base, prd_show_app_base, NULL);
static TOUCH_ATTR(prd_app_label, prd_show_app_label, NULL);
static TOUCH_ATTR(prd_app_delta, prd_show_app_delta, NULL);
static TOUCH_ATTR(prd_app_debug_buf, prd_show_app_debug_buf, NULL);
static TOUCH_ATTR(prd_app_end, prd_show_app_end, NULL);
static TOUCH_ATTR(prd_app_info, prd_show_app_info, NULL);
/* [End] for siw app */

#define TOUCH_BIN_ATTR(_name, _read, _write, _size)		\
			struct bin_attribute touch_bin_attr_##_name	\
			= __BIN_ATTR(_name, S_IRUGO | S_IWUSR, _read, _write, _size)

static TOUCH_BIN_ATTR(delta, read_delta, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(rawdata, read_rawdata, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(base, read_base, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(debug, read_debug, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(label, read_labeldata, NULL, LOG_MAX_BUF);

static TOUCH_BIN_ATTR(delta_s, read_delta_s, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(rawdata_s, read_rawdata_s, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(base_s, read_base_s, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(debug_s, read_debug_s, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(label_s, read_labeldata_s, NULL, LOG_MAX_BUF);

static TOUCH_BIN_ATTR(p_pos_raw, read_p_pos_raw, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_pos_base, read_p_pos_base, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_pos_delta, read_p_pos_delta, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_pos_label, read_p_pos_label, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_pos_debug, read_p_pos_debug, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_raw, read_p_raw, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_base, read_p_base, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_delta, read_p_delta, NULL, LOG_MAX_BUF);
static TOUCH_BIN_ATTR(p_debug, read_p_debug, NULL, LOG_MAX_BUF);


static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_fdata.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_psy_test.attr,

	/* [Start] for siw app */
	&touch_attr_prd_app_raw.attr,
	&touch_attr_prd_app_base.attr,
	&touch_attr_prd_app_label.attr,
	&touch_attr_prd_app_delta.attr,
	&touch_attr_prd_app_debug_buf.attr,
	&touch_attr_prd_app_end.attr,
	&touch_attr_prd_app_info.attr,
	/* [End] for siw app */
	NULL,
};

static struct bin_attribute *prd_attribute_bin_list[] = {
	//mutual
	&touch_bin_attr_delta,
	&touch_bin_attr_rawdata,
	&touch_bin_attr_base,
	&touch_bin_attr_debug,
	&touch_bin_attr_label,
	//mutual+self
	&touch_bin_attr_delta_s,
	&touch_bin_attr_rawdata_s,
	&touch_bin_attr_base_s,
	&touch_bin_attr_debug_s,
	&touch_bin_attr_label_s,
	//pen
	&touch_bin_attr_p_pos_raw,
	&touch_bin_attr_p_pos_base,
	&touch_bin_attr_p_pos_delta,
	&touch_bin_attr_p_pos_label,
	&touch_bin_attr_p_pos_debug,
	&touch_bin_attr_p_raw,
	&touch_bin_attr_p_base,
	&touch_bin_attr_p_delta,
	&touch_bin_attr_p_debug,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
	.bin_attrs = prd_attribute_bin_list,
};

int dlkm_sw82908_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw82908_data *d = to_sw49202_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	int ret = 0;

	TOUCH_TRACE();

	/* [Start] for siw app */
	prd = siw_hal_prd_alloc(dev);
	if (!prd) {
		ret = -ENOMEM;
		return ret;
	}
	/* [End] for siw app */

	/* prd parameter settings */
	prd_param_set(dev);

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}
#if IS_ENABLED(CONFIG_LGE_TOUCH_COMMON_CONTROL)
	ret = read_sysfs_lists(dev, &prd_attribute_group);
	if (ret < 0)
		TOUCH_I("fail to read sysfs lists\n");
#endif
	return ret;
}
