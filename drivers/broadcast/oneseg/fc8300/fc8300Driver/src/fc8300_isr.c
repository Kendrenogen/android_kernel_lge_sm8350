/*****************************************************************************
    Copyright(c) 2013 FCI Inc. All Rights Reserved

    File name : fc8300_isr.c

    Description : source of interrupt service routine

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

    History :
    ----------------------------------------------------------------------
*******************************************************************************/
#include "../inc/fci_types.h"
#include "../inc/fc8300_regs.h"
#include "../inc/fci_hal.h"
#ifndef BBM_I2C_TSIF
#include "../inc/fc8300_drv_api.h"
#endif

s32 (*fc8300_ac_callback)(u32 userdata, u8 bufid, u8 *data, s32 length) = NULL;
s32 (*fc8300_ts_callback)(u32 userdata, u8 bufid, u8 *data, s32 length) = NULL;

u32 fc8300_ac_user_data;
u32 fc8300_ts_user_data;

#ifndef BBM_I2C_TSIF
static u8 ts_buffer[188 * 320];

static void fc8300_data(HANDLE handle, DEVICEID devid, u8 buf_int_status)
{
    u32 size = 0;

    if (buf_int_status == 1) {
        size = TS0_BUF_LENGTH;

            bbm_data(handle, devid,
                BBM_TS0_DATA, &ts_buffer[0], size);

            if (fc8300_ts_callback)
                (*fc8300_ts_callback)(fc8300_ts_user_data,
                    0, &ts_buffer[0], size);
    }
}
#endif /* #ifndef BBM_I2C_TSIF */

#ifdef BBM_AUX_INT
static void fc8300_aux_int(HANDLE handle, DEVICEID devid, u8 aux_int_status)
{
    if (aux_int_status & AUX_INT_TMCC_INT_SRC)
        ;

    if (aux_int_status & AUX_INT_TMCC_INDTPS_SRC)
        ;

    if (aux_int_status & AUX_INT_AC_PREFRM_SRC)
        ;

    if (aux_int_status & AUX_INT_AC_EWISTAFLAG_SRC)
        ;

    if (aux_int_status & AUX_INT_SYNC_RELATED_INT) {
        u8 sync = 0;
        bbm_byte_read(handle, DIV_MASTER, BBM_SYS_MD_INT_CLR, &sync);

        if (sync) {
            bbm_byte_write(handle, DIV_MASTER, BBM_SYS_MD_INT_CLR,
                                    sync);

            if (sync & SYS_MD_NO_OFDM_DETECT)
                ;

            if (sync & SYS_MD_RESYNC_OCCUR)
                ;

            if (sync & SYS_MD_TMCC_LOCK)
                ;

            if (sync & SYS_MD_A_LAYER_BER_UPDATE)
                ;

            if (sync & SYS_MD_B_LAYER_BER_UPDATE)
                ;

            if (sync & SYS_MD_C_LAYER_BER_UPDATE)
                ;

            if (sync & SYS_MD_BER_UPDATE)
                ;
        }
    }

    if (aux_int_status & AUX_INT_GPIO_INT_CLEAR)
        ;

    if (aux_int_status & AUX_INT_FEC_RELATED_INT) {
        u8 fec = 0;
        bbm_byte_read(handle, DIV_MASTER, BBM_FEC_INT_CLR, &fec);

        if (fec) {
            bbm_byte_write(handle, DIV_MASTER, BBM_FEC_INT_CLR,
                                fec);

            if (fec & FEC_INT_IRQ_A_TS_ERROR)
                ;

            if (fec & FEC_INT_IRQ_B_TS_ERROR)
                ;

            if (fec & FEC_INT_IRQ_C_TS_ERROR)
                ;
        }
    }

    if (aux_int_status & AUX_INT_AUTO_SWITCH) {
        u8 auto_switch = 0;
        bbm_byte_read(handle, DIV_MASTER, BBM_OSS_MNT, &auto_switch);

        if (auto_switch & AUTO_SWITCH_1_SEG) /* 1-SEG */
            ;
        else /* 12-SEG */
            ;
    }
}
#endif

#ifndef BBM_I2C_TSIF
extern unsigned int overrun_count;
#endif
extern struct fc8300Status_t st;
extern unsigned int irq_cnt;

void fc8300_isr(HANDLE handle)
{
#ifdef BBM_AUX_INT
    u8 aux_int_status = 0;
#endif

#ifndef BBM_I2C_TSIF
    static unsigned int overrun_check_count = 0;
    u8 buf_int_status = 0;
    bbm_byte_read(handle, DIV_MASTER, BBM_BUF_STATUS_CLEAR,
                    &buf_int_status);
    if (buf_int_status) {
        bbm_byte_write(handle, DIV_MASTER,
                BBM_BUF_STATUS_CLEAR, buf_int_status);

        fc8300_data(handle, DIV_MASTER, buf_int_status);

        buf_int_status = 0;
        bbm_byte_read(handle, DIV_MASTER, BBM_BUF_STATUS_CLEAR,
                        &buf_int_status);
        if (buf_int_status) {
            bbm_byte_write(handle, DIV_MASTER,
                    BBM_BUF_STATUS_CLEAR, buf_int_status);

            fc8300_data(handle, DIV_MASTER, buf_int_status);
        }


        irq_cnt++;
            if((overrun_check_count%20) == 0)
            {
            tunerbb_drv_fc8300_Get_SignalInfo(&st, ISDBT_13SEG);
            }
            overrun_check_count++;
    }
#endif

#ifdef BBM_AUX_INT
    bbm_byte_read(handle, DIV_MASTER, BBM_AUX_STATUS_CLEAR,
                    &aux_int_status);

    if (aux_int_status) {
        bbm_byte_write(handle, DIV_MASTER,
                BBM_AUX_STATUS_CLEAR, aux_int_status);

        fc8300_aux_int(handle, DIV_MASTER, aux_int_status);
    }
#endif
}

