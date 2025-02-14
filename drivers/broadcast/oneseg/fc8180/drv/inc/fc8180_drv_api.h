/*****************************************************************************
     Copyright(c) 2013 FCI Inc. All Rights Reserved

     File name : fc8180_drv_api.h

     Description : header of fc8180 driver api

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

#ifndef __FC8180_API_H__
#define __FC8180_API_H__

#ifdef __cplusplus
extern "C" {
#endif

struct fc8180Status_t{
    int lock;
    int ber;
    int per;
    int ErrTSP;
    int TotalTSP;
    int antenna_level;

    int tmccinfo;
    int receive_status;
    int rssi;
    int scan_status;
    int sysinfo;
    int cn;

    int ber_a;
    int per_a;
    int layerinfo_a;
    int total_tsp_a;

    int ber_b;
    int per_b;
    int layerinfo_b;
    int total_tsp_b;

    int ber_c;
    int per_c;
    int layerinfo_c;
    int total_tsp_c;

    int fullseg_oneseg_flag;
    int antenna_level_fullseg;
    int antenna_level_oneseg;
    int agc;
    int ber_1seg;
    int per_1seg;
    int total_tsp_1seg;
    int err_tsp_1seg;
    int ber_fullseg;
    int per_fullseg;
    int total_tsp_fullseg;
    int err_tsp_fullseg;
};

int tunerbb_drv_fc8180_init(int mode);
int tunerbb_drv_fc8180_stop(void);
int tunerbb_drv_fc8180_set_channel(s32 f_rf, u16 mode);
int tunerbb_drv_fc8180_Get_SyncStatus(void);
int tunerbb_drv_fc8180_Get_SignalInfo(struct fc8180Status_t *st, s32 brd_type);
void tunerbb_drv_fc8180_set_user_stop(int ustop);
extern int get_fc8180_ustop_state(void);

#ifdef __cplusplus
};
#endif

#endif
