/*****************************************************************************
    Copyright(c) 2013 FCI Inc. All Rights Reserved

    File name : fc8300_ppi.c

    Description : source of EBI2/LCD interface

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
#include "linux/io.h"

#include "../inc/fci_types.h"
#include "../inc/fc8300_regs.h"
#include "../inc/fci_oal.h"

#define BBM_BASE_ADDR       0x00

#define PPI_LEN             0x10
#define PPI_REG             0x20
#define PPI_THR             0x30
#define PPI_READ            0x40
#define PPI_WRITE           0x00
#define PPI_AINC            0x80

#define FC8300_PPI_REG_OUT(x)      outb(x, BBM_BASE_ADDR)
#define FC8300_PPI_REG_IN      inb(BBM_BASE_ADDR)

s32 fc8300_ppi_init(HANDLE handle, u16 param1, u16 param2)
{
    OAL_CREATE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_byteread(HANDLE handle, DEVICEID devid, u16 addr, u8 *data)
{
    u16 length = 1;
    u8 command = PPI_READ;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    *data = FC8300_PPI_REG_IN;

    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_wordread(HANDLE handle, DEVICEID devid, u16 addr, u16 *data)
{
    u16 length = 2;
    u8 command = PPI_AINC | PPI_READ;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    *data = FC8300_PPI_REG_IN;
    *data |= FC8300_PPI_REG_IN << 8;

    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_longread(HANDLE handle, DEVICEID devid, u16 addr, u32 *data)
{
    u16 length = 4;
    u8 command = PPI_AINC | PPI_READ;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    *data = FC8300_PPI_REG_IN;
    *data |= FC8300_PPI_REG_IN << 8;
    *data |= FC8300_PPI_REG_IN << 16;
    *data |= FC8300_PPI_REG_IN << 24;

    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_bulkread(HANDLE handle, DEVICEID devid,
    u16 addr, u8 *data, u16 length)
{
    s32 i;
    u8 command = PPI_AINC | PPI_READ;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    for (i = 0; i < length; i++)
        data[i] = FC8300_PPI_REG_IN;

    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_bytewrite(HANDLE handle, DEVICEID devid, u16 addr, u8 data)
{
    u16 length = 1;
    u8 command = PPI_WRITE;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    FC8300_PPI_REG_OUT(data);

    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_wordwrite(HANDLE handle, DEVICEID devid, u16 addr, u16 data)
{
    u16 length = 2;
    u8 command = PPI_AINC | PPI_WRITE;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    FC8300_PPI_REG_OUT(data & 0xff);
    FC8300_PPI_REG_OUT((data & 0xff00) >> 8);
    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_longwrite(HANDLE handle, DEVICEID devid, u16 addr, u32 data)
{
    u16 length = 4;
    u8 command = PPI_AINC | PPI_WRITE;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    FC8300_PPI_REG_OUT(data &  0x000000ff);
    FC8300_PPI_REG_OUT((data & 0x0000ff00) >> 8);
    FC8300_PPI_REG_OUT((data & 0x00ff0000) >> 16);
    FC8300_PPI_REG_OUT((data & 0xff000000) >> 24);
    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_bulkwrite(HANDLE handle, DEVICEID devid,
    u16 addr, u8 *data, u16 length)
{
    s32 i;
    u8 command = PPI_AINC | PPI_WRITE;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(length & 0xff);

    for (i = 0; i < length; i++)
        FC8300_PPI_REG_OUT(data[i]);

    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_dataread(HANDLE handle, DEVICEID devid,
    u16 addr, u8 *data, u32 length)
{
    s32 i;
    u8 command = PPI_READ | PPI_THR;

    OAL_OBTAIN_SEMAPHORE();

    FC8300_PPI_REG_OUT((addr & 0xff00) >> 8);
    FC8300_PPI_REG_OUT(addr & 0xff);
    FC8300_PPI_REG_OUT(command);
    FC8300_PPI_REG_OUT(0);

    for (i = 0; i < length; i++)
        data[i] = FC8300_PPI_REG_IN;

    OAL_RELEASE_SEMAPHORE();

    return BBM_OK;
}

s32 fc8300_ppi_deinit(HANDLE handle)
{
    OAL_DELETE_SEMAPHORE();

    return BBM_OK;
}
