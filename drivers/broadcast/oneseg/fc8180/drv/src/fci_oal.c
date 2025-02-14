/*****************************************************************************
    Copyright(c) 2014 FCI Inc. All Rights Reserved

    File name : fci_oal.c

    Description : source of OS adaptation layer

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
#include <linux/kernel.h>
#include <linux/delay.h>

#include "../inc/fc8180_regs.h"
#include "../inc/fci_types.h"

void print_log(HANDLE handle, s8 *fmt, ...)
{
#ifdef FEATURE_DEBUG
    va_list ap;
    char str[256];

    va_start(ap, fmt);
    vsprintf(str, fmt, ap);
    printk(KERN_ERR"%s", str);

    va_end(ap);
#endif
}


void ms_wait(s32 ms)
{
    msleep(ms);
}


/* Write your own mutual exclusion method */
void OAL_CREATE_SEMAPHORE(void)
{
    /* called in driver initialization */
}

void OAL_DELETE_SEMAPHORE(void)
{
    /* called in driver deinitializaton */
}

void OAL_OBTAIN_SEMAPHORE(void)
{

}

void OAL_RELEASE_SEMAPHORE(void)
{

}

