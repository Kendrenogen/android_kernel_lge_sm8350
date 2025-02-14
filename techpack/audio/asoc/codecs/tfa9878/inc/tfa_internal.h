/*
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 * Copyright 2020 GOODIX, All Rights Reserved.
 */
/*
 * internal functions for TFA layer (not shared with SRV and HAL layer!)
 */

#ifndef __TFA_INTERNAL_H__
#define __TFA_INTERNAL_H__

#include "tfa_dsp_fw.h"
/* #include <sound/tfa_ext.h> */
#include "tfa_ext.h"

#ifdef QPLATFORM
#if defined(TFA_SET_EXT_INTERNALLY)
#include <dsp/q6afe-v2.h>
#endif
#if defined(TFADSP_DSP_MSG_PACKET_STRATEGY)
#include <ipc/apr_tal.h>
#endif
#endif

#if __GNUC__ >= 4
  #define TFA_INTERNAL __attribute__((visibility("hidden")))
#else
  #define TFA_INTERNAL
#endif

#define TFA98XX_GENERIC_SLAVE_ADDRESS 0x1C

#define MAX_HANDLES 4
/* max. length of a alsa mixer control name */
#define MAX_CONTROL_NAME        48

#if defined(TFADSP_DSP_MSG_PACKET_STRATEGY)
#ifdef QPLATFORM
// in case of CONFIG_MSM_QDSP6_APRV2_GLINK/APRV3_GLINK, with smaller APR_MAX_BUF (512)
// in other cases, with safe size (4096)
#define APR_RESIDUAL_SIZE	60
#define APR_MAX_BUF2 ((APR_MAX_BUF > 512) ? 4096 : APR_MAX_BUF)
#define MAX_PKT_MSG_SIZE	(APR_MAX_BUF2-APR_RESIDUAL_SIZE) // 452 or 4036
// data size in packet by excluding header (packet_id:2, packet_size:2)
// #define STANDARD_PACKET_SIZE	(MAX_APR_MSG_SIZE - 4) // 448 or 4032
#else
#define MAX_PKT_MSG_SIZE	65536 // 64*1024
#endif
#endif

enum instream_state {
	BIT_PSTREAM = 1, /* b0 */
	BIT_CSTREAM = 2, /* b1 */
	BIT_SAMSTREAM = 4 /* b2 */
};

TFA_INTERNAL enum tfa98xx_error tfa98xx_check_rpc_status
	(struct tfa_device *tfa, int *p_rpc_status);
TFA_INTERNAL enum tfa98xx_error tfa98xx_wait_result
	(struct tfa_device *tfa, int waitRetryCount);

#if defined(TFADSP_DSP_BUFFER_POOL)
enum tfa98xx_error tfa_buffer_pool(struct tfa_device *tfa,
	int index, int size, int control);
int tfa98xx_buffer_pool_access(int r_index,
	size_t g_size, uint8_t **buf, int control);
#endif

struct tfa_device *tfa98xx_get_tfa_device_from_index(int index);
struct tfa_device *tfa98xx_get_tfa_device_from_channel(int channel);
int tfa98xx_count_active_stream(int stream_flag);

int tfa_ext_event_handler(enum tfadsp_event_en tfadsp_event);

void tfa_set_ipc_loaded(int status);
int tfa_get_ipc_loaded(void);

#define TFA_LOG_MAX_COUNT	4
#if defined(TFA_BLACKBOX_LOGGING)
int tfa_set_blackbox(int enable);
enum tfa98xx_error tfa_configure_log(int enable);
enum tfa98xx_error tfa_update_log(void);
#endif /* TFA_BLACKBOX_LOGGING */

#endif /* __TFA_INTERNAL_H__ */
