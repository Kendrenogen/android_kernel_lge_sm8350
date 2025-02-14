#ifndef _H_LGE_DP_
#define _H_LGE_DP_

#ifdef CONFIG_LGE_DISPLAY_NOT_SUPPORT_DISPLAYPORT
struct dp_noti_dev {
	const char 		*name;
	struct device 	*dev;
	int 			state;
};

int dp_noti_register(struct dp_noti_dev *ndev);
void dp_noti_unregister(struct dp_noti_dev *ndev);
void dp_noti_set_state(struct dp_noti_dev *ndev, int state);
void lge_dp_set_id(unsigned int id);
#else
#define LGE_DP_VSYNC_SWAP_VID		0x0000003b
#define LGE_DP_SPECIFIC_VID		0x1F29
struct dp_display;

void lge_dp_drv_init(struct dp_display *dp_display);
void lge_set_dp_hpd(struct dp_display *dp_display, int value);
void lge_dp_set_id(unsigned int id);
#endif
#endif // _H_LGE_DP_
