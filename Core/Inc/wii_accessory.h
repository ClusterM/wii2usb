#ifndef _WII_ACCESSORY_H_
#define _WII_ACCESSORY_H_

#define WII_ACCESSORY_ADDRESS 0x52 << 1
#define WII_ACCESSORY_TIMEOUT 100

#define WII_ACCESSORY_OK 0
#define WII_ACCESSORY_NOT_CONNECTED 1
#define WII_ACCESSORY_POLL_ERROR 2
#define WII_ACCESSORY_INVALID_DATA 3
#define WII_ACCESSORY_UNKNOWN_FORMAT 4

#define D_BTN_R      1
#define D_BTN_START  2
#define D_BTN_HOME   3
#define D_BTN_SELECT 4
#define D_BTN_L      5
#define D_BTN_DOWN   6
#define D_BTN_RIGHT  7

#define D_BTN_UP     0
#define D_BTN_LEFT   1
#define D_BTN_ZR     2
#define D_BTN_X      3
#define D_BTN_A      4
#define D_BTN_Y      5
#define D_BTN_B      6
#define D_BTN_ZL     7

typedef struct
{
	uint8_t data_format;
	uint8_t device_type;
	int8_t jx, jy, rx, ry, acc_x, acc_y, acc_z;
	uint8_t tl, tr;
	uint8_t dpad_left, dpad_right, dpad_up, dpad_down;
	uint8_t button_a, button_b, button_x, button_y, button_select, button_start, button_home, button_l, button_r, button_zl, button_zr;
	uint8_t wtf;
} Wii_Accessory_Data;

uint8_t wii_accessory_poll(I2C_HandleTypeDef* hi2c, Wii_Accessory_Data* wii_accessory_data);

#endif
