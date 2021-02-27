#ifndef COMBUS_H
#define COMBUS_H

#include <stdint.h>

enum {
	cbf_Params = 1,
	cbf_Data,
	cbf_Cmd
};

enum {
	cbr_Cyclic,
	cbr_Important,
	cbr_Confirm
};

enum {
	cbp_sens_T = 1,
	cbp_sens_Hum,
	cbp_accelShock,
	cbp_accelVib,
	cbp_sens_Open = 5,
	cbp_sens_1wT = 12,
	cbp_bat = 14,
	cbp_cmnp,

	cbp2_sens_T = 100+1,
	cbp2_sens_Hum,
	cbp2_accelShock,
	cbp2_accelVib,
	cbp2_sens_1wT = 100+12,
	cbp2_bat = 100+14,
};

enum {
	cbd_sens_T = 1,
	cbd_sens_Hum,
	cbd_accelShock,
	cbd_accelVib,
	cbd_sens_Open = 5,
	cbd_sens_1wT = 12,
	cbd_sensInfo_1wT = 14,
	cbd_bat = 16,
	cbd_getTime,
	cbd_modInfo,
	cbd_echo,
	cbd_allprm,
	cbd_accelRaw = 21,
	cbd_wt,
};

enum {
	cbc_sens_T = 1,
	cbc_sens_Hum,
	cbc_accelShock,
	cbc_accelVib,
	cbc_sens_Open = 5,
	cbc_sens_1wT = 12,
	cbc_sensInfo_1wT = 14,
	cbc_bat = 16,
	cbc_getTime,
	cbc_modInfo,
	cbc_echo,
	cbc_allprm,
	cbc_accelRaw,
	cbc_wt,

	cbc_reset = 100,
	cbc_delAllTs,
	cbc_applyPrms,
	cbc_saveApply,
	cbc_defaultPrms,
	cbc_setTime
};

#pragma pack (push,1)

typedef struct {
	uint8_t frame;
	uint8_t reason;
	uint8_t num;
	uint8_t ack;
} CombusHeader_t;

#pragma pack (pop)


#endif // COMBUS_H
