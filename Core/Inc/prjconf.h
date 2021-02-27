#ifndef PRJCONF_H
#define PRJCONF_H

#include <stdint.h>

#define DEFU8			1
#define DEFS8			1
#define DEFU16			1
#define DEFS16			1
#define DEFU32			1
#define DEFS32			1
#define DEFU64			1
#define DEFS64			1
#define DEFF32			1
#define DEFF64			1

extern uint64_t prg_locked_getsystick(void);
extern void prg_delay_us(uint32_t d);
extern void prg_setrtc(uint32_t timeval20);
extern uint32_t prg_gettimestamp(void);

#define GETSYSTICK prg_locked_getsystick

#endif // PRJCONF_H
