#ifndef BASEDRIVER_H
#define BASEDRIVER_H

#if defined(__cplusplus)
# define EXTERN_C extern
extern "C" {
# include "servto.h"
}
#else
# define EXTERN_C extern
# include "servto.h"
#endif

#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>

/*

scheme

	periph_irq() {
		driver->cycle() // hardcode
	}

	timer_irq() {
		// NOP, just wake up
	}

	disable_all_periph() {
		foreach (base : drivers) {
			mask |= base->resman
		}
		foreach (res : resources) {
			if (!(res & mask))
				disable_res()
		}
	}

	main() {
		driver->init()
		driver->registerQue()

		foreach (base : drivers) {
			base->cycle()
		}
		infinite() {
			foreach (timeout : to_que) {
				base = timeout->data
				base->cycle()
			}
			while (busy) {
				busy = false
				foreach (base : drivers) {
					busy |= base->busy
				}
			}
			disable_all_periph()
			foreach (base : drivers) {
				if (base->dataIsReady) {
					base->getData()
				}
			}
			timer->setTimeout(to_poll(to_que))
			if (timer->timeout) {
				sleep()
			}
		}
	}

*/

typedef enum {
	SIGMODE_Enable	= 0x01,
	SIGMODE_Cyclic	= 0x02,
	SIGMODE_Event	= 0x04,
} SignalMode_e;

#define SIGBITSET 1

#pragma pack (push,1)

typedef union {
	struct {
		uint8_t status : 1;
		uint8_t cyclic : 1;
		uint8_t event : 1;
	};
	uint8_t allbits;
} SignalMode_t;

#pragma pack (pop)


typedef enum {
	RESMAN_SUP_5V 	= (1 << 0),
	RESMAN_SUP_5V_P = (1 << 1),
	RESMAN_SUP_3_3V = (1 << 2),
	RESMAN_DMA1 	= (1 << 3),
	RESMAN_DMA2 	= (1 << 4),
	RESMAN_GPIOA 	= (1 << 5),
	RESMAN_GPIOB 	= (1 << 6),
	RESMAN_GPIOC 	= (1 << 7),
	RESMAN_GPIOD 	= (1 << 8),
	RESMAN_GPIOE 	= (1 << 9),
} Resource_e;

typedef struct {
	union {
		struct {
			uint32_t sup5 : 1;
			uint32_t sup5_p : 1;
			uint32_t sup3_3 : 1;
			uint32_t dma1 : 1;
			uint32_t dma2 : 1;
			uint32_t gpioa : 1;
			uint32_t gpiob : 1;
			uint32_t gpioc : 1;
			uint32_t gpiod : 1;
			uint32_t gpioe : 1;
		};
		uint32_t allbits;
	};
} ResourceManager_t;


typedef struct BaseDriver_s BaseDriver_t;

typedef void (*basedrv_cycle)(BaseDriver_t *base, void *caller);
typedef int (*basedrv_getId)(BaseDriver_t *base);
typedef void (*basedrv_resetTs)(BaseDriver_t *base);
typedef void (*basedrv_callAfter)(BaseDriver_t *base, uint32_t timeout);

struct BaseDriver_s {
	// public
	BaseDriver_t *list;			//!< linked list (next)
	void *user;					//!< some user data
	ResourceManager_t resman;	//!< for currently used resources only
	uint8_t enabled : 1;		//!<
	uint8_t busy : 1;			//!< sets on periph waiting; clears on periph interrupt
	uint8_t dataIsReady : 1;	//!< sets when data is ready; clears on getData
	uint8_t eventIsReady : 1;	//!< sets when event was occurred; clears on getData
	uint8_t *supBusy;			//!< shared supply is busy | appointed by derived
	basedrv_cycle cycle;		//!< | appointed by derived
	basedrv_getId getId;		//!< | appointed by base
	basedrv_resetTs resetTs;	//!< | appointed by derived | can be NULL

	// protected
	void *priv;			//!< derived class private data
	uint8_t state;		//!< derived class internal state
	basedrv_callAfter callAfter;	//!< | appointed by base

	// private
	to_item_t toitem;	//!<
	to_que_t *mytoque;	//!<
};

#define dToB(d) ((BaseDriver_t*)(d))
#define dToV(d) ((void*)(d))

EXTERN_C uint64_t prg_locked_getsystick(void);
EXTERN_C uint32_t prg_gettimestamp(void);
EXTERN_C void prg_delay_us(uint32_t d);
EXTERN_C void prg_delay_locked_us(uint32_t d);
EXTERN_C void prg_enablesupply(Resource_e res);

EXTERN_C void basedriver_init(BaseDriver_t *base);
EXTERN_C void basedriver_registerQue(BaseDriver_t *base, to_que_t *toque, int id);
EXTERN_C uint8_t basedriver_setSupBusy(BaseDriver_t *base, uint8_t state);

#endif // BASEDRIVER_H
