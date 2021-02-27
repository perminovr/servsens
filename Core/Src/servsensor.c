
#include "servsensor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "retain.h"
#include "uartdrv.h"
#include "combus.h"

const char version[][16] = {
	#include "version"
};

#define prg_SystemCoreClock 8 * 1000000

static void handleSysTimerOnWakeUp(void);
static void runSysTimer(void);
static void setSysTimer(uint32_t timeout);


enum {
	drvId_Retain,
	drvId_Uart,
	drvId_Din,
	drvId_Adc,
	drvId_St1w,
	drvId_Sht,
	drvId_Wt,
};


typedef struct {
	to_que_t toque;
	BaseDriver_t *list;

	ResourceManager_t resman;
	uint8_t supBusy;
	uint8_t wrn;
	uint8_t sup5;

	Retain_t retain;
	UartDrv_t uart;

	DinDrv_t din;
	AdcDrv_t adc;
	SensT1wDrv_t st1w;
	ShtDrv_t sht;
	WtDrv_t wt;
} Module_t;
static Module_t *module;


static void handle5VEvents(uint8_t sup5)
{
	module->sup5 = sup5;

	UartDrvParams_t uartPrms = module->uart.prms;
	if (sup5 != uartPrms.enabled) {
		uartPrms.enabled = sup5;
		uartdrv_setParams(&module->uart, &uartPrms);
	}
}

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
	if (module->sup5)
		wtdrv_cycle(&module->wt, hrtc->Instance);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartdrv_cycle(&module->uart, huart->Instance);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcdrv_cycle(&module->adc, hadc->Instance);
	handle5VEvents(module->adc.data.sup5);
}

void DMA1_Channel5_IRQHandler(void)
{
	shtdrv_cycle(&module->sht, DMA1);
}

void DMA1_Channel6_IRQHandler(void)
{
	// write
	retain_cycle(&module->retain, DMA1);
}


typedef enum {
	hc_adc_changed = (1<<0),
	hc_din_changed = (1<<1),
	hc_st1w_changed = (1<<2),
	hc_sht_changed = (1<<3),
	hc_wt_changed = (1<<4),
} HcChanges_e;


static void fillDatabaseParams(HcChanges_e chg)
{
	RetainDataBase_t *db = &module->retain.db;
	db->cyclePeriod = 0;
	if (chg & hc_adc_changed) memcpy(&db->adc.prms, &module->adc.prms, sizeof(AdcDrvParams_t));
	if (chg & hc_din_changed) memcpy(&db->din.prms, &module->din.prms, sizeof(DinDrvParams_t));
	if (chg & hc_st1w_changed) memcpy(&db->st1w.prms, &module->st1w.prms, sizeof(SensT1wDrvParams_t));
	if (chg & hc_sht_changed) memcpy(&db->sht.prms, &module->sht.prms, sizeof(ShtDrvParams_t));
	if (chg & hc_wt_changed) memcpy(&db->wt.prms, &module->wt.prms, sizeof(WtDrvParams_t));
	retain_triggerUpdate(&module->retain);
}


static inline void resetAllParams(void){

	retain_resetDefault(&module->retain);
	dindrv_setParams(&module->din, &module->retain.db.din.prms);
	adcdrv_setParams(&module->adc, &module->retain.db.adc.prms);
	senst1wdrv_setParams(&module->st1w, &module->retain.db.st1w.prms);
	shtdrv_setParams(&module->sht, &module->retain.db.sht.prms);
}


static inline uint8_t wrnum(uint8_t *wrn)
{
	uint8_t num = (*wrn)++;
	if (!(*wrn)) (*wrn) = 1;
	return num;
}


static void handleCombus(void)
{
	UartDrvData_t recvd;
	uartdrv_getData(&module->uart, &recvd, 1);
	
}


void prg_enablesupply(Resource_e res)
{
	uint32_t rbits;
	GPIO_InitTypeDef init = {0};
	init.Mode = GPIO_MODE_OUTPUT_PP;
	init.Pull = GPIO_PULLUP;
	init.Speed = GPIO_SPEED_FREQ_LOW;

	rbits = res & RESMAN_SUP_5V;
	if (rbits) {
		if ( ~(module->resman.allbits) & rbits ) {
			init.Pin = SUPPLY_EN_5_PIN;
			SUPPLY_EN_5_CLOCK_CMD(ENABLE);
			HAL_GPIO_Init(SUPPLY_EN_5_PORT, &init);
			HAL_GPIO_WritePin(SUPPLY_EN_5_PORT, init.Pin, 0); // enable
			module->resman.allbits |= rbits;
			prg_delay_locked_us(SUPPLY_STARTUP_DELAY_MS*1000);
		}
	}

	rbits = res & (RESMAN_SUP_5V_P | RESMAN_SUP_3_3V);
	if (rbits) {
		if ( ~(module->resman.allbits) & rbits ) {
			init.Pin = 0;
			init.Pin |= ((res & RESMAN_SUP_5V_P)? SUPPLY_5V_P_PIN : 0);
			init.Pin |= ((res & RESMAN_SUP_3_3V)? SUPPLY_3_3V_PIN : 0);
			SUPPLY_P_CLOCK_CMD(ENABLE);
			HAL_GPIO_Init(SUPPLY_P_PORT, &init);
			HAL_GPIO_WritePin(SUPPLY_P_PORT, init.Pin, 0); // enable
			module->resman.allbits |= rbits;
			prg_delay_locked_us(SUPPLY_STARTUP_DELAY_MS*1000);
		}
	}
}


static void disableSupply(Resource_e res)
{
	uint32_t pin = 0;
	if (res & RESMAN_SUP_5V) {
		pin = SUPPLY_EN_5_PIN;
		HAL_GPIO_WritePin(SUPPLY_EN_5_PORT, pin, 1);
		HAL_GPIO_DeInit(SUPPLY_EN_5_PORT, pin);
	}
	if (res & (RESMAN_SUP_5V_P | RESMAN_SUP_3_3V)) {
		pin |= ((res & RESMAN_SUP_5V_P)? SUPPLY_5V_P_PIN : 0);
		pin |= ((res & RESMAN_SUP_3_3V)? SUPPLY_3_3V_PIN : 0);
		HAL_GPIO_WritePin(SUPPLY_P_PORT, pin, 1);
		HAL_GPIO_DeInit(SUPPLY_P_PORT, pin);
	}
}


static inline void disableAllPeriph(void)
{
	module->resman.allbits = 0;
	for (BaseDriver_t *b = module->list; b; b = b->list) {
		module->resman.allbits |= b->resman.allbits;
	}
	module->resman.allbits |= (RESMAN_SUP_5V | RESMAN_SUP_3_3V); // do not touch
	disableSupply( ~(module->resman.allbits) );
	if ( !module->resman.dma1 ) {
		__HAL_RCC_DMA1_CLK_DISABLE(); }
	if ( !module->resman.dma2 ) {
		__HAL_RCC_DMA2_CLK_DISABLE(); }
	if ( !module->resman.gpioa ) {
		__HAL_RCC_GPIOA_CLK_DISABLE(); }
	if ( !module->resman.gpiob ) {
		__HAL_RCC_GPIOB_CLK_DISABLE(); }
	if ( !module->resman.gpioc ) {
		__HAL_RCC_GPIOC_CLK_DISABLE(); }
	if ( !module->resman.gpiod ) {
		__HAL_RCC_GPIOD_CLK_DISABLE(); }
	if ( !module->resman.gpioe ) {
		__HAL_RCC_GPIOE_CLK_DISABLE(); }
}


void main_init(void)
{
	module = (Module_t*)calloc(1, sizeof(Module_t));

	runSysTimer();
	HAL_TIM_Base_Start_IT(&htim6);

	prg_enablesupply(RESMAN_SUP_5V | RESMAN_SUP_3_3V);
	//HAL_IWDG_Refresh(&hiwdg);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_SuspendTick();
	//HAL_IWDG_Refresh(&hiwdg);

	// todo
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // led on
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	prg_delay_locked_us(50000);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	prg_delay_locked_us(50000);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	prg_delay_locked_us(50000);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	prg_delay_locked_us(50000);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	prg_delay_locked_us(50000);

	BaseDriver_t *drv;
	BaseDriver_t *list;

	drv = dToB(&module->retain);
	list = module->list = drv;
	retain_init(dToV(drv), 0);
	drv->supBusy = &module->supBusy;
	basedriver_registerQue(drv, &module->toque, drvId_Retain);

	drv = dToB(&module->uart);
	list->list = drv; list = drv;
	uartdrv_init(dToV(drv), 0);
	drv->supBusy = &module->supBusy;
	basedriver_registerQue(drv, &module->toque, drvId_Uart);

	drv = dToB(&module->din);
	list->list = drv; list = drv;
	dindrv_init(dToV(drv), &module->retain.db.din.ts);
	drv->supBusy = &module->supBusy;
	basedriver_registerQue(drv, &module->toque, drvId_Din);
	dindrv_setParams(dToV(drv), &module->retain.db.din.prms);

	drv = dToB(&module->adc);
	list->list = drv; list = drv;
	adcdrv_init(dToV(drv), &module->retain.db.adc.ts);
	drv->supBusy = &module->supBusy;
	basedriver_registerQue(drv, &module->toque, drvId_Adc);
	adcdrv_setParams(dToV(drv), &module->retain.db.adc.prms);

	drv = dToB(&module->st1w);
	list->list = drv; list = drv;
	senst1wdrv_init(dToV(drv), &module->retain.db.st1w.ts);
	drv->supBusy = &module->supBusy;
	basedriver_registerQue(drv, &module->toque, drvId_St1w);
	senst1wdrv_setParams(dToV(drv), &module->retain.db.st1w.prms);

	drv = dToB(&module->sht);
	list->list = drv; list = drv;
	shtdrv_init(dToV(drv), &module->retain.db.sht.ts);
	drv->supBusy = &module->supBusy;
	basedriver_registerQue(drv, &module->toque, drvId_Sht);
	shtdrv_setParams(dToV(drv), &module->retain.db.sht.prms);

	drv = dToB(&module->wt);
	list->list = drv; list = drv;
	wtdrv_init(dToV(drv), &module->retain.db.wt.ts);
	drv->supBusy = &module->supBusy;
	basedriver_registerQue(drv, &module->toque, drvId_Wt);
	wtdrv_setParams(dToV(drv), &module->retain.db.wt.prms);

	HAL_RTCEx_SetSecond_IT(&hrtc);

	handle5VEvents(1);

	for (BaseDriver_t *b = module->list; b; b = b->list) {
		b->cycle(b, module);
	}
}


void main_loop(void)
{
	// check timeouts
	for (;;) {
		to_item_t *toitem = to_check(&module->toque);
		if (!toitem) break;
		to_delete(&module->toque, toitem);

		BaseDriver_t *b = toitem->data;
		b->cycle(b, module);

		//HAL_IWDG_Refresh(&hiwdg);
	}

	// waiting for short operations
	uint8_t busy;
	do {
		busy = 0;
		for (BaseDriver_t *b = module->list; b; b = b->list) {
			busy |= b->busy;
		}
		//HAL_IWDG_Refresh(&hiwdg);
	} while (busy);

	disableAllPeriph();

	// check for events
	for (BaseDriver_t *b = module->list; b; b = b->list) {
		if (b->eventIsReady) {
			switch ( b->getId(b) ) {
				case drvId_Uart: { handleCombus(); } break;
				case drvId_Din: {
					DinDrvData_t data;
					dindrv_getData(dToV(b), &data, 1);
					memcpy(&module->retain.db.din.ts, &data.ts, sizeof(DinDrvTimeStamp_t));
					retain_triggerUpdate(&module->retain);
				} break;
				case drvId_Adc: {
					AdcDrvData_t data;
					adcdrv_getData(dToV(b), &data, 1);
					memcpy(&module->retain.db.adc.ts, &data.ts, sizeof(AdcDrvTimeStamp_t));
					retain_triggerUpdate(&module->retain);
				} break;
				case drvId_St1w: {
					SensT1wDrvData_t data;
					senst1wdrv_getData(dToV(b), &data, 1);
					memcpy(&module->retain.db.st1w.ts, &data.ts, sizeof(SensT1wDrvTimeStamp_t));
					retain_triggerUpdate(&module->retain);
				} break;
				case drvId_Sht: {
					ShtDrvData_t data;
					shtdrv_getData(dToV(b), &data, 1);
					memcpy(&module->retain.db.sht.ts, &data.ts, sizeof(ShtDrvTimeStamp_t));
					retain_triggerUpdate(&module->retain);
				} break;
				case drvId_Wt: {
					WtDrvData_t data;
					wtdrv_getData(dToV(b), &data, 1);
					memcpy(&module->retain.db.wt.ts, &data.ts, sizeof(WtDrvTimeStamp_t));
					retain_triggerUpdate(&module->retain);
				} break;
				default: break;
			}
		}
	}

	//HAL_IWDG_Refresh(&hiwdg);

	// sleep
	uint32_t to = to_poll(&module->toque);
	if (to > 2000) {
		setSysTimer(to);
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		//HAL_IWDG_Refresh(&hiwdg);
		handleSysTimerOnWakeUp();
	}
}


/* system timer ******************************************************* */
/* ******************************************************************** */


static const uint32_t prg_sysClockTicksUs = prg_SystemCoreClock/1000000;
static const uint32_t prg_sysClockTicksMs = prg_SystemCoreClock/1000;
//static const uint32_t prg_sysClockTicksS = prg_SystemCoreClock;

enum { SysTimerRunning, SysTimerWaiting };
static volatile int sysTimerMode = SysTimerRunning;
static volatile uint64_t systemTimeInUs = 0;
static volatile uint64_t SYSTIM_CNT = 0;
#define SYSTIM 			TIM6
#define SYSTIM_PERIOD	10000

static uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc)
{
	uint16_t high1 = 0U, high2 = 0U, low = 0U;
	uint32_t timecounter = 0U;
	high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
	low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
	high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
	timecounter = (high1 != high2)?
		(((uint32_t) high2 << 16U) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT)) :
		(((uint32_t) high1 << 16U) | low);
	return timecounter;
}

static void RTC_WriteTimeCounter(RTC_HandleTypeDef *hrtc, uint32_t TimeCounter)
{
	while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
		;
	__HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
	WRITE_REG(hrtc->Instance->CNTH, (TimeCounter >> 16U));
	WRITE_REG(hrtc->Instance->CNTL, (TimeCounter & RTC_CNTL_RTC_CNT));
	__HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
	while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
		;
}

static inline void setSystemTimeInUs(uint32_t val)
{
	systemTimeInUs += val;

//	// todo
//	static volatile uint64_t timtest2 = 0;
//	if (systemTimeInUs >= timtest2 + 995000) {
//		timtest2 = systemTimeInUs;
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
//	}
}

void TIM1_UP_IRQHandler(void)
{
	if (ACCEL_TIMER->SR & TIM_IT_UPDATE) {
		ACCEL_TIMER->SR = (uint32_t)~(TIM_IT_UPDATE);
		adcdrv_cycle(&module->adc, ACCEL_TIMER);
	}
}

void TIM6_IRQHandler(void)
{
	if (SYSTIM->SR & TIM_IT_UPDATE) {
		SYSTIM->SR &= (uint32_t)~(TIM_IT_UPDATE);
		if (sysTimerMode == SysTimerRunning) {
			setSystemTimeInUs(SYSTIM_PERIOD);
		} else {
			setSystemTimeInUs(SYSTIM_CNT*1000);
			runSysTimer(); // reset tim and restart
		}
	}
}

static void runSysTimer(void)
{
	SYSTIM->CR1 &= ~(TIM_CR1_CEN); // disable tim
	__disable_fault_irq();
	SYSTIM->DIER &= ~(TIM_IT_UPDATE); // disable int
	SYSTIM->CNT = 0; // reset current value
	SYSTIM->PSC = prg_sysClockTicksUs-1; // freq prescaler
	SYSTIM->ARR = SYSTIM_PERIOD; // period
	SYSTIM->EGR = 1; // reload
	SYSTIM->SR &= ~(TIM_IT_UPDATE);
	SYSTIM->DIER |= (TIM_IT_UPDATE); // enable int
	sysTimerMode = SysTimerRunning;
	SYSTIM->CR1 |= TIM_CR1_CEN; // enable tim
	__enable_fault_irq();
}

static void handleSysTimerOnWakeUp(void)
{
	SYSTIM->CR1 &= ~(TIM_CR1_CEN); // disable tim
	setSystemTimeInUs(SYSTIM->CNT*1000);
	runSysTimer();
}

static void setSysTimer(uint32_t timeout)
{
	SYSTIM->CR1 &= ~(TIM_CR1_CEN); // disable tim
	__disable_fault_irq();
	SYSTIM->DIER &= ~(TIM_IT_UPDATE); // disable int
	setSystemTimeInUs(SYSTIM->CNT);
	SYSTIM->CNT = 0; // reset current value
	SYSTIM->PSC = prg_sysClockTicksMs-1; // freq prescaler
	SYSTIM_CNT = timeout/1000;
	SYSTIM->ARR = SYSTIM_CNT; // period
	SYSTIM->EGR = 1; // reload
	SYSTIM->SR &= ~(TIM_IT_UPDATE);
	SYSTIM->DIER |= (TIM_IT_UPDATE); // enable int
	sysTimerMode = SysTimerWaiting;
	SYSTIM->CR1 |= (TIM_CR1_CEN); // enable tim
	__enable_fault_irq();
}

uint64_t prg_locked_getsystick(void)
{
	static uint32_t prevSysTick = 0;
	uint64_t ret = systemTimeInUs;
	uint32_t tick = SYSTIM->CNT;
	ret += (tick >= prevSysTick)?
		(tick - prevSysTick) : (SYSTIM_PERIOD - prevSysTick) + tick;
	prevSysTick = tick;
	return ret;
}

void prg_delay_locked_us(uint32_t d)
{
	// tested on 8 & 48 MHz
	d = (d*10*prg_sysClockTicksUs)/77;
	while (d--);
}

void prg_delay_us(uint32_t d)
{
	__disable_fault_irq();
	prg_delay_locked_us(d);
	__enable_fault_irq();
}

void prg_setrtc(uint32_t timeval20)
{
	__disable_irq();
	RTC_WriteTimeCounter(&hrtc, timeval20);
	__enable_irq();
}

uint32_t prg_gettimestamp(void)
{
	uint32_t ret;
	__disable_irq();
	ret = RTC_ReadTimeCounter(&hrtc);
	__enable_irq();
	return ret;
}


/* ******************************************************************** */
/* system timer ******************************************************* */


int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len);//, HAL_MAX_DELAY);
	return len;
}

