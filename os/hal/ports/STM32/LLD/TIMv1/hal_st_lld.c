/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    TIMv1/hal_st_lld.c
 * @brief   ST Driver subsystem low level driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

#if (OSAL_ST_RESOLUTION == 32)
#define ST_ARR_INIT                         0xFFFFFFFFU
#else
#define ST_ARR_INIT                         0x0000FFFFU
#endif

#if STM32_ST_USE_RTC
#if OSAL_ST_RESOLUTION != 32
#error "ST resolution must be 32 bit when using RTC"
#endif

#define ST_CLOCK_SRC                        STM32_RTCCLK
#define ST_HANDLER                          STM32_RTC1_HANDLER
#define ST_NUMBER                           STM32_RTC1_NUMBER
#define ST_ENABLE_CLOCK()
#define ST_ENABLE_STOP()

#elif STM32_ST_USE_TIMER == 2
#if !STM32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif
#if defined(STM32_TIM2_IS_USED)
#error "ST requires TIM2 but the timer is already used"
#endif
#if (OSAL_ST_RESOLUTION == 32) && !STM32_TIM2_IS_32BITS
#error "TIM2 is not a 32bits timer"
#endif

#define ST_HANDLER                          STM32_TIM2_HANDLER
#define ST_NUMBER                           STM32_TIM2_NUMBER
#define ST_CLOCK_SRC                        STM32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM2(true)
#if defined(STM32F1XX)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP
#elif defined(STM32L4XX) || defined(STM32L4XXP) || defined(STM32G4XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM2_STOP
#elif defined(STM32G0XX)
#define ST_ENABLE_STOP()                    DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM2_STOP
#elif defined(STM32H7XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM2
#else
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP
#endif

#elif STM32_ST_USE_TIMER == 3
#if !STM32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif
#if defined(STM32_TIM3_IS_USED)
#error "ST requires TIM3 but the timer is already used"
#endif
#if (OSAL_ST_RESOLUTION == 32) && !STM32_TIM3_IS_32BITS
#error "TIM3 is not a 32bits timer"
#endif

#define ST_HANDLER                          STM32_TIM3_HANDLER
#define ST_NUMBER                           STM32_TIM3_NUMBER
#define ST_CLOCK_SRC                        STM32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM3(true)
#if defined(STM32F1XX)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP
#elif defined(STM32L4XX) || defined(STM32L4XXP) || defined(STM32G4XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM3_STOP
#elif defined(STM32G0XX)
#define ST_ENABLE_STOP()                    DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM3_STOP
#elif defined(STM32H7XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM3
#else
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP
#endif

#elif STM32_ST_USE_TIMER == 4
#if !STM32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif
#if defined(STM32_TIM4_IS_USED)
#error "ST requires TIM4 but the timer is already used"
#endif
#if (OSAL_ST_RESOLUTION == 32) && !STM32_TIM4_IS_32BITS
#error "TIM4 is not a 32bits timer"
#endif

#define ST_HANDLER                          STM32_TIM4_HANDLER
#define ST_NUMBER                           STM32_TIM4_NUMBER
#define ST_CLOCK_SRC                        STM32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM4(true)
#if defined(STM32F1XX)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM4_STOP
#elif defined(STM32L4XX) || defined(STM32L4XXP) || defined(STM32G4XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM4_STOP
#elif defined(STM32H7XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM4
#else
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP
#endif

#elif STM32_ST_USE_TIMER == 5
#if !STM32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif
#if defined(STM32_TIM5_IS_USED)
#error "ST requires TIM5 but the timer is already used"
#endif
#if (OSAL_ST_RESOLUTION == 32) && !STM32_TIM5_IS_32BITS
#error "TIM5 is not a 32bits timer"
#endif

#define ST_HANDLER                          STM32_TIM5_HANDLER
#define ST_NUMBER                           STM32_TIM5_NUMBER
#define ST_CLOCK_SRC                        STM32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM5(true)
#if defined(STM32F1XX)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM5_STOP
#elif defined(STM32L4XX) || defined(STM32L4XXP) || defined(STM32G4XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM5_STOP
#elif defined(STM32H7XX)
#define ST_ENABLE_STOP()                    DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM5
#else
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP
#endif

#elif STM32_ST_USE_TIMER == 21
#if !STM32_HAS_TIM21
#error "TIM21 not present in the selected device"
#endif
#if defined(STM32_TIM21_IS_USED)
#error "ST requires TIM21 but the timer is already used"
#endif
#if (OSAL_ST_RESOLUTION == 32) && !STM32_TIM21_IS_32BITS
#error "TIM21 is not a 32bits timer"
#endif

#define ST_HANDLER                          STM32_TIM21_HANDLER
#define ST_NUMBER                           STM32_TIM21_NUMBER
#define ST_CLOCK_SRC                        STM32_TIMCLK2
#define ST_ENABLE_CLOCK()                   rccEnableTIM21(true)
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZ |= DBGMCU_APB2_FZ_DBG_TIM21_STOP

#elif STM32_ST_USE_TIMER == 22
#if !STM32_HAS_TIM22
#error "TIM22 not present in the selected device"
#endif
#if defined(STM32_TIM22_IS_USED)
#error "ST requires TIM22 but the timer is already used"
#endif
#if (OSAL_ST_RESOLUTION == 32) && !STM32_TIM22_IS_32BITS
#error "TIM21 is not a 32bits timer"
#endif

#define ST_HANDLER                          STM32_TIM22_HANDLER
#define ST_NUMBER                           STM32_TIM22_NUMBER
#define ST_CLOCK_SRC                        STM32_TIMCLK2
#define ST_ENABLE_CLOCK()                   rccEnableTIM22(true)
#define ST_ENABLE_STOP()                    DBGMCU->APB1FZ |= DBGMCU_APB2_FZ_DBG_TIM21_STOP

#else
#error "STM32_ST_USE_TIMER specifies an unsupported timer"
#endif

#if ST_CLOCK_SRC % OSAL_ST_FREQUENCY != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if STM32_ST_USE_RTC
#define ST_PRESCALER_MAX                    0xFFFFF
#else
#define ST_PRESCALER_MAX                    0xFFFF
#endif

#if (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1 > ST_PRESCALER_MAX
#error "the selected ST frequency is not obtainable because TIM timer prescaler limits"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

#if defined(STM32_CORE_CK)
#define SYSTICK_CK                          STM32_CORE_CK
#else
#define SYSTICK_CK                          STM32_HCLK
#endif

#if SYSTICK_CK % OSAL_ST_FREQUENCY != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1 > 0xFFFFFF
#error "the selected ST frequency is not obtainable because SysTick timer counter limits"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC) || defined(__DOXYGEN__)
/**
 * @brief   System Timer vector.
 * @details This interrupt is used for system tick in periodic mode.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SysTick_Handler) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  osalOsTimerHandlerI();
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)
/**
 * @brief   TIM2 interrupt handler.
 * @details This interrupt is used for system tick in free running mode.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ST_HANDLER) {
  uint32_t sr;
#if !STM32_ST_USE_RTC
  stm32_tim_t *timp = STM32_ST_TIM;
#endif

  OSAL_IRQ_PROLOGUE();

#if STM32_ST_USE_RTC
  sr  = RTC->CRL;
  RTC->CRL = RTC_CRL_RSF;
#else
  sr  = timp->SR;
  sr &= timp->DIER & STM32_TIM_DIER_IRQ_MASK;
  timp->SR = ~sr;
#endif

#if STM32_ST_USE_RTC
  if ((sr & RTC_CRL_ALRF) != 0U) {
#else
  if ((sr & TIM_SR_CC1IF) != 0U) {
#endif
    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }
#if ST_LLD_NUM_ALARMS > 1
  if ((sr & TIM_SR_CC2IF) != 0U) {
    if (st_callbacks[2] != NULL) {
      st_callbacks[0](1U);
    }
  }
#endif
#if ST_LLD_NUM_ALARMS > 2
  if ((sr & TIM_SR_CC3IF) != 0U) {
    if (st_callbacks[2] != NULL) {
      st_callbacks[1](2U);
    }
  }
#endif
#if ST_LLD_NUM_ALARMS > 3
  if ((sr & TIM_SR_CC4IF) != 0U) {
    if (st_callbacks[2] != NULL) {
      st_callbacks[2](3U);
    }
  }
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  /* Free running counter mode.*/

  /* Enabling timer clock.*/
  ST_ENABLE_CLOCK();

  /* Enabling the stop mode during debug for this timer.*/
  ST_ENABLE_STOP();

  /* Initializing the counter in free running mode.*/
#if STM32_ST_USE_RTC
  osalDbgAssert(__atomic_always_lock_free(sizeof RTC->CRL, &RTC->CRL), "need lock-free atoic compare-and-swap");
  RTC->CRL &= ~RTC_CRL_RSF;
  st_rtc_apb1_sync();
  st_rtc_wait_write_completed();
  RTC->CRH = 0;
  bool locked = st_rtc_try_lock();
  RTC->ALRH = 0;
  RTC->ALRL = 0;
  RTC->PRLH = ((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1) << 16;
  RTC->PRLL = ((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1) & 0xFFFF;
  if (locked)
    st_rtc_unlock();
#else
  STM32_ST_TIM->PSC    = (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1;
  STM32_ST_TIM->ARR    = ST_ARR_INIT;
  STM32_ST_TIM->CCMR1  = 0;
  STM32_ST_TIM->CCR[0] = 0;
#if ST_LLD_NUM_ALARMS > 1
  STM32_ST_TIM->CCR[1] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 2
  STM32_ST_TIM->CCR[2] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 3
  STM32_ST_TIM->CCR[3] = 0;
#endif
  STM32_ST_TIM->DIER   = 0;
  STM32_ST_TIM->CR2    = 0;
  STM32_ST_TIM->EGR    = TIM_EGR_UG;
  STM32_ST_TIM->CR1    = TIM_CR1_CEN;
#endif

  /* IRQ enabled.*/
  nvicEnableVector(ST_NUMBER, STM32_ST_IRQ_PRIORITY);
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
  /* Periodic systick mode, the Cortex-Mx internal systick timer is used
     in this mode.*/
  SysTick->LOAD = (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_ENABLE_Msk |
                  SysTick_CTRL_TICKINT_Msk;

  /* IRQ enabled.*/
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK, STM32_ST_IRQ_PRIORITY);
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
