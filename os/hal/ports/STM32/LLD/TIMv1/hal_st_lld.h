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
 * @file    TIMv1/hal_st_lld.h
 * @brief   ST Driver subsystem low level driver header.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup ST
 * @{
 */

#ifndef HAL_ST_LLD_H
#define HAL_ST_LLD_H

#include "mcuconf.h"
#include "stm32_registry.h"
#include "stm32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Feature currently disabled.*/
#define STM32_ST_ENFORCE_ALARMS 1

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(STM32_ST_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ST_IRQ_PRIORITY               8
#endif

#if !defined(STM32_ST_USE_RTC) || defined(__DOXYGEN__)
#define STM32_ST_USE_RTC                    FALSE
#endif

#if STM32_ST_USE_RTC
#include <stdatomic.h>
#endif

/**
 * @brief   TIMx unit (by number) to be used for free running operations.
 * @note    You must select a 32 bits timer if a 32 bits @p systick_t type
 *          is required.
 * @note    Timers 2, 3, 4, 5, 21 and 22 are supported.
 */
#if !defined(STM32_ST_USE_TIMER) && !STM32_ST_USE_RTC || defined(__DOXYGEN__)
#define STM32_ST_USE_TIMER                  2
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_HAS_TIM2)
#define STM32_HAS_TIM2                      FALSE
#endif

#if !defined(STM32_HAS_TIM3)
#define STM32_HAS_TIM3                      FALSE
#endif

#if !defined(STM32_HAS_TIM4)
#define STM32_HAS_TIM4                      FALSE
#endif

#if !defined(STM32_HAS_TIM5)
#define STM32_HAS_TIM5                      FALSE
#endif

#if !defined(STM32_HAS_TIM21)
#define STM32_HAS_TIM21                     FALSE
#endif

#if !defined(STM32_HAS_TIM22)
#define STM32_HAS_TIM22                     FALSE
#endif

#if defined(STM32_ST_USE_TIMER) && STM32_ST_USE_RTC
#error "cannot set STM32_ST_USE_TIMER and STM32_ST_USE_RTC at the same time"
#endif

#if STM32_ST_USE_RTC
#define ST_LLD_NUM_ALARMS                   1

#elif STM32_ST_USE_TIMER == 2
#define STM32_ST_TIM                        STM32_TIM2
#define ST_LLD_NUM_ALARMS                   STM32_TIM2_CHANNELS

#elif STM32_ST_USE_TIMER == 3
#define STM32_ST_TIM                        STM32_TIM3
#define ST_LLD_NUM_ALARMS                   STM32_TIM3_CHANNELS

#elif STM32_ST_USE_TIMER == 4
#define STM32_ST_TIM                        STM32_TIM4
#define ST_LLD_NUM_ALARMS                   STM32_TIM4_CHANNELS

#elif STM32_ST_USE_TIMER == 5
#define STM32_ST_TIM                        STM32_TIM5
#define ST_LLD_NUM_ALARMS                   STM32_TIM5_CHANNELS

#elif STM32_ST_USE_TIMER == 21
#define STM32_ST_TIM                        STM32_TIM21
#define ST_LLD_NUM_ALARMS                   STM32_TIM21_CHANNELS

#elif STM32_ST_USE_TIMER == 22
#define STM32_ST_TIM                        STM32_TIM22
#define ST_LLD_NUM_ALARMS                   STM32_TIM22_CHANNELS

#else
#error "STM32_ST_USE_TIMER specifies an unsupported timer"
#endif

#if defined(STM32_ST_ENFORCE_ALARMS)

#if (STM32_ST_ENFORCE_ALARMS < 1) || (STM32_ST_ENFORCE_ALARMS > ST_LLD_NUM_ALARMS)
#error "invalid STM32_ST_ENFORCE_ALARMS value"
#endif

#undef ST_LLD_NUM_ALARMS
#define ST_LLD_NUM_ALARMS                   STM32_ST_ENFORCE_ALARMS
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define st_rtc_read_2x16(high, low) ({ \
  uint32_t _val; \
  do { \
    _val = (high) << 16 | (low); \
  } while (_val != ((high) << 16 | (low))); \
  _val; \
})

#define st_rtc_write_2x16(high, low, value) ({ \
  do { \
    (high) = (value) >> 16; \
    (low) = (value) & 0xFFFF; \
  } while ((high) != ((value) >> 16) || (low) != ((value) & 0xFFFF)); \
})

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void st_lld_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

#if STM32_ST_USE_RTC
/**
 * @brief   Wait for synchronization of RTC registers with APB1 bus.
 * @details This function must be invoked before trying to read RTC registers
 *          in the backup domain: DIV, CNT, ALR. CR registers can always
 *          be read.
 *
 * @notapi
 */
static inline void st_rtc_apb1_sync(void) {

  while ((RTC->CRL & RTC_CRL_RSF) == 0)
    ;
}

/**
 * @brief   Wait for for previous write operation complete.
 * @details This function must be invoked before writing to any RTC registers
 *
 * @notapi
 */
static inline void st_rtc_wait_write_completed(void) {

  while ((RTC->CRL & RTC_CRL_RTOFF) == 0)
    ;
}

/**
 * @brief   Acquires write access to RTC registers.
 * @details Before writing to the backup domain RTC registers the previous
 *          write operation must be completed. Use this function before
 *          writing to PRL, CNT, ALR registers.
 *
 * @notapi
 */
static inline bool st_rtc_try_lock(void) {

  const uint32_t idle = RTC_CRL_RSF | RTC_CRL_RTOFF;
  const uint32_t config = RTC_CRL_RSF | RTC_CRL_CNF;
  uint32_t expected = idle;
  st_rtc_apb1_sync();
  st_rtc_wait_write_completed();
  if (atomic_compare_exchange_strong_explicit(&RTC->CRL, &expected, config, memory_order_relaxed, memory_order_relaxed))
    return true;
  for (;;) {
    if (expected & RTC_CRL_CNF)
      return false;
    expected |= idle;
    if (atomic_compare_exchange_strong_explicit(&RTC->CRL, &expected, expected | config, memory_order_relaxed, memory_order_relaxed))
      return true;
  }
}

/**
 * @brief   Releases write access to RTC registers.
 *
 * @notapi
 */
static inline void st_rtc_unlock(void) {

  RTC->CRL &= ~RTC_CRL_CNF;
}
#endif /* STM32_ST_USE_RTC */

/**
 * @brief   Returns the time counter value.
 *
 * @return              The counter value.
 *
 * @notapi
 */
static inline systime_t st_lld_get_counter(void) {

#if STM32_ST_USE_RTC
  st_rtc_apb1_sync();
  // st_rtc_wait_write_completed();
  return st_rtc_read_2x16(RTC->CNTH, RTC->CNTL);
#else
  return (systime_t)STM32_ST_TIM->CNT;
#endif
}

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] abstime   the time to be set for the first alarm
 *
 * @notapi
 */
static inline void st_lld_start_alarm(systime_t abstime) {

#if STM32_ST_USE_RTC
  bool locked = st_rtc_try_lock();
  st_rtc_write_2x16(RTC->ALRH, RTC->ALRL, abstime);
  RTC->CRH |= RTC_CRH_ALRIE;
  if (locked)
    st_rtc_unlock();
#else
  STM32_ST_TIM->CCR[0] = (uint32_t)abstime;
  STM32_ST_TIM->SR     = 0;
#if ST_LLD_NUM_ALARMS == 1
  STM32_ST_TIM->DIER   = STM32_TIM_DIER_CC1IE;
#else
  STM32_ST_TIM->DIER  |= STM32_TIM_DIER_CC1IE;
#endif
#endif
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {

#if STM32_ST_USE_RTC
  RTC->CRH &= ~RTC_CRH_ALRIE;
#else
#if ST_LLD_NUM_ALARMS == 1
  STM32_ST_TIM->DIER = 0U;
#else
 STM32_ST_TIM->DIER &= ~STM32_TIM_DIER_CC1IE;
#endif
#endif
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t abstime) {

#if STM32_ST_USE_RTC
  bool locked = st_rtc_try_lock();
  st_rtc_write_2x16(RTC->ALRH, RTC->ALRL, abstime);
  if (locked)
    st_rtc_unlock();
#else
  STM32_ST_TIM->CCR[0] = (uint32_t)abstime;
#endif
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {

#if STM32_ST_USE_RTC
  st_rtc_apb1_sync();
  return st_rtc_read_2x16(RTC->ALRH, RTC->ALRL);
#else
  return (systime_t)STM32_ST_TIM->CCR[0];
#endif
}

/**
 * @brief   Determines if the alarm is active.
 *
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active(void) {

#if STM32_ST_USE_RTC
  return (bool)((RTC->CRH & RTC_CRH_ALRIE) != 0);
#else
  return (bool)((STM32_ST_TIM->DIER & STM32_TIM_DIER_CC1IE) != 0);
#endif
}

#if (ST_LLD_NUM_ALARMS > 1) || defined(__DOXYGEN__)
/**
 * @brief   Starts an alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] abstime   the time to be set for the first alarm
 * @param[in] alarm     alarm channel number
 *
 * @notapi
 */
static inline void st_lld_start_alarm_n(unsigned alarm, systime_t abstime) {


  STM32_ST_TIM->CCR[alarm] = (uint32_t)abstime;
  STM32_ST_TIM->SR         = 0;
  STM32_ST_TIM->DIER      |= (STM32_TIM_DIER_CC1IE << alarm);
}

/**
 * @brief   Stops an alarm interrupt.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 *
 * @notapi
 */
static inline void st_lld_stop_alarm_n(unsigned alarm) {

  STM32_ST_TIM->DIER &= ~(STM32_TIM_DIER_CC1IE << alarm);
}

/**
 * @brief   Sets an alarm time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm_n(unsigned alarm, systime_t abstime) {

  STM32_ST_TIM->CCR[alarm] = (uint32_t)abstime;
}

/**
 * @brief   Returns an alarm current time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm_n(unsigned alarm) {

  return (systime_t)STM32_ST_TIM->CCR[alarm];
}

/**
 * @brief   Determines if an alarm is active.
 *
 * @param[in] alarm     alarm channel number
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active_n(unsigned alarm) {

  return (bool)((STM32_ST_TIM->DIER & (STM32_TIM_DIER_CC1IE << alarm)) != 0);
}
#endif /* ST_LLD_NUM_ALARMS > 1 */

#endif /* HAL_ST_LLD_H */

/** @} */
