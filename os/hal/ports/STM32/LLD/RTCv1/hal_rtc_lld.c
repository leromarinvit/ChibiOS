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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    RTCv1/hal_rtc_lld.c
 * @brief   STM32 RTC subsystem low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

#include "hal.h"

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if STM32_ST_USE_RTC
#define RTC_PRESCALER       (STM32_RTCCLK / OSAL_ST_FREQUENCY - 1)
#else
#define RTC_PRESCALER       (STM32_RTCCLK - 1)
#endif

#define RTC_PRESCALER_HIGH  ((RTC_PRESCALER >> 16) & 0x000F)
#define RTC_PRESCALER_LOW   ((RTC_PRESCALER)       & 0xFFFF)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief RTC driver identifier.
 */
RTCDriver RTCD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if STM32_ST_USE_RTC
static uint32_t offset_seconds;
static int32_t  offset_div;

static RTCAlarm alarm_time;
static uint32_t alarm_ticks;
static bool     alarm_armed;
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Wait for synchronization of RTC registers with APB1 bus.
 * @details This function must be invoked before trying to read RTC registers
 *          in the backup domain: DIV, CNT, ALR. CR registers can always
 *          be read.
 *
 * @notapi
 */
static void rtc_apb1_sync(void) {

  while ((RTC->CRL & RTC_CRL_RSF) == 0)
    ;
}

/**
 * @brief   Wait for for previous write operation complete.
 * @details This function must be invoked before writing to any RTC registers
 *
 * @notapi
 */
static void rtc_wait_write_completed(void) {

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
static void rtc_acquire_access(void) {

  rtc_wait_write_completed();
  RTC->CRL |= RTC_CRL_CNF;
}

/**
 * @brief   Releases write access to RTC registers.
 *
 * @notapi
 */
static void rtc_release_access(void) {

  RTC->CRL &= ~RTC_CRL_CNF;
}

/**
 * @brief   Converts time from timespec to seconds counter.
 *
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 * @return              the TR register encoding.
 *
 * @notapi
 */
static time_t rtc_encode(const RTCDateTime *timespec) {
  struct tm tim;

  rtcConvertDateTimeToStructTm(timespec, &tim, NULL);
  return mktime(&tim);
}

/**
 * @brief   Converts time from seconds/milliseconds to timespec.
 *
 * @param[in] tv_sec      seconds value
 * @param[in] tv_msec     milliseconds value
 * @param[out] timespec   pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
static void rtc_decode(uint32_t tv_sec,
                       uint32_t tv_msec,
                       RTCDateTime *timespec) {
  struct tm tim;
  struct tm *t;
  const time_t time = (const time_t)tv_sec;   /* Could be 64 bits.*/

  /* If the conversion is successful the function returns a pointer
     to the object the result was written into.*/
#if defined(__GNUC__) || defined(__CC_ARM)
  t = localtime_r(&time, &tim);
  osalDbgAssert(t != NULL, "conversion failed");
#else
  t = localtime(&time);
  memcpy(&tim, t, sizeof(struct tm));
#endif

  rtcConvertStructTmToDateTime(&tim, tv_msec, timespec);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   RTC interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_RTC1_HANDLER) {
  uint16_t flags;

  OSAL_IRQ_PROLOGUE();

  /* Code hits this wait only when AHB1 bus was previously powered off by any
     reason (standby, reset, etc). In other cases there is no waiting.*/
  rtc_apb1_sync();

  /* Mask of all enabled and pending sources.*/
  flags = RTCD1.rtc->CRH & RTCD1.rtc->CRL;
  RTCD1.rtc->CRL &= ~(RTC_CRL_SECF | RTC_CRL_ALRF | RTC_CRL_OWF);

#if STM32_ST_USE_RTC
  uint32_t next_rollover = offset_seconds + 0x100000000LL / OSAL_ST_FREQUENCY;
  static bool overflow_fired;

  /* Epoch overflow within this RTC period? */
  if (next_rollover < offset_seconds) {
    if (RTCD1.callback != NULL && !overflow_fired) {
      uint32_t sec;
      rtcSTM32GetSecMsec(&RTCD1, &sec, NULL);
      if (sec < offset_seconds)
        RTCD1.callback(&RTCD1, RTC_EVENT_OVERFLOW);
    }
    overflow_fired = true;
  } else {
    overflow_fired = false;
  }

  /* Systick alarm and possibly RTC alarm.*/
  if (flags & RTC_CRL_ALRF) {
    if (alarm_armed && st_lld_get_counter() >= alarm_ticks) {
      alarm_armed = false;
      alarm_ticks = 0;
      if (RTCD1.callback != NULL)
        RTCD1.callback(&RTCD1, RTC_EVENT_ALARM);
    }

    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }

  /* Systick overflow - adjust RTC offset.*/
  if (flags & RTC_CRL_OWF) {
    osalSysLockFromISR();
    offset_seconds += 0x100000000LL / OSAL_ST_FREQUENCY;
    offset_div -= (0x100000000LL % OSAL_ST_FREQUENCY) * (RTC_PRESCALER + 1);
    if (offset_div <= -OSAL_ST_FREQUENCY * (RTC_PRESCALER + 1)) {
      offset_seconds++;
      offset_div += OSAL_ST_FREQUENCY * (RTC_PRESCALER + 1);
    }
    osalSysUnlockFromISR();
  }
#else
  if (flags & RTC_CRL_SECF)
    RTCD1.callback(&RTCD1, RTC_EVENT_SECOND);

  if (flags & RTC_CRL_ALRF)
    RTCD1.callback(&RTCD1, RTC_EVENT_ALARM);

  if (flags & RTC_CRL_OWF)
    RTCD1.callback(&RTCD1, RTC_EVENT_OVERFLOW);
#endif

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Load value of RTCCLK to prescaler registers.
 * @note    The pre-scaler must not be set on every reset as RTC clock
 *          counts are lost when it is set.
 * @note    This function designed to be called from
 *          hal_lld_backup_domain_init(). Because there is only place
 *          where possible to detect BKP domain reset event reliably.
 *
 * @notapi
 */
void rtc_lld_set_prescaler(void) {
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  rtc_acquire_access();
  RTC->PRLH = RTC_PRESCALER_HIGH;
  RTC->PRLL = RTC_PRESCALER_LOW;
  rtc_release_access();

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
}

/**
 * @brief   Initialize RTC.
 *
 * @notapi
 */
void rtc_lld_init(void) {

#if STM32_ST_USE_RTC
  /* We want to avoid locking in ST inline functions. */
  osalDbgAssert(atomic_is_lock_free(&RTC->CRL), "need lock-free atomic access to RTC_CRL");
#endif

  /* RTC object initialization.*/
  rtcObjectInit(&RTCD1);

  /* RTC pointer initialization.*/
  RTCD1.rtc = RTC;

  /* RSF bit must be cleared by software after an APB1 reset or an APB1 clock
     stop. Otherwise its value will not be actual. */
  RTCD1.rtc->CRL &= ~RTC_CRL_RSF;

  /* Required because access to PRL.*/
  rtc_apb1_sync();

  rtc_wait_write_completed();
#if STM32_ST_USE_RTC
  /* Only overflow interrupt initially enabled.*/
  RTCD1.rtc->CRH = RTC_CRH_OWIE;
#else
  /* All interrupts initially disabled.*/
  RTCD1.rtc->CRH = 0;
#endif

  /* Callback initially disabled.*/
  RTCD1.callback = NULL;

  /* IRQ vector permanently assigned to this driver.*/
  nvicEnableVector(STM32_RTC1_NUMBER, STM32_RTC_IRQ_PRIORITY);
}

/**
 * @brief   Set current time.
 * @note    Fractional part will be silently ignored. There is no possibility
 *          to change it on STM32F1xx platform.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
void rtc_lld_set_time(RTCDriver *rtcp, const RTCDateTime *timespec) {
  time_t tv_sec = rtc_encode(timespec);

  rtcSTM32SetSec(rtcp, tv_sec);
}

/**
 * @brief   Get current time.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
void rtc_lld_get_time(RTCDriver *rtcp, RTCDateTime *timespec) {
  uint32_t tv_sec, tv_msec;

  rtcSTM32GetSecMsec(rtcp, &tv_sec, &tv_msec);
  rtc_decode(tv_sec, tv_msec, timespec);
}

/**
 * @brief   Set alarm time.
 *
 * @note    Default value after BKP domain reset is 0xFFFFFFFF
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[in] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_set_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm_number,
                       const RTCAlarm *alarmspec) {
  syssts_t sts;
  (void)alarm_number;

#if STM32_ST_USE_RTC
  (void)rtcp;

  uint32_t next_rollover;
  uint32_t cnt, alarm;

  /* Disable any previously armed alarm.*/
  alarm_ticks = 0;
  alarm_armed = false;

  if (alarmspec == NULL) {
    alarm_time = (RTCAlarm){};
    return;
  }
  alarm_time = *alarmspec;

  if (alarm_time.tv_sec == 0)
    return;

  sts = osalSysGetStatusAndLockX();

  next_rollover = offset_seconds + 0x100000000LL / OSAL_ST_FREQUENCY;

  /* If the alarm is after the next RTC rollover, we can't set an alarm yet.*/
  if (alarm_time.tv_sec > next_rollover)
    goto out;

  cnt = st_lld_get_counter();
  alarm = cnt + (alarm_time.tv_sec - offset_seconds) * OSAL_ST_FREQUENCY;
  if (offset_div > 0)
    alarm++;

  /* If the alarm is between the next rollover and the first second boundary
     after it (so the addition overflowed), we can't set an alarm either.*/
  if (alarm < cnt)
    goto out;

  /* Now we know that the alarm must fire within this RTC period.*/
  alarm_ticks = alarm;
  alarm_armed = true;

  /* If an earlier alarm is already set, don't change it.*/
  if (st_lld_is_alarm_active() && st_lld_get_alarm() <= alarm)
    goto out;

  st_lld_start_alarm(alarm);

out:
  osalSysRestoreStatusX(sts);
#else

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  rtc_acquire_access();
  if (alarmspec != NULL) {
    rtcp->rtc->ALRH = (uint16_t)(alarmspec->tv_sec >> 16);
    rtcp->rtc->ALRL = (uint16_t)(alarmspec->tv_sec & 0xFFFF);
  }
  else {
    rtcp->rtc->ALRH = 0;
    rtcp->rtc->ALRL = 0;
  }
  rtc_release_access();

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
#endif
}

/**
 * @brief   Get current alarm.
 * @note    If an alarm has not been set then the returned alarm specification
 *          is not meaningful.
 * @note    The function can be called from any context.
 * @note    Default value after BKP domain reset is 0xFFFFFFFF.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_get_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm_number,
                       RTCAlarm *alarmspec) {
  (void)alarm_number;
#if STM32_ST_USE_RTC
  (void)rtcp;
  if (alarmspec)
    *alarmspec = alarm_time;
#else
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  /* Required because access to ALR.*/
  rtc_apb1_sync();

  alarmspec->tv_sec = ((rtcp->rtc->ALRH << 16) + rtcp->rtc->ALRL);

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
#endif
}

/**
 * @brief   Enables or disables RTC callbacks.
 * @details This function enables or disables callbacks, use a @p NULL pointer
 *          in order to disable a callback.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] callback  callback function pointer or @p NULL
 *
 * @notapi
 */
void rtc_lld_set_callback(RTCDriver *rtcp, rtccb_t callback) {
#if STM32_ST_USE_RTC
  rtcp->callback = callback;
#else
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  if (callback != NULL) {

    /* IRQ sources enabled only after setting up the callback.*/
    rtcp->callback = callback;

    rtc_wait_write_completed();
    rtcp->rtc->CRL &= ~(RTC_CRL_OWF | RTC_CRL_ALRF | RTC_CRL_SECF);
    rtcp->rtc->CRH = RTC_CRH_OWIE | RTC_CRH_ALRIE | RTC_CRH_SECIE;
  }
  else {
    rtc_wait_write_completed();
    rtcp->rtc->CRH = 0;

    /* Callback set to NULL only after disabling the IRQ sources.*/
    rtcp->callback = NULL;
  }

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
#endif
}

/**
 * @brief   Get seconds and (optionally) milliseconds from RTC.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[out] tv_sec   pointer to seconds value
 * @param[out] tv_msec  pointer to milliseconds value, set it
 *                      to @p NULL if not needed
 *
 * @api
 */
void rtcSTM32GetSecMsec(RTCDriver *rtcp, uint32_t *tv_sec, uint32_t *tv_msec) {
#if STM32_ST_USE_RTC
  uint32_t sec, o_div;
  int_fast16_t msec;
  uint32_t cnt, div;
#else
  uint32_t time_frac;
#endif
  syssts_t sts;

  osalDbgCheck((NULL != tv_sec) && (NULL != rtcp));

#if STM32_ST_USE_RTC
  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  sec = offset_seconds;
  o_div = offset_div;

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);

  do {
    cnt = st_lld_get_counter();
    div = st_rtc_read_2x16(rtcp->rtc->DIVH, rtcp->rtc->DIVL);
  } while (cnt != st_lld_get_counter() || div != st_rtc_read_2x16(rtcp->rtc->DIVH, rtcp->rtc->DIVL));

  sec += cnt / OSAL_ST_FREQUENCY;
  msec = (cnt % OSAL_ST_FREQUENCY) * 1000 / OSAL_ST_FREQUENCY;
  msec += (div - o_div) * 1000 / STM32_RTCCLK;
  if (msec >= 1000) {
    sec++;
    msec -= 1000;
  } else if (msec < 0) {
    sec--;
    msec += 1000;
  }

  *tv_sec = sec;
  if (NULL != tv_msec)
    *tv_msec = msec;
#else
  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  /* Required because access to CNT and DIV.*/
  rtc_apb1_sync();

  /* wait for previous write accesses to complete.*/
  rtc_wait_write_completed();

  /* Loops until two consecutive read returning the same value.*/
  do {
    *tv_sec = ((uint32_t)(rtcp->rtc->CNTH) << 16) + rtcp->rtc->CNTL;
    time_frac = (((uint32_t)rtcp->rtc->DIVH) << 16) + (uint32_t)rtcp->rtc->DIVL;
  } while ((*tv_sec) != (((uint32_t)(rtcp->rtc->CNTH) << 16) + rtcp->rtc->CNTL));

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);

  if (NULL != tv_msec)
    *tv_msec = (((uint32_t)STM32_RTCCLK - 1 - time_frac) * 1000) / STM32_RTCCLK;
#endif
}

/**
 * @brief   Set seconds in RTC.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] tv_sec    seconds value
 *
 * @api
 */
void rtcSTM32SetSec(RTCDriver *rtcp, uint32_t tv_sec) {
  syssts_t sts;

  osalDbgCheck(NULL != rtcp);

#if STM32_ST_USE_RTC
  uint32_t sec, div;

  do {
    sec = st_lld_get_counter();
    div = st_rtc_read_2x16(rtcp->rtc->DIVH, rtcp->rtc->DIVL);
  } while (sec != st_lld_get_counter() || div != st_rtc_read_2x16(rtcp->rtc->DIVH, rtcp->rtc->DIVL));

  sec = tv_sec - sec / OSAL_ST_FREQUENCY;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  offset_seconds = sec;
  offset_div = div;
#else
  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  rtc_acquire_access();
  rtcp->rtc->CNTH = (uint16_t)(tv_sec >> 16);
  rtcp->rtc->CNTL = (uint16_t)(tv_sec & 0xFFFF);
  rtc_release_access();
#endif

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
}

#endif /* HAL_USE_RTC */

/** @} */
