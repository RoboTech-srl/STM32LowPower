/**
******************************************************************************
* @file    STM32LowPower.cpp
* @author  WI6LABS
* @version V1.0.0
* @date    11-December-2017
* @brief   Provides a STM32 Low Power interface with Arduino
*
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

#include "STM32LowPower.h"

STM32LowPower LowPower;


STM32LowPower::STM32LowPower()
{
  _configured = false;
}

/**
  * @brief  Initializes the low power mode
  * @param  None
  * @retval None
  */
void STM32LowPower::begin(void)
{
  LowPower_init();
  _configured = true;
}

/**
  * @brief  Enable the idle low power mode (STM32 sleep). Exit this mode on interrupt.
  * @param  None
  * @retval None
  */
void STM32LowPower::idle(void)
{
  LowPower_sleep(PWR_MAINREGULATOR_ON);
}

/**
  * @brief  Enable the idle low power mode (STM32 sleep). Exit this mode on
  *         interrupt or in n milliseconds.
  * @param  millis: delay before leave the idle mode.
  * @retval None
  */
void STM32LowPower::idle(uint32_t millis)
{
  programRtcWakeUp(millis);
  LowPower_sleep(PWR_MAINREGULATOR_ON);
}

/**
  * @brief  Enable the sleep low power mode (STM32 sleep). Exit this mode on
  *         interrupt.
  * @param  none
  * @retval None
  */
void STM32LowPower::sleep(void)
{
  LowPower_sleep(PWR_LOWPOWERREGULATOR_ON);
}

/**
  * @brief  Enable the sleep low power mode (STM32 sleep). Exit this mode on
  *         interrupt or in n milliseconds.
  * @param  millis: delay before leave the idle mode.
  * @retval None
  */
void STM32LowPower::sleep(uint32_t millis)
{
  programRtcWakeUp(millis);
  LowPower_sleep(PWR_LOWPOWERREGULATOR_ON);
}

/**
  * @brief  Enable the deepsleep low power mode (STM32 stop). Exit this mode on
  *         interrupt.
  * @param  none
  * @retval None
  */
void STM32LowPower::deepSleep(void)
{
  LowPower_stop();
}

/**
  * @brief  Enable the deepsleep low power mode (STM32 stop). Exit this mode on
  *         interrupt or in n milliseconds.
  * @param  millis: delay before leave the idle mode.
  * @retval None
  */
void STM32LowPower::deepSleep(uint32_t millis)
{
  programRtcWakeUp(millis);
  LowPower_stop();
}

/**
  * @brief  Enable the shutdown low power mode (STM32 shutdown or standby mode).
  *         Exit this mode on interrupt.
  * @param  none
  * @retval None
  */
void STM32LowPower::shutdown(void)
{
  LowPower_shutdown();
}

/**
  * @brief  Enable the shutdown low power mode (STM32 shutdown or standby mode).
  *          Exit this mode on interrupt or in n milliseconds.
  * @param  millis: delay before leave the idle mode.
  * @retval None
  */
void STM32LowPower::shutdown(uint32_t millis)
{
  programRtcWakeUp(millis);
  LowPower_shutdown();
}

/**
  * @brief  Enable or disable GPIO pin in interrupt mode. If the pin is a wakeup pin, it is
  *         configured as wakeup source (only supported modes).
  * @param  pin:  pin number
  * @param  callback: pointer to callback function (optional).
  * @param  mode: pin interrupt mode (HIGH, LOW, RISING, FALLING or CHANGE, NOT_AN_INTERRUPT to disable).
  * @retval None
  */
void STM32LowPower::enableWakeupPin(uint32_t pin, voidFuncPtrVoid callback, uint32_t mode)
{
  // all GPIO for idle (smt32 sleep) and sleep (stm32 stop)
  attachInterrupt(pin, callback, mode);

  // If Gpio is a Wake up pin activate it for deepSleep (standby stm32) and shutdown
  LowPower_EnableWakeUpPin(pin, mode);
}

/**
  * @brief  Enable or disable a serial interface as a wakeup source.
  * @param  serial: pointer to a HardwareSerial (NULL to disable)
  * @param  callback: pointer to callback function called when leave the low power
  *                   mode (optional).
  * @retval None
  */
void STM32LowPower::enableWakeupSerial(HardwareSerial *serial, voidFuncPtrVoid callback)
{
  LowPower_EnableWakeUpUart(serial ? &(serial->_serial) : NULL, callback);
}

/**
  * @brief  Enable or disable a callback function for RTC wakeup.
  * @param  callback: pointer to callback function called when leave the low power
  *                   mode (NULL to disable).
  * @retval None
  */
void STM32LowPower::enableWakeupRtcFunction(voidFuncPtr callback)
{
  STM32RTC& rtcLowPower = STM32RTC::getInstance();

  if (callback)
    rtcLowPower.attachInterrupt(callback);
  else
    rtcLowPower.detachInterrupt();
}

/**
  * @brief  Configure the RTC alarm
  * @param  millis: time of the alarm in milliseconds. At least 1000ms on some platforms.
  * @retval None
  */
void STM32LowPower::programRtcWakeUp(uint32_t millis)
{
  STM32RTC& rtcLowPower = STM32RTC::getInstance();

  uint32_t epoc, ss;
  int32_t sec, ms;

  if(millis > 0) {
    if (!rtcLowPower.isConfigured()){
      //Enable RTC
      rtcLowPower.begin(STM32RTC::RTC_HOUR_24);
    }

    // convert millisecond to second
    ms = millis % 1000;
    sec = millis / 1000;

    epoc = rtcLowPower.getEpoch();
    ss = rtcLowPower.getSubSeconds();
    // handle carry
    if (ss + ms >= 1000) {
      ms -= 1000;
      sec += 1;
    }
    rtcLowPower.setAlarmSubSeconds( ss + ms );
    rtcLowPower.setAlarmEpoch( epoc + sec );
  }

}
