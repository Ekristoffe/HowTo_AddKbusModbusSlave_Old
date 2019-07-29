//------------------------------------------------------------------------------
/// Copyright (c) WAGO Kontakttechnik GmbH & Co. KG
///
/// PROPRIETARY RIGHTS are involved in the subject matter of this material.
/// All manufacturing, reproduction, use and sales rights pertaining to this
/// subject matter are governed by the license agreement. The recipient of this
/// software implicitly accepts the terms of the license.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
///
///  \file     modbus_watchdog.c
///
///  \brief    modbus watchdog handling
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <modbus/modbus.h>
#include "modbus_watchdog.h"
#include "modbus_reply.h"
#include "modbus.h"
#include "utils.h"


#define MODBUSWATCHDOG_INTERVAL (100 * 1000) /**< @brief Modbus watchdog task interval. Default: 100ms*/
#define MODBUSWATCHDOG_REGISTER_START_ADDRESS 0x1000 /**< @brief Start address of modbus watchdog registers */

static pthread_t modbusWatchdog_thread;
static char modbusWatchdog_threadRunning = TRUE;
static volatile char modbusWatchdog_active = FALSE; /**< @brief Flag for active Watchdog*/
static modbus_mapping_t *mb_watchdog_mapping; /**< @brief Modbus register storrage*/

static uint16_t watchdogTimeout; /**< @brief Global variable for watchdog timeout*/

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

/**
 * @brief Reset minimal watchdog time to modbus register
 * @param[in] time Time to be set
 */
static void modbusWatchdog_resetMinimalTime(uint16_t time)
{
    mb_watchdog_mapping->tab_registers[4] = time;
}
//------------------------------------------------------------------------------------

/**
 * @brief Set minimal watchdog time to modbus register
 * @param[in] time Time to be set
 */
static void modbusWatchdog_setMinimalTime(uint16_t time)
{
    uint16_t reg_time = mb_watchdog_mapping->tab_registers[4];

    if (time < reg_time) // only set new value if smaller than old one
    {
        mb_watchdog_mapping->tab_registers[4] = time;
    }
}

//------------------------------------------------------------------------------------
/**
 * @brief Setting default values during start up.\n
 * For Register 0x1000 Watchdog timeout default is 100\n
 * For Register 0x1004 Watchdog minimal time is Watchdog timeout
 * @param[in] *conf Pointer to modbus storage
 */
static void modbusWatchdog_settingDefaults(modbus_mapping_t *conf)
{
    conf->tab_registers[0] = 0x0064; // 0x1000: 100*100ms = 10s
    conf->tab_registers[4] = 0x0064; // min Triggertime
}

//------------------------------------------------------------------------------------

static void modbusWatchdog_setTimeout(void)
{
    watchdogTimeout = mb_watchdog_mapping->tab_registers[0];
    modbusWatchdog_resetMinimalTime(watchdogTimeout);
    dprintf(VERBOSE_STD, "Watchdog Timeout: %ums\n", watchdogTimeout * 100);
    //modbusWatchdog_setMinimalTime(watchdogTimeout);
}

//------------------------------------------------------------------------------------
/**
 * @brief Get Watchdog time out value (0x1000)
 * @return time out value - (VALUE * 100ms) = Watchdog time out
 * @retval {N} If watchdog mapping is valid
 * @retval 0   If watchdog mapping is invalid
 */
static uint16_t modbusWatchdog_getTimeout(void)
{
    if (mb_watchdog_mapping != NULL)
    {
        return mb_watchdog_mapping->tab_registers[0];
    }
    else
    {
        return 0;
    }
}

static volatile uint16_t timeout; /**< @brief Global variable for timeout @todo Check who uses this!*/


//------------------------------------------------------------------------------------
/**
 * @brief  Modbus Watchdog thread task
 * @param[in] watchdogExpiredFkt Callback function that will be executed if watchdog expired.
 */
static void *modbusWatchdog_task(void (*watchdogExpiredFkt))
{
    void (*function)() = watchdogExpiredFkt;
    while (modbusWatchdog_threadRunning)
    {
        if (modbusWatchdog_active)
        {
            if (timeout > 0)
            {
                modbusWatchdog_setMinimalTime(--timeout);
                dprintf(VERBOSE_DEBUG, "MODBUS Watchdog active: %u\n", timeout);
            }
            else //expired
            {
                dprintf(VERBOSE_STD, "MODBUS Watchdog expired\n");
                //Execute callback function
                if (function != NULL)
                {
                    function();
                }
                //stop Watchdog
                modbusWatchdog_stop();
            }
        }
        usleep(MODBUSWATCHDOG_INTERVAL);
    }
    return NULL;
}

//------------------------------------------------------------------------------------
int modbusWatchdog_init(void (*watchdogExpiredFkt)())
{
    dprintf(VERBOSE_STD, "Watchdog Init\n");
    mb_watchdog_mapping = modbus_mapping_new(0, 0, 12, 0);
    if (mb_watchdog_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    modbusWatchdog_settingDefaults(mb_watchdog_mapping);
    watchdogTimeout = mb_watchdog_mapping->tab_registers[0];

    if (pthread_create(&modbusWatchdog_thread, NULL,
                       &modbusWatchdog_task, (void *) watchdogExpiredFkt) != 0)
    {
        return -2;
    }
    return 0;
}

//------------------------------------------------------------------------------------
void modbusWatchdog_deInit(void)
{
    modbusWatchdog_stop();
    modbusWatchdog_threadRunning = FALSE;
    pthread_join(modbusWatchdog_thread, NULL);
    if (mb_watchdog_mapping != NULL)
    {
        modbus_mapping_free(mb_watchdog_mapping);
    }
}
//------------------------------------------------------------------------------------
void modbusWatchdog_trigger(void)
{
    dprintf(VERBOSE_DEBUG, "Watchdog trigger\n");
    timeout = modbusWatchdog_getTimeout();
}

//------------------------------------------------------------------------------------
void modbusWatchdog_start(void)
{
    dprintf(VERBOSE_STD, "Watchdog start\n");
    modbusWatchdog_active = TRUE;
}

//------------------------------------------------------------------------------------
void modbusWatchdog_stop(void)
{
    dprintf(VERBOSE_STD, "Watchdog stop\n");
    modbusWatchdog_active = FALSE;
}

//------------------------------------------------------------------------------------
void modbusWatchdog_setStatus(void)
{
    mb_watchdog_mapping->tab_registers[6] = modbusWatchdog_active;
}

//------------------------------------------------------------------------------------
/**
 * If asked for Modbus-Watchdog registers, this will be handled here.
 * The replay also will be initiated here.
 *
 * We have to manipulate the given register address and substract the MODBUSWATCHDOG_REGISTER_START_ADDRESS from it.
 */
int modbusWatchdog_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len)
{
    int offset = modbus_get_header_length(ctx);
    int function = command[offset];
    uint16_t address = (command[offset + 1] << 8) + command[offset + 2];
    static unsigned char modbusWatchdog_stopFlag = FALSE;

    //manipulate address
    uint16_t fakeAddress = address - MODBUSWATCHDOG_REGISTER_START_ADDRESS;

    switch(function)
    {
     case _FC_READ_HOLDING_REGISTERS:
      modbusWatchdog_setStatus();
      modbus_reply_offset(ctx, command, command_len, mb_watchdog_mapping, MODBUSWATCHDOG_REGISTER_START_ADDRESS);
      break;
     case _FC_WRITE_SINGLE_REGISTER:
      if (fakeAddress == 0) //Watchdog timeout
      {
          if (modbusWatchdog_active == FALSE)
          {
              modbus_reply_offset(ctx, command, command_len, mb_watchdog_mapping, MODBUSWATCHDOG_REGISTER_START_ADDRESS);
              modbusWatchdog_setTimeout(); //set new timeout only when watchdog inactive.
          }
          else
          {
              modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
          }
      }
      else if (fakeAddress == 3) //Watchdog trigger
      {
          modbus_reply_offset(ctx, command, command_len, mb_watchdog_mapping, MODBUSWATCHDOG_REGISTER_START_ADDRESS);
          if (mb_watchdog_mapping->tab_registers[fakeAddress] > 0) // start Watchdog if > 0
          {
              if (modbusWatchdog_active == FALSE)
              {
                  timeout = modbusWatchdog_getTimeout();
                  modbusWatchdog_resetMinimalTime(timeout);
                  modbusWatchdog_start();
              }
              else
              {
                  modbusWatchdog_trigger();
              }
          }

          //reset value
          mb_watchdog_mapping->tab_registers[fakeAddress] = 0;

      }
      else if (fakeAddress == 4) // Min Triggertime
      {
          modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_FUNCTION );
      }
      else if (fakeAddress == 8) //Watchdog stoppen
      {
          modbus_reply_offset(ctx, command, command_len, mb_watchdog_mapping, MODBUSWATCHDOG_REGISTER_START_ADDRESS);

          if (mb_watchdog_mapping->tab_registers[fakeAddress] == 0x55AA)
          {
              modbusWatchdog_stopFlag = TRUE;
          }
          else if ((modbusWatchdog_stopFlag == TRUE) && 
              (mb_watchdog_mapping->tab_registers[fakeAddress] == 0xAA55))
          {
              dprintf(VERBOSE_STD, "Watchdog STOP\n");
              modbusWatchdog_stop();
              modbusWatchdog_stopFlag = FALSE;
          }
          else
          {
              modbusWatchdog_stopFlag = FALSE;
          }
      }
      else
      {
          modbus_reply_offset(ctx, command, command_len, mb_watchdog_mapping, MODBUSWATCHDOG_REGISTER_START_ADDRESS);
      }
      break;
     default:
      modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_FUNCTION );
    }

    return 0;
}
