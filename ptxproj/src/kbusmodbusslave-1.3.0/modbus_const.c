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
///  \file     modbus_const.c
///
///  \brief    Setting up the modbus constats register
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include "modbus.h"
#include "modbus_const.h"
#include "modbus_reply.h"
#include "utils.h"

#define MODBUSCONFIG_CONST_REGISTER_START_ADDRESS 0x2000 /**< @brief Start address of modbus const register */
#define MODBUSCONFIG_CONST_REGISTER_LEN 9 /**< @brief Register count for const registers */

static modbus_mapping_t *mb_config_const_mapping; /**< @brief Modbus register storage for const register */


static void modbusConfigConst_setValues(void)
{
    if (mb_config_const_mapping != NULL)
    {
        //Store pointer to make it more readable
        uint16_t *ptr = mb_config_const_mapping->tab_registers;

        ptr[0] = 0x0000;
        ptr[1] = 0xFFFF;
        ptr[2] = 0x1234;
        ptr[3] = 0xAAAA;
        ptr[4] = 0x5555;
        ptr[5] = 0x7FFF;
        ptr[6] = 0x8000;
        ptr[7] = 0x3FFF;
        ptr[8] = 0x4000;
    }
}

/**
 * @brief Initialize modbus const. Allocate memory for registers.
 */
int modbusConfigConst_init(void)
{
    dprintf(VERBOSE_STD, "Modbus const Init\n");
    mb_config_const_mapping = modbus_mapping_new(0, 0, MODBUSCONFIG_CONST_REGISTER_LEN, 0);

    if (mb_config_const_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    modbusConfigConst_setValues();

    return 0;
}

/**
 * @brief DeInit modbus config. Free register storage
 */
void modbusConfigConst_deInit(void)
{
    modbus_mapping_free(mb_config_const_mapping);
}

/**
 * @brief Command parser for modbus config communication.
 * It will substract the given START-ADDRESS from the requested
 * modbus addres to match the storage mapping.
 * It will reply the modbus request no need for upper layer.
 * @param[in] *ctx Modbus Contex
 * @param[in] *command Modbus datagram
 * @param[in] command_len Modbus datagram len
 */
void modbusConfigConst_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len)
{
    int offset = modbus_get_header_length(ctx);
    int function = command[offset];

    switch(function)
    {
     case _FC_READ_INPUT_REGISTERS:
     case _FC_READ_HOLDING_REGISTERS:
      modbus_reply_offset(ctx, command, command_len, mb_config_const_mapping, MODBUSCONFIG_CONST_REGISTER_START_ADDRESS);
      break;
     default:
      modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_FUNCTION );
    }
}

