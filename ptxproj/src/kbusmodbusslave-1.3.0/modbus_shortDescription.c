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
///  \file     modbus_shortDescription.c
///
///  \brief    Sets up the modbus short description register, which holds the ASCII values of the description.
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "modbus_shortDescription.h"
#include "modbus.h"
#include "modbus_reply.h"
#include "utils.h"

#define MODBUS_SHORT_DESCRIPTION_START_ADDRESS 0x2020 /**< @brief Start address of short description register*/
static modbus_mapping_t *mb_shortDescription_mapping; /**< @brief Modbus register storage*/

#define MAX_DESCRIPTION_REGISTER_COUNT  16 /** @brief Maximum register for desctiption (word) */
#define MAX_DESCRIPTION_LEN (MAX_DESCRIPTION_REGISTER_COUNT * 2) /**< @brief byte count */

static const char description_string[]="MODBUSPFCSLAVE-"; /**< @brief start of description string */

/**
 * @brief Initialize modbus mac config. Allocate memory for register
 * storage
 *
 * @retval 0 on success
 * @retval <0 on failure
 */
int modbusShortDescription_init(void)
{
    dprintf(VERBOSE_STD, "Modbus ShortDesctiption Init\n");
    mb_shortDescription_mapping = modbus_mapping_new(0, 0, MAX_DESCRIPTION_REGISTER_COUNT, 0);
    if (mb_shortDescription_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    char *dest=(char *) mb_shortDescription_mapping->tab_registers;
    size_t bytesWritten = strlen(description_string);
    if ((strncpy(dest, description_string, MAX_DESCRIPTION_LEN)) == NULL)
    {
        return -2;
    }

    dest += bytesWritten; // move pointer
    bytesWritten += strlen(VERSION);

    if (bytesWritten < MAX_DESCRIPTION_LEN)
    {
        if ((strncpy(dest, VERSION, (MAX_DESCRIPTION_LEN - bytesWritten))) == NULL)
        {
            return -3;
        }
    }

    return 0;
}

/**
 * @brief DeInit-Free register storage
 */
void modbusShortDescription_deInit(void)
{
    modbus_mapping_free(mb_shortDescription_mapping);
}

/**
 * @brief Command parser for modbus communication.
 * It will substract the given START-ADDRESS from the requested
 * modbus addres to match the storage mapping.
 * It will reply the modbus request no need for upper layer.
 * @param[in] *ctx Modbus Contex
 * @param[in] *command Modbus datagram
 * @param[in] command_len Modbus datagram len
 */
void modbusShortDescription_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len)
{
    int offset = modbus_get_header_length(ctx);
    int function = command[offset];

    switch(function)
    {
     case _FC_READ_HOLDING_REGISTERS:
      modbus_reply_offset(ctx, command, command_len, mb_shortDescription_mapping, MODBUS_SHORT_DESCRIPTION_START_ADDRESS);
      break;
     default:
      modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_FUNCTION );
    }

}

