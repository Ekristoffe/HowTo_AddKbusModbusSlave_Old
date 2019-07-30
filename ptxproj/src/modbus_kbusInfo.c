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
///  \file     modbus_kbusInfo.c
///
///  \brief    Sets up the KBUS information register for modbus
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <errno.h>
#include <stdio.h>
#include "modbus_kbusInfo.h"
#include "modbus.h"
#include "modbus_reply.h"
#include "utils.h"
#include "kbus.h"

#define MODBUS_KBUSINFO_START_ADDRESS   0x1022
static modbus_mapping_t *mb_mapping_kbusInfo; /**< @brief Modbus register storage for configuration */

static int modbusKBUSInfo_getValues(void)
{
    uint16_t table[4];
    int i = 0;
    if (kbus_getBitCounts(table, 4) < 0)
        return -1;
    if (mb_mapping_kbusInfo == NULL)
        return -2;

    for (i=0; i<4; i++)
    {
        mb_mapping_kbusInfo->tab_registers[i] = table[i];
    }
    return 0;
}

int modbusKBUSInfo_init(void)
{
    dprintf(VERBOSE_STD, "Modbus KBUS Info Init\n");
    mb_mapping_kbusInfo = modbus_mapping_new(0, 0, 4, 0);
    if (mb_mapping_kbusInfo == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    if (modbusKBUSInfo_getValues() <0)
    {
        return -2;
    }

    return 0;
}

void modbusKBUSInfo_deInit(void)
{
    modbus_mapping_free(mb_mapping_kbusInfo);
}

void modbusKBUSInfo_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len)
{
    int offset = modbus_get_header_length(ctx);
    int function = command[offset];

    switch(function)
    {
     case _FC_READ_INPUT_REGISTERS:
     case _FC_READ_HOLDING_REGISTERS:
      modbus_reply_offset(ctx, command, command_len, mb_mapping_kbusInfo, MODBUS_KBUSINFO_START_ADDRESS);
      break;
     default:
      modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_FUNCTION );
    }
}

