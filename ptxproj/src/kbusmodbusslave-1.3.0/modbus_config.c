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
///  \file     modbus_config.c
///
///  \brief    modbus register handling for configuration
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include "kbus.h"
#include "modbus.h"
#include "modbus_config.h"
#include "modbus_reply.h"
#include "utils.h"

#define MODBUSCONFIG_REGISTER_START_ADDRESS_1 0x2030 /**< @brief Start address of modbus configuration register */
#define MODBUSCONFIG_REGISTER_START_ADDRESS_2 0x2031 /**< @brief Start address of modbus configuration register */
#define MODBUSCONFIG_REGISTER_START_ADDRESS_3 0x2032 /**< @brief Start address of modbus configuration register */
#define MODBUSCONFIG_REGISTER_START_ADDRESS_4 0x2033 /**< @brief Start address of modbus configuration register */
#define MODBUSCONFIG_MAX_TERMINALS_1 65 /**< @brief Maximal register space for I/O-Modules */
#define MODBUSCONFIG_MAX_TERMINALS_2 64 /**< @brief Maximal register space for I/O-Modules */
#define MODBUSCONFIG_MAX_TERMINALS_3 64 /**< @brief Maximal register space for I/O-Modules */
#define MODBUSCONFIG_MAX_TERMINALS_4 63 /**< @brief Maximal register space for I/O-Modules */

/** @brief Modbus register storage for configuration */
static modbus_mapping_t *mb_knot_asam_1;
static modbus_mapping_t *mb_knot_asam_2;
static modbus_mapping_t *mb_knot_asam_3;
static modbus_mapping_t *mb_knot_asam_4;

/**
 * @brief Get KBUS-Terminal information and write them to the
 * modbus configuration storage
 * @retval 0 on success
 * @retval <0 on failure
 */
static int modbusConfig_writeTerminalInfo(void)
{
   size_t termCnt;
   uint16_t termInfo[MODBUSCONFIG_MAX_TERMINALS_1 +
                     MODBUSCONFIG_MAX_TERMINALS_2 +
                     MODBUSCONFIG_MAX_TERMINALS_3 +
                     MODBUSCONFIG_MAX_TERMINALS_4];

   if (kbus_getTerminals(&termCnt, termInfo, sizeof(termInfo)/sizeof(uint16_t)))
       return -1;

   mb_knot_asam_1->tab_registers[0] = 352; //Fake 750-352
   size_t i=0;
   for (i = 1; i <= termCnt; i++)
   {
       if ((i >= 1) && (i <=64))
       {
           mb_knot_asam_1->tab_registers[i] = termInfo[i-1];
       }
       else if ((i >= 65) && (i <=128))
       {
           mb_knot_asam_2->tab_registers[i-65] = termInfo[i-1];
       }
       else if ((i >= 129) && (i <=192))
       {
           mb_knot_asam_3->tab_registers[i-129] = termInfo[i-1];
       }
       else if ((i >= 193) && (i <=255))
       {
           mb_knot_asam_3->tab_registers[i-193] = termInfo[i-1];
       }
   }
   return 0;
}

/**
 * @brief Initialize modbus config. Allocate memory for register
 * storage and write the registers by
 * modbusConfig_writeTerminalInfo.
 */
int modbusConfig_init(void)
{
    dprintf(VERBOSE_STD, "Modbus config Init\n");
    //modbus_mapping_t* modbus_mapping_new(int 'nb_bits', int 'nb_input_bits', int 'nb_registers', int 'nb_input_registers');
    mb_knot_asam_1 = modbus_mapping_new(0, 0, MODBUSCONFIG_MAX_TERMINALS_1, 0);
    if (mb_knot_asam_1 == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    mb_knot_asam_2 = modbus_mapping_new(0, 0, MODBUSCONFIG_MAX_TERMINALS_2, 0);
    if (mb_knot_asam_2 == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    mb_knot_asam_3 = modbus_mapping_new(0, 0, MODBUSCONFIG_MAX_TERMINALS_3, 0);
    if (mb_knot_asam_3 == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    mb_knot_asam_4 = modbus_mapping_new(0, 0, MODBUSCONFIG_MAX_TERMINALS_4, 0);
    if (mb_knot_asam_4 == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    if (modbusConfig_writeTerminalInfo() < 0)
    {
        dprintf(VERBOSE_STD, "ModbusConfig: Writing terminals failed\n");
        return -2;
    }

    return 0;
}

/**
 * @brief DeInit modbus config. Free register storage
 */
void modbusConfig_deInit(void)
{
    modbus_mapping_free(mb_knot_asam_1);
    modbus_mapping_free(mb_knot_asam_2);
    modbus_mapping_free(mb_knot_asam_3);
    modbus_mapping_free(mb_knot_asam_4);
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
void modbusConfig_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len)
{
    int offset = modbus_get_header_length(ctx);
    int function = command[offset];
    uint16_t address = (command[offset + 1] << 8) + command[offset + 2];

    switch(function)
    {
     case _FC_READ_INPUT_REGISTERS:
     case _FC_READ_HOLDING_REGISTERS:
       if (address == MODBUSCONFIG_REGISTER_START_ADDRESS_1)
       {
           modbus_reply_offset(ctx, command, command_len, mb_knot_asam_1, MODBUSCONFIG_REGISTER_START_ADDRESS_1);
       }
       else if (address == MODBUSCONFIG_REGISTER_START_ADDRESS_2)
       {
           modbus_reply_offset(ctx, command, command_len, mb_knot_asam_2, MODBUSCONFIG_REGISTER_START_ADDRESS_2);
       }
       else if (address == MODBUSCONFIG_REGISTER_START_ADDRESS_3)
       {
           modbus_reply_offset(ctx, command, command_len, mb_knot_asam_3, MODBUSCONFIG_REGISTER_START_ADDRESS_3);
       }
       else if (address == MODBUSCONFIG_REGISTER_START_ADDRESS_4)
       {
           modbus_reply_offset(ctx, command, command_len, mb_knot_asam_4, MODBUSCONFIG_REGISTER_START_ADDRESS_4);
       }
      break;
     default:
      modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_FUNCTION );
    }
}
