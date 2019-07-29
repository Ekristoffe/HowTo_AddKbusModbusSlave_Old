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
///  \file     modbus_mac.c
///
///  \brief    Reads the device MAC-Address and sets up the modbus register.
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <errno.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include "modbus.h"
#include "modbus_mac.h"
#include "modbus_reply.h"
#include "utils.h"

#define MODBUSCONFIG_MAC_START_ADDRESS 0x1031 /**< @brief Start address of modbus configuration register for MAC-Address*/

static modbus_mapping_t *mb_config_mac_mapping; /**< @brief Modbus register storage for configuration */

static int modbusConfigMac_setMACAddress(void)
{
    struct ifreq ifr;
    struct ifconf ifc;
    char buf[1024];
    int success = 0;
    int i = 0;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock == -1)
    {
        return -1;
    };

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) { /* handle error */ }

    struct ifreq* it = ifc.ifc_req;
    const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

    for (; it != end; ++it)
    {
        strcpy(ifr.ifr_name, it->ifr_name);
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0)
        {
            if (! (ifr.ifr_flags & IFF_LOOPBACK))
            { // don't count loopback
                if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0)
                {
                    success = 1;
                    break;
                }
            }
        }
        else
        {
            return -2;
        }
    }

    if (success == 1)
    {
        //copy mac to the 3 register
        for (i=0; i<3; i++)
        {
            if (mb_config_mac_mapping != NULL)
            {
                mb_config_mac_mapping->tab_registers[i] =
                    (uint16_t) ((ifr.ifr_hwaddr.sa_data[i*2] << 8) |
                                (ifr.ifr_hwaddr.sa_data[(i*2)+1]));
            }

        }
        return 0;
    }

    return -3;
}

/**
 * @brief Initialize modbus mac config. Allocate memory for register
 * storage
 *
 * @retval 0 on success
 * @retval <0 on failure
 */
int modbusConfigMac_init(void)
{
    dprintf(VERBOSE_STD, "Modbus Config MAC Init\n");
    mb_config_mac_mapping = modbus_mapping_new(0, 0, 3, 0);
    if (mb_config_mac_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return -1;
    }

    if (modbusConfigMac_setMACAddress() < 0)
    {
        return -2;
    }
    return 0;
}

/**
 * @brief DeInit modbus mac config. Free register storage
 */
void modbusConfigMac_deInit(void)
{
    modbus_mapping_free(mb_config_mac_mapping);
}

/**
 * @brief Command parser for modbus config mac communication.
 * It will substract the given START-ADDRESS from the requested
 * modbus addres to match the storage mapping.
 * It will reply the modbus request no need for upper layer.
 * @param[in] *ctx Modbus Contex
 * @param[in] *command Modbus datagram
 * @param[in] command_len Modbus datagram len
 */
void modbusConfigMac_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len)
{
    int offset = modbus_get_header_length(ctx);
    int function = command[offset];

    switch(function)
    {
     case _FC_READ_INPUT_REGISTERS:
     case _FC_READ_HOLDING_REGISTERS:
      modbus_reply_offset(ctx, command, command_len, mb_config_mac_mapping, MODBUSCONFIG_MAC_START_ADDRESS);
      break;
     default:
      modbus_reply_exception(ctx, command, MODBUS_EXCEPTION_ILLEGAL_FUNCTION );
    }
}

