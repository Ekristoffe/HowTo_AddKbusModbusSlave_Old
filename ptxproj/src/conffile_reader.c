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
///  \file     conffile_reader.c
///
///  \brief    Reads the configuration file /etc/kbusmodbusslave.conf and parses the values
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "utils.h"
#include "conffile_reader.h"

#define CONF_FILENAME "/etc/kbusmodbusslave.conf"
#define CONF_OPTS_COUNT (sizeof(options) / sizeof(options[0]))

/**
 * Parameter variables
 */
int conf_modbus_port = 0;
int conf_max_tcp_connections = 0;
int conf_operation_mode = 0;
int conf_modbus_delay_ms = 0;
int conf_kbus_priority = 0;
int conf_kbus_cycle_ms = 0;

/**
 * @brief Config file available parameters
 */
const char *options[] = {
    "modbus_port",
    "max_tcp_connections",
    "operation_mode",
    "modbus_delay_ms",
    "kbus_priority",
    "kbus_cycle_ms"
};

/**
 * @brief Config
 */
static int conf_setParameter(const char *parameter, char *value)
{
    //PORT-PARAMTER
    if (strcmp(parameter, options[0]) == 0)
    {
        if(str2int(&conf_modbus_port, value, 10) != STR2INT_SUCCESS)
            return -1;
    }
    else if (strcmp(parameter, options[1]) == 0)
    {
        if (str2int(&conf_max_tcp_connections, value, 10) != STR2INT_SUCCESS)
            return -1;
    }
    else if (strcmp(parameter, options[2]) == 0)
    {
        if (str2int(&conf_operation_mode, value, 10 ) != STR2INT_SUCCESS)
            return -1;

        if (conf_operation_mode < 0)
            conf_operation_mode = 0;
        else if (conf_operation_mode >= 1)
            conf_operation_mode = 1;
    }
    else if (strcmp(parameter, options[3]) == 0)
    {
        if (str2int(&conf_modbus_delay_ms, value, 10) != STR2INT_SUCCESS)
            return -1;
    }
    else if (strcmp(parameter, options[4]) == 0)
    {
        if (str2int(&conf_kbus_priority, value, 10) != STR2INT_SUCCESS)
            return -1;
        //checking the min and max values for kbus_priority!
        if ((conf_kbus_priority < 1) || (conf_kbus_priority > 99))
        {
            fprintf(stderr, "INVALID PARAMETER: Priority-value must be in the range of 1-99\n");
            return -1;
        }
    }
    else if (strcmp(parameter, options[5])== 0)
    {
        if (str2int(&conf_kbus_cycle_ms, value, 10) != STR2INT_SUCCESS)
            return -1;

        //checking range
        if ((conf_kbus_cycle_ms < 5) || (conf_kbus_cycle_ms > 50))
        {
            fprintf(stderr, "INVALID PARAMETER: KBUS cycle time must be in the range of 5-50 ms\n");
            return -1;
        }
    }

    return 0;
}

static void conf_printConfiguration(void)
{
    fprintf(stdout, "\n======= CONFIGURATION =======\n");
    fprintf(stdout, "PORT: %d\n", conf_modbus_port);
    fprintf(stdout, "MAX CONNECTIONS: %u\n", conf_max_tcp_connections);
    fprintf(stdout, "OPERATION MODE: %u\n", conf_operation_mode);
    fprintf(stdout, "MODBUS DELAY MS: %u\n", conf_modbus_delay_ms);
    fprintf(stdout, "KBUS CYCLE TIME MS: %d\n", conf_kbus_cycle_ms);
    fprintf(stdout, "KBUS PRIORITY: %d\n", conf_kbus_priority);
    fprintf(stdout, "==============================\n");
}

/**
 * @brief Initialisation of Config setting defaults.
 */
int conf_init(void)
{
    //-------- Port --------
    conf_modbus_port = DEFAULT_CONFIG_PORT;
    //-------- Max Connections ------
    conf_max_tcp_connections = DEFAULT_CONFIG_MAX_TCP_CONNECTIONS;
    //-------- Coupler Mode --------
    conf_operation_mode = DEFAULT_CONFIG_COUPLER_MODE;
    //-------- Modbus Delay ms -------
    conf_modbus_delay_ms = DEFAULT_CONFIG_MODBUS_DELAY_MS;
    //-------- KBUS Priority ------
    conf_kbus_priority = DEFAULT_CONFIG_KBUS_PRIORITY;
    return 0;
}

/**
 * @brief Parse config file. Config entries are speparated by space and newline character.
 * @retval 0 on success
 * @retval <0 on failure
 */
int conf_getConfig(void)
{
    FILE * fp ;
    char *bufr;
    char *conf;
    size_t len = 0;
    char delimiter[] = " \n"; //Only space and \n as delimimiter
    size_t i=0;

    if((fp = fopen(CONF_FILENAME, "r")) != NULL)
    {
        while(getline(&bufr, &len, fp) != -1)
        {
            //skip comment line and blank line
            if ((bufr[0] == '#') || (bufr[0] == '\n'))
                continue;
            /* parsing conf file */
            conf = strtok(bufr, delimiter);
            for (i = 0; i < CONF_OPTS_COUNT; i++)
            {
                if (strcmp(conf, options[i]) == 0)
                {
                    conf = strtok(NULL, delimiter);
                    if (conf_setParameter(options[i], conf) < 0)
                    {
                        return -1;
                    }
                    continue;
                }
            }
        }
    }
    else
    {
        /* error processing, couldn't open file */
        return -1;
    }
    free(bufr);
    conf_printConfiguration();
    return 0;
}

/**
 * @brief DeInit of config free all variables
 */
void conf_deInit(void)
{
}
