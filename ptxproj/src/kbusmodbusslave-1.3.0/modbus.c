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
///  \file     modbus.c
///
///  \brief    Modbus Handling. Establish Modbus communication. Main modbus receiver thread
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <errno.h>
#include <modbus/modbus.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include "modbus.h"
#include "modbus_config.h"
#include "modbus_watchdog.h"
#include "modbus_mac.h"
#include "modbus_kbusInfo.h"
#include "modbus_const.h"
#include "modbus_reply.h"
#include "modbus_shortDescription.h"
#include "kbus.h"
#include "utils.h"
#include "conffile_reader.h"

static pthread_t modbus_thread;
static pthread_t modbus_udp_thread;         /**< @brief Modbus UDP Thread */
static char modbus_running = 1;
static modbus_mapping_t *mb_mapping_in;     /**< @brief Modbus register storage - Input*/
static modbus_mapping_t *mb_mapping_write;  /**< @brief Modbus register storage - Output*/
static modbus_mapping_t *mb_mapping_2_in;   /**< @brief Modbus register storage - Input2*/
static modbus_mapping_t *mb_mapping_2_write;/**< @brief Modbus register storage - Output2*/

static modbus_mapping_t *mb_digital_1_in;    /**< @brief Modbus coil storage - Area 1*/
static modbus_mapping_t *mb_digital_1_write; /**< @brief Modbus coil storage - Area 1*/
static modbus_mapping_t *mb_digital_2_in;    /**< @brief Modbus coil storage - Area 2*/
static modbus_mapping_t *mb_digital_2_write; /**< @brief Modbus coil storage - Area 2*/

static pthread_mutex_t write_mapping_mutex=PTHREAD_MUTEX_INITIALIZER; /**< @brief Mutex for write mapping*/

static unsigned char modbus_initialized = FALSE; /**< @brief Flag for modbus initialized ready*/
static void (*modbus_receivedCallback)() = NULL; /**< @brief Callback after message is received*/

static uint8_t modbus_ApplicationState;
#define APPLICATION_STOP 0
#define APPLICATION_RUNNING 1

#define MODBUS_INREGISTER_COUNT  256 /**< @brief Maximum modbus register for input */
#define MODBUS_OUTREGISTER_COUNT 256 /**< @brief Maximum modbus register for output */

#define MODBUS_INREGISTER_2_COUNT  764 /**< @brief Maximum modbus register for input 2 */
#define MODBUS_OUTREGISTER_2_COUNT 764 /**< @brief Maximum modbus register for output 2 */

#define MODBUS_BIT_1_COUNT 512  /**< @brief Maximum modbus bits for single coil input*/
#define MODBUS_BIT_2_COUNT 1528 /**< @brief Maximum modbus bits for single coil input 2*/

/**
 * @brief Map write coils to register/process data.
 * It will directy modify the mb_mapping_write.
 */
static void modbus_mapWriteCoilsToRegister(void)
{
    unsigned int offset = kbus_getDigitalByteOffsetOutput();
    unsigned int max_bytes = kbus_getBytesToWrite();
    unsigned int i;

    uint8_t *reg = (uint8_t *)mb_mapping_write->tab_registers;
    pthread_mutex_lock(&write_mapping_mutex);
    for (i=0; i < (max_bytes - offset); i++)
    {
        reg[offset+i] = mb_digital_1_write->tab_bits[i];
    }
    pthread_mutex_unlock(&write_mapping_mutex);
}

/**
 * @brief Map read coils to register/process data.
 * It will directy modify the mb_mapping_in.
 */
static void modbus_mapReadCoils(void)
{
    unsigned int offset = kbus_getDigitalByteOffsetInput();
    unsigned int max_bytes = kbus_getBytesToRead();
    unsigned int i;

    uint8_t *reg = (uint8_t *) mb_mapping_in->tab_registers;

    for (i=0; i < (max_bytes - offset); i++)
    {
        mb_digital_1_in->tab_bits[i] = reg[offset+i];
    }
}

/**
 * @brief Clear modbus register and coil storage.
 * Set all register values and tables to '0'
 */
static void modbus_clearMapping(modbus_mapping_t *mapping)
{
    int size = 0; /**< @brief Need a different size calculation due to changes on libmodbus on WAGO side. */

    if (mapping != NULL)
    {
        if (mapping->tab_bits != NULL)
        {
            size = ((mapping->nb_bits / 16u) + ((mapping->nb_bits % 16) ? 1 : 0)) * sizeof(uint16_t);
            memset(mapping->tab_bits, 0, size);
        }

        if (mapping->tab_input_bits != NULL)
        {
            size = ((mapping->nb_input_bits / 16u) + ((mapping->nb_input_bits % 16) ? 1 : 0)) * sizeof(uint16_t);
            memset(mapping->tab_input_bits, 0, size);
        }

        if (mapping->tab_registers != NULL)
        {
            memset(mapping->tab_registers, 0, mapping->nb_registers * sizeof(uint16_t));
        }
        if (mapping->tab_input_registers != NULL)
        {
            memset(mapping->tab_input_registers, 0, mapping->nb_input_registers * sizeof(uint16_t));
        }
    }
}

/**
 * @brief Clear all modbus register.
 */
void modbus_clearAllMappings(void)
{
    /*Clear all writing mappings */
    modbus_clearMapping(mb_mapping_write);
    modbus_clearMapping(mb_mapping_2_write);
    modbus_clearMapping(mb_digital_1_write);
    modbus_clearMapping(mb_digital_2_write);

    /*Clear all reading mappings*/
    modbus_clearMapping(mb_mapping_in);
    modbus_clearMapping(mb_mapping_2_in);
    modbus_clearMapping(mb_digital_1_in);
    modbus_clearMapping(mb_digital_2_in);
}

/**
 * @brief Callback function for modbus watchdog. Will be executed if watchdog
 * expired.
 */
static void modbusWatchdog_expiredTask(void)
{
    dprintf(VERBOSE_DEBUG, "ModbusWatchdog Expired Task\n");
    if (mb_mapping_write != NULL)
    {
        //set all outputs to 0
        pthread_mutex_lock( &write_mapping_mutex );
          modbus_clearAllMappings();
        pthread_mutex_unlock( &write_mapping_mutex );
    }
}

static void modbus_worker_read(modbus_t *ctx, uint8_t *query, int rc)
{
    //calculate the address to switch between mappings
    int offset = modbus_get_header_length(ctx);
    int function = query[offset];
    dprintf(VERBOSE_INFO, "Function :%d\n", function);
    uint16_t address = (query[offset + 1] << 8) + query[offset + 2];

    switch (function)
    {
     case _FC_READ_COILS:
      modbus_mapReadCoils();
      //Read digital inputs
      if (address <=511)
      {
          modbus_reply(ctx, query, rc, mb_digital_1_in);
      }
      //Read digital outputs
      else if ((address >= 512) && (address <=1023))
      {
          modbus_reply_offset(ctx, query, rc, mb_digital_1_write, 512);
      }
      else if ((address >=0x8000) && (address <= 0x85F7)) //Digital Input-Area 2 (Bit 513 - 2039)
      {
          modbus_reply_offset(ctx, query, rc, mb_digital_2_in, 0x8000);
      }
      else if ((address >=0x9000) && (address <= 0x95F7)) //Digital Output-Area 2 (Bit 513 - 2093)
      {
          modbus_reply_offset(ctx, query, rc, mb_digital_2_write, 0x9000);
      }
      else
      {
          modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS );
      }
      break;

     case _FC_READ_DISCRETE_INPUTS:
     case _FC_READ_HOLDING_REGISTERS:
     case _FC_READ_INPUT_REGISTERS:
     case _FC_WRITE_AND_READ_REGISTERS:
      //Address check
      if (address <= 255)
      {
          //rc is the query size
          modbus_reply(ctx, query, rc, mb_mapping_in);
      }
      else if ((address >= 512) && (address <= 767))
      {
          modbus_reply_offset(ctx, query, rc, mb_mapping_write, 512);
      }
      // Handle configuration registers
      else if ((address >=0x1000) && (address <=0x2043))
      {
          dprintf(VERBOSE_DEBUG, "Config Dataset\n");
          //Watchdog
          if ((address >= 0x1000) && (address <=0x100B))
          {
              modbusWatchdog_parseModbusCommand(ctx, query, rc);
          }
          //Processdata-Information
          else if ((address >= 0x1022) && (address <=0x1025))
          {
              modbusKBUSInfo_parseModbusCommand(ctx, query, rc);
          }
          //MAC-Address
          else if ((address >= 0x1031) && (address <= 0x1033))
          {
              modbusConfigMac_parseModbusCommand(ctx, query, rc);
          }
          //Const
          else if ((address >= 0x2000) && (address <= 0x2008))
          {
              modbusConfigConst_parseModbusCommand(ctx, query, rc);
          }
          //Short description
          else if (address == 0x2020)
          {
              modbusShortDescription_parseModbusCommand(ctx,query,rc);
          }
          //Knot-assembly 1-4
          else if ((address >= 0x2030) && (address <= 0x2033))
          {
              modbusConfig_parseModbusCommand(ctx, query, rc);
          }
          else
          {
              modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS );
          }
      }
      //In-Area 2
      else if ((address >= 0x6000) && (address <= 0x62FB))
      {
          modbus_reply_offset(ctx, query, rc, mb_mapping_2_in, 0x6000);
      }
      else if  ((address >= 0x7000) && (address <= 0x72FB))
      {
          modbus_reply_offset(ctx, query, rc, mb_mapping_2_write, 0x7000);
      }
      else
      {
          modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS );
      }

      break;
    }
}

static void modbus_worker_write(modbus_t *ctx, uint8_t *query, int rc)
{
    //calculate the address to switch between mappings
    int offset = modbus_get_header_length(ctx);
    int function = query[offset];
    dprintf(VERBOSE_INFO, "Function :%d\n", function);
    uint16_t address = (query[offset + 1] << 8) + query[offset + 2];

    switch (function)
    {
     case _FC_WRITE_SINGLE_COIL:
     case _FC_WRITE_MULTIPLE_COILS:
      modbus_mapReadCoils();
      if (address <=511)//Output Area 1
      {
          modbus_reply(ctx, query, rc, mb_digital_1_write);
      }
      else if ((address >= 512) && (address <= 1023)) //Output Area 1-Mirror
      {
          modbus_reply_offset(ctx, query, rc, mb_digital_1_write, 512);
      }
      else if ((address >=0x8000) && (address <= 0x85F7)) //Digital Output-Area 2 (Bit 513 - 2039)
      {
          modbus_reply_offset(ctx, query, rc, mb_digital_2_write, 0x8000);
      }
      else if ((address >=0x9000) && (address <= 0x95F7)) //Digital Output-Area 2 (Bit 513 - 2093) - Mirror
      {
          modbus_reply_offset(ctx, query, rc, mb_digital_2_write, 0x9000);
      }
      else
      {
          modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS );
      }
      //Map Coilbits to Register.
      modbus_mapWriteCoilsToRegister();
      break;

     case _FC_WRITE_SINGLE_REGISTER:
     case _FC_WRITE_MULTIPLE_REGISTERS:
     case _FC_WRITE_AND_READ_REGISTERS:
      if (address <=255)
      {
          modbus_reply_offset(ctx, query, rc, mb_mapping_write, 0);
      }
      else if ((address >=512) && (address <=767))
      {
          modbus_reply_offset(ctx, query, rc, mb_mapping_write, 512);
      }
      //Configuration
      else if ((address >=0x1000) && (address <=0x2043))
      {
          //Watchdog
          if ((address >= 0x1000) && (address <=0x100B))
          {
              modbusWatchdog_parseModbusCommand(ctx, query, rc);
          }
      }
      //In-Area 2
      else if ((address >= 0x6000) && (address <= 0x62FB))
      {
          modbus_reply_offset(ctx, query, rc, mb_mapping_2_write, 0x6000);
      }
      else if ((address >= 0x7000) && (address <= 0x72FB))
      {
          modbus_reply_offset(ctx, query, rc, mb_mapping_2_write, 0x7000);
      }
      else
      {
          modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS );
      }
      break;

     case _FC_REPORT_SLAVE_ID:
     case _FC_READ_EXCEPTION_STATUS:
      dprintf(VERBOSE_DEBUG, "-------------------TODO--------------------\n");

    }
}

/**
 * @brief Modbus worker for all incomming modbus messages.
 * @param[in] ctx - Modbus environment
 * @param[in] query - Modbus message
 * @param[in] rc - Modbus message length
 */
static void modbus_worker(modbus_t *ctx, uint8_t *query, int rc)
{

    int offset = modbus_get_header_length(ctx);
    int function = query[offset];
    uint8_t function_found = FALSE;

    if (modbus_ApplicationState == APPLICATION_STOP)
    {
        modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY);
        return;
    }

    modbusWatchdog_trigger();

    //modbus write
    //received Callback
    //modbus read
    switch (function)
    {
     case _FC_WRITE_SINGLE_COIL:
     case _FC_WRITE_MULTIPLE_COILS:
     case _FC_WRITE_SINGLE_REGISTER:
     case _FC_WRITE_MULTIPLE_REGISTERS:
      modbus_worker_write(ctx, query, rc);
      function_found = TRUE;
      break;
     case _FC_WRITE_AND_READ_REGISTERS:
      modbus_worker_write(ctx, query, rc);
      //All done we can go back!
      return;
      break;
    }

    if (modbus_receivedCallback != NULL)
        modbus_receivedCallback();

    switch (function)
    {
     case _FC_READ_COILS:
     case _FC_READ_DISCRETE_INPUTS:
     case _FC_READ_HOLDING_REGISTERS:
     case _FC_READ_INPUT_REGISTERS:
      modbus_worker_read(ctx, query, rc);
      function_found = TRUE;
      break;
    }

    if (function_found == FALSE)
    {
      modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
    }
}

static void *modbus_udp_task(void *none)
{
    none=none; //-Wunused-parameter
    modbus_t *ctx_udp=NULL;

    ctx_udp = modbus_new_udp("127.0.0.1", conf_modbus_port);
    int udp_socket = modbus_udp_bind(ctx_udp);
    uint8_t udp_query[MODBUS_TCP_MAX_ADU_LENGTH];

    if (ctx_udp == NULL) {
        fprintf(stderr, "Unable to allocate libmodbus UDP context\n");
        return NULL;
    }

    struct pollfd fds[] = {{udp_socket, POLLIN, 0}};

    while (modbus_running)
    {
        if (poll(fds, 1, 1000) == 0)
        {
            //dprintf(VERBOSE_STD, "TIMEOUT UDP\n");
        }
        else
        {
            if (fds[0].revents & POLLIN)
            {
                int rc = modbus_receive(ctx_udp, udp_query, sizeof(udp_query));
                if (rc >= 0)
                {
                    modbus_worker(ctx_udp, udp_query, rc);
                }
            }
        }
    }

    modbus_close(ctx_udp);
    modbus_free(ctx_udp);
    return NULL;

}

/**
 * @brief Modbus thread task
 */
static void *modbus_task(void *none)
{
    none=none; //-Wunused-parameter
    int master_socket;
    int server_socket=0;
    int rc;
    int fdmax=0; //Maximum file descriptor number
    fd_set refset;
    fd_set rdset;
    modbus_t *ctx;

    struct timeval modbus_select_timeout;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

    //----- Wait for kbus to be initialized -----
    dprintf(VERBOSE_DEBUG, "Modbus: Wait for KBUS to be initialized\n");
    while ((kbus_getIsInitialized() == FALSE) && (modbus_running))
    {
        usleep(50 * 1000); //50ms sleep
    }

    //check if application was terminated in the meantime
    if (modbus_running == 0)
    {
        return NULL;
    }
    //-------------------------------------------

    //modbus_mapping_t* modbus_mapping_new(int 'nb_bits', int 'nb_input_bits', int 'nb_registers', int 'nb_input_registers');
    mb_mapping_in = modbus_mapping_new(0, 0, MODBUS_OUTREGISTER_COUNT, MODBUS_INREGISTER_COUNT);
    if (mb_mapping_in == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }

    mb_mapping_write = modbus_mapping_new(0, 0, MODBUS_OUTREGISTER_COUNT, MODBUS_INREGISTER_COUNT);
    if (mb_mapping_write == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }

    mb_mapping_2_in = modbus_mapping_new(0, 0, MODBUS_OUTREGISTER_2_COUNT, MODBUS_INREGISTER_2_COUNT);
    if (mb_mapping_2_in == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }

    mb_mapping_2_write = modbus_mapping_new(0, 0, MODBUS_OUTREGISTER_2_COUNT, MODBUS_INREGISTER_2_COUNT);
    if (mb_mapping_2_write == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }

    //--- Storage for coil read and write ---
    mb_digital_1_in = modbus_mapping_new(MODBUS_BIT_1_COUNT, MODBUS_BIT_1_COUNT, 0, 0);
    if (mb_digital_1_in == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }

    mb_digital_1_write = modbus_mapping_new(MODBUS_BIT_1_COUNT, MODBUS_BIT_1_COUNT, 0, 0);
    if (mb_digital_1_write == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }

    mb_digital_2_in = modbus_mapping_new(MODBUS_BIT_2_COUNT, MODBUS_BIT_2_COUNT, 0, 0);
    if (mb_digital_2_in == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }

    mb_digital_2_write = modbus_mapping_new(MODBUS_BIT_2_COUNT, MODBUS_BIT_2_COUNT, 0, 0);
    if (mb_digital_2_write == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        return NULL;
    }
    //-------------------------------------------

    //Initialize TCP-Modbus-Connection:
    ctx = modbus_new_tcp("127.0.0.1", conf_modbus_port);

    server_socket = modbus_tcp_listen(ctx, conf_max_tcp_connections);
    //Clear the reference set of socket
    FD_ZERO(&refset);
    //Add the server socket
    FD_SET(server_socket, &refset);

    //Keep track of the max file descriptor
    fdmax=server_socket;

    if (ctx == NULL) {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return NULL;
    }

    //Set Modbus response delay
    modbus_set_response_delay(conf_modbus_delay_ms);

    if (modbusWatchdog_init(modbusWatchdog_expiredTask) < 0)
    {
        dprintf(VERBOSE_STD, "ModbusWatchdog: Init failed\n");
        return NULL;
    }

    if (modbusConfig_init() < 0)
    {
        dprintf(VERBOSE_STD, "ModbusConfig: ModbusConfig Init failed\n");
        return NULL;
    }

    if (modbusConfigMac_init() < 0)
    {
        dprintf(VERBOSE_STD, "ModbusConfigMac: Init failed\n");
        return NULL;
    }

    if (modbusKBUSInfo_init() < 0)
    {
        dprintf(VERBOSE_STD, "ModbusKBUSInfo: Init failed\n");
        return NULL;
    }

    if (modbusConfigConst_init() < 0)
    {
        dprintf(VERBOSE_STD, "ModbusConfigConst: Init failed\n");
        return NULL;
    }

    if (modbusShortDescription_init() < 0)
    {
        dprintf(VERBOSE_STD, "ModbusShortDescription: Init failed\n");
        return NULL;
    }

    modbus_initialized = TRUE;
    dprintf(VERBOSE_STD, "Modbus-Init complete - Ready for take off\n");
    //--- Start Modbus-UDP Thread
    if (pthread_create(&modbus_udp_thread, NULL,
                   &modbus_udp_task, NULL) != 0)
    {
        return NULL;
    }
    //--- Modbus-TCP Thread
    while (modbus_running)
    {
        rdset = refset;
        //Reset Timeout
        //select() may update the timeout argument to  indicate  how  much
        //time was left.
        modbus_select_timeout.tv_sec = 1;         /* seconds */
        modbus_select_timeout.tv_usec = 0;        /* microseconds */
        if (select(fdmax+1, &rdset, NULL, NULL, &modbus_select_timeout) == -1)
        {
            fprintf(stderr, "Server select() failure.\n");
            continue;
        }

        //Run through the existing connections looking for data to be read
        for (master_socket = 0; master_socket <= fdmax; master_socket++)
        {
            if (FD_ISSET(master_socket, &rdset))
            {
                //A client is asking a new connection
                if (master_socket == server_socket)
                {
                    socklen_t addrlen;
                    struct sockaddr_in clientaddr;
                    int newfd;

                    //Handle new connection
                    addrlen = sizeof(clientaddr);
                    memset(&clientaddr, 0, sizeof(clientaddr));
                    newfd = accept(server_socket, (struct sockaddr *) &clientaddr, &addrlen);
                    if (newfd == -1)
                    {
                        fprintf(stderr, "Server accept error\n");
                    }
                    else
                    {
                        FD_SET(newfd, &refset);
                        if (newfd > fdmax)
                        {
                            // Keep track of the maximum file descriptor
                            fdmax = newfd;
                        }
                        dprintf(VERBOSE_STD, "New Modbus connection from %s:%d on socket %d\n",
                                inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
                    }
                }
                else //An already connected master has sent a new query
                {
                    modbus_set_socket(ctx, master_socket);
                    //API-Change made by WAGO:
                    //int modbus_receive(modbus_t *ctx, uint8_t *req, size_t max_size)
                    //origin:
                    //int modbus_receive(modbus_t *ctx, uint8_t *req)
                    rc = modbus_receive(ctx, query, sizeof(query));
                    if (rc != -1)
                    {
                        modbus_worker(ctx, query, rc);
                    }
                    else
                    {
                        //Connection closed by the client, end of server
                        dprintf(VERBOSE_STD, "Connection closed on socket %d\n", master_socket);
                        close(master_socket);
                        //Remove from reference set
                        FD_CLR(master_socket, &refset);

                        if (master_socket == fdmax)
                        {
                            fdmax--;
                        }
                    }
                }
            }
        }
    }

    dprintf(VERBOSE_STD, "Modbus loop exit\n");
    modbus_mapping_free(mb_mapping_in);
    modbus_mapping_free(mb_mapping_write);
    modbus_mapping_free(mb_mapping_2_in);
    modbus_mapping_free(mb_mapping_2_write);

    modbus_mapping_free(mb_digital_1_in);
    modbus_mapping_free(mb_digital_1_write);
    modbus_mapping_free(mb_digital_2_in);
    modbus_mapping_free(mb_digital_2_write);

    modbus_close(ctx);
    modbus_free(ctx);
    return NULL;
}

/**
 * @brief Start kbus thread
 * @retval 0 on success
 * @retval <0 on failure
 */
int modbus_start(void)
{
    modbus_running = 1;
    pthread_mutex_init(&write_mapping_mutex, NULL);
    if (pthread_create(&modbus_thread, NULL,
                   &modbus_task, NULL) != 0)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief Stop kbus thread
 */
void modbus_stop(void)
{
    modbus_running = 0;
    modbus_initialized = FALSE;
    dprintf(VERBOSE_STD, "MODBUS STOP\n");
    pthread_join(modbus_thread, NULL);
    pthread_join(modbus_udp_thread, NULL);
    modbusWatchdog_deInit();
    modbusConfig_deInit();
    modbusConfigMac_deInit();
    modbusKBUSInfo_deInit();
    modbusConfigConst_deInit();
    modbusShortDescription_deInit();
    pthread_mutex_destroy(&write_mapping_mutex);
}

/**
 * @brief Copy data to modbus register
 * @param[in] *source pointer to the source
 * @param[in] n number of words to be copied from source
 * @return number of registers copied from source
 */
int modbus_copy_register_in(uint16_t *source, size_t n)
{
    int secondRegisterBytes = 0;
    if (source == NULL)
    {
        return -1;
    }

    if (n > (MODBUS_OUTREGISTER_COUNT + MODBUS_OUTREGISTER_2_COUNT)) //Check for maximum size
    {
        return -2;
    }

    if ( modbus_initialized == FALSE)
    {
        return 0;
    }

    if ( n > MODBUS_OUTREGISTER_COUNT)
    {
        secondRegisterBytes = (n - MODBUS_OUTREGISTER_COUNT);
        n = MODBUS_OUTREGISTER_COUNT;
    }
    //memset(mb_mapping_in>tab_registers, 0, sizeof(mb_mapping_in->tab_registers));
    memcpy(mb_mapping_in->tab_registers, source, sizeof(uint16_t) * n);

    if ( secondRegisterBytes > 0 )
    {
        //calculate source offset for new starting point
        source += (MODBUS_OUTREGISTER_COUNT * sizeof(uint16_t));
        memcpy(mb_mapping_2_in->tab_registers, source, secondRegisterBytes * sizeof(uint16_t));
    }

    return n;
}

/**
 * @brief Copy modbus register to datapointer
 * @param[out] *dest pointer to the destination
 * @param[in] n number of bytes to be copied from source
 * @return number of bytes copied from source
 */
int modbus_copy_register_out(uint8_t *dest, size_t n)
{
    uint8_t *dest_buffer=dest;
    size_t secondRegisterBytes = 0;
    size_t totalModbusBytes = (MODBUS_OUTREGISTER_COUNT + MODBUS_OUTREGISTER_2_COUNT)*sizeof(int16_t);

    if (dest_buffer == NULL)
    {
        return -1;
    }

    //Check if Buffer (byte) is smaller than the total amount of registers (int16_t)
    if (n < totalModbusBytes)
    {
        return -2;
    }

    if (modbus_initialized == FALSE)
    {
        return 0;
    }

    if ( n > (MODBUS_OUTREGISTER_COUNT * sizeof(uint16_t)))
    {
        secondRegisterBytes = (n - (MODBUS_OUTREGISTER_COUNT * sizeof(uint16_t)));
        n = MODBUS_OUTREGISTER_COUNT * sizeof(uint16_t);
    }
    pthread_mutex_lock( &write_mapping_mutex );
        memcpy(dest_buffer, mb_mapping_write->tab_registers, n);
        if ( secondRegisterBytes > 0 )
        {
            //calculate source offset for new starting point
            dest_buffer += (MODBUS_OUTREGISTER_COUNT * sizeof(uint16_t));
            memcpy(dest_buffer, mb_mapping_2_write->tab_registers, secondRegisterBytes);
        }

    pthread_mutex_unlock( &write_mapping_mutex );
    return n;
}

/**
 * @brief Register a callback function, that is executed on every
 * message action.
 * @param[in] funct Void pointer to callback function
 */
void modbus_registerMsgReceivedCallback(void (*funct)())
{
    if (funct != NULL)
    {
        modbus_receivedCallback = funct;
        modbus_replyRegisterCallback(funct);
    }
    else
        modbus_receivedCallback = NULL;
}

/**
 * @brief Handler function to set the correct behavior if OMS is switched to stop
 * @return Always 0
 */
void modbus_ApplicationStateStop(void)
{
    modbus_ApplicationState = APPLICATION_STOP;
}

/**
 * @brief Handler function to set the correct behavior if OMS is switched to run.
 */
void modbus_ApplicationStateRun(void)
{
    modbus_ApplicationState = APPLICATION_RUNNING;
}

/**
 * @brief Returns to correct mapping according to the given address
 * @param[in] write_address - Write address
 * @returns modbus_mapping pointer to the initialized mapping.
 * @reval NULL on error
 */
modbus_mapping_t *modbus_getWriteMapping(uint16_t *write_address)
{
    uint16_t address = *write_address;
    if (address <= 255)
    {
        return mb_mapping_write;
    }
    else if ((address >= 512) && (address <= 767))
    {
        *write_address -= 512;
        return mb_mapping_write;
    }
    else if ((address >= 0x6000) && (address <= 0x62FB))
    {
        *write_address -= 0x6000;
        return mb_mapping_2_write;
    }
    else if ((address >= 0x7000) && (address <= 0x72FB))
    {
        *write_address -= 0x7000;
        return mb_mapping_2_write;
    }
    else
        return NULL;
}

/**
 * @brief Returns to correct mapping according to the given address
 * @param[in] read_address - read address
 * @returns modbus_mapping pointer to the initialized mapping.
 * @reval NULL on error
 */
modbus_mapping_t *modbus_getReadMapping(uint16_t *read_address)
{
    uint16_t address = *read_address;

    if (address <= 255)
    {
        return mb_mapping_in;
    }
    else if ((address >= 512) && (address <= 767))
    {
        *read_address -= 512;
        return mb_mapping_write;
    }
    else if ((address >= 0x6000) && (address <= 0x62FB))
    {
        *read_address -= 0x6000;
        return mb_mapping_2_in;
    }
    else if ((address >= 0x7000) && (address <= 0x72FB))
    {
        *read_address -= 0x7000;
        return mb_mapping_2_write;
    }
    else
        return NULL;
}
