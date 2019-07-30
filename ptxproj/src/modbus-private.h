/*
 * Copyright © 2010-2012 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef _MODBUS_PRIVATE_H_
#define _MODBUS_PRIVATE_H_

#ifndef _MSC_VER
# include <stdint.h>
# include <sys/time.h>
#else
# include "stdint.h"
# include <time.h>
typedef int ssize_t;
#endif
#include <sys/types.h>
//#include <config.h>

#include <modbus/modbus.h>

MODBUS_BEGIN_DECLS

/* It's not really the minimal length (the real one is report slave ID
 * in RTU (4 bytes)) but it's a convenient size to use in RTU or TCP
 * communications to read many values or write a single one.
 * Maximum between :
 * - HEADER_LENGTH_TCP (7) + function (1) + address (2) + number (2)
 * - HEADER_LENGTH_RTU (1) + function (1) + address (2) + number (2) + CRC (2)
 */
#define _MIN_REQ_LENGTH 12

#define _REPORT_SLAVE_ID 180

#define _MODBUS_EXCEPTION_RSP_LENGTH 5

/* Timeouts in microsecond (0.5 s) */
#define _RESPONSE_TIMEOUT    500000
#define _BYTE_TIMEOUT        500000

/* Function codes */
#define _FC_READ_COILS                0x01
#define _FC_READ_DISCRETE_INPUTS      0x02
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_READ_INPUT_REGISTERS      0x04
#define _FC_WRITE_SINGLE_COIL         0x05
#define _FC_WRITE_SINGLE_REGISTER     0x06
#define _FC_READ_EXCEPTION_STATUS     0x07
#define _FC_DIAGNOSTICS               0x08
#define _FC_GET_COMM_EVENT_COUNT      0x0B
#define _FC_GET_COMM_EVENT_LOG        0x0C
#define _FC_WRITE_MULTIPLE_COILS      0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10
#define _FC_REPORT_SLAVE_ID           0x11
#define _FC_MASK_WRITE_REGISTER       0x16
#define _FC_WRITE_AND_READ_REGISTERS  0x17
#define _FC_READ_INPUT_REGISTERS_XL   0x42

/* Subfunction codes of _FC_DIAGNOSTICS */
#define _SFC_RETURN_QUERY_DATA                       0x00
#define _SFC_RESTART_COMMUNICATIONS_OPTION           0x01
#define _SFC_RETURN_DIAGNOSTIC_REGISTER              0x02
#define _SFC_CHANGE_ASCII_INPUT_DELIMITER            0x03
#define _SFC_FORCE_LISTEN_ONLY_MODE                  0x04
#define _SFC_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER  0x0A
#define _SFC_RETURN_BUS_MESSAGE_COUNT                0x0B
#define _SFC_RETURN_BUS_COMMUNICATION_ERROR_COUNT    0x0C
#define _SFC_RETURN_BUS_EXCEPTION_ERROR_COUNT        0x0D
#define _SFC_RETURN_SERVER_MESSAGE_COUNT             0x0E
#define _SFC_RETURN_SERVER_NO_RESPONSE_COUNT         0x0F
#define _SFC_RETURN_NAK_COUNT                        0x10
#define _SFC_RETURN_SERVER_BUSY_COUNT                0x11
#define _SFC_RETURN_BUS_CHARACTER_OVERRUN_COUNT      0x12
#define _SFC_CLEAR_OVERRUN_COUNTER_AND_FLAG          0x14

typedef enum {
    _MODBUS_BACKEND_TYPE_RTU=0,
    _MODBUS_BACKEND_TYPE_TCP,
    _MODBUS_BACKEND_TYPE_UDP,
    _MODBUS_BACKEND_TYPE_TCP_PI,
    _MODBUS_BACKEND_TYPE_UDP_PI,
} modbus_backend_type_t;

typedef enum {
    /* Request message on the server side */
    MSG_INDICATION,
    /* Request message on the client side */
    MSG_CONFIRMATION
} msg_type_t;

/* This structure reduces the number of params in functions and so
 * optimizes the speed of execution (~ 37%). */
typedef struct _sft {
    int slave;
    int function;
    int t_id;
} sft_t;

typedef struct _modbus_backend {
    unsigned int backend_type;
    unsigned int header_length;
    unsigned int checksum_length;
    unsigned int max_adu_length;
    int (*set_slave) (modbus_t *ctx, int slave);
    int (*build_request_basis) (modbus_t *ctx, int function, int addr,
                                int nb, uint8_t *req);
    int (*build_response_basis) (sft_t *sft, uint8_t *rsp);
    int (*prepare_response_tid) (const uint8_t *req, int *req_length);
    int (*send_msg_pre) (uint8_t *req, int req_length);
    ssize_t (*send) (modbus_t *ctx, const uint8_t *req, int req_length);
    ssize_t (*recv) (modbus_t *ctx, uint8_t *rsp, int rsp_length);
    int (*check_integrity) (modbus_t *ctx, uint8_t *msg,
                            const int msg_length);
    int (*pre_check_indication) (modbus_t *ctx, uint8_t const *req);
    int (*pre_check_confirmation) (modbus_t *ctx, const uint8_t *req,
                                   const uint8_t *rsp, int rsp_length);
    int (*connect) (modbus_t *ctx);
    void (*close) (modbus_t *ctx);
    int (*flush) (modbus_t *ctx);
    int (*select) (modbus_t *ctx, fd_set *rfds, struct timeval *tv, int msg_length);
    int (*filter_request) (modbus_t *ctx, int slave);
    int (*is_multicast) (modbus_t *ctx);
} modbus_backend_t;

struct _modbus {
    /* Slave address */
    int slave;
    /* Socket or file descriptor */
    int s;
    /* Transaction ID for TCP and UDP */
    uint16_t t_id;
    int debug;
    int error_recovery;
    struct timeval response_timeout;
    struct timeval byte_timeout;
    modbus_backend_t *backend;
    void *backend_data;
    int alternate_data_enabled;
    uint8_t alternate_data_value;
};

void _modbus_init_common(modbus_t *ctx);
void _error_print(modbus_t *ctx, const char *context);

#ifndef HAVE_STRLCPY
size_t strlcpy(char *dest, const char *src, size_t dest_size);
#endif

MODBUS_END_DECLS

#endif  /* _MODBUS_PRIVATE_H_ */
