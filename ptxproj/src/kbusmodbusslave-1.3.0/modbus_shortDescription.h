#ifndef __MODBUS_SHORTDESCRIPTION_H__
#define __MODBUS_SHORTDESCRIPTION_H__

#include <modbus/modbus.h>

int modbusShortDescription_init(void);
void modbusShortDescription_deInit(void);
void modbusShortDescription_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len);

#endif /* __MODBUS_SHORTDESCRIPTION_H__ */
