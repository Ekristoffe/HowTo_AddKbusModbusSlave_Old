#ifndef __MODBUS_CONFIG_H__
#define __MODBUS_CONFIG_H__

#include <modbus/modbus.h>

int modbusConfig_init(void);
void modbusConfig_deInit(void);
void modbusConfig_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len);

#endif /* __MODBUS_CONFIG_H__ */
