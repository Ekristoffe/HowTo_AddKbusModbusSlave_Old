#ifndef __MODBUS_MAC_H__
#define __MODBUS_MAC_H__

#include <modbus/modbus.h>

int modbusConfigMac_init(void);
void modbusConfigMac_deInit(void);
void modbusConfigMac_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len);

#endif /* __MODBUS_MAC_H__ */
