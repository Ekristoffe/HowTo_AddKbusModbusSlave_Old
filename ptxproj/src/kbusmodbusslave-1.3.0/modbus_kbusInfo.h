#ifndef __MODBUS_KBUSINFO_H__
#define __MODBUS_KBUSINFO_H__

#include <modbus/modbus.h>

int modbusKBUSInfo_init(void);
void modbusKBUSInfo_deInit(void);
void modbusKBUSInfo_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len);


#endif /* __MODBUS_KBUSINFO_H__ */
