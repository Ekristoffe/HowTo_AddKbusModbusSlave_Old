#ifndef __MODBUS_CONST_H__
#define __MODBUS_CONST_H__

#include <modbus/modbus.h>

int modbusConfigConst_init(void);
void modbusConfigConst_deInit(void);
void modbusConfigConst_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len);


#endif /* __MODBUS_CONST_H__ */
