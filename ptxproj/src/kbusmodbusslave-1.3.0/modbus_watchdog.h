#ifndef __MODBUS_WATCHDOG_H__
#define __MODBUS_WATCHDOG_H__

int modbusWatchdog_init(void (*watchdogExpiredFkt)());
void modbusWatchdog_deInit(void);
void modbusWatchdog_trigger(void);
void modbusWatchdog_start(void);
void modbusWatchdog_stop(void);
int modbusWatchdog_parseModbusCommand(modbus_t *ctx, uint8_t *command, int command_len);


#endif /* __MODBUS_WATCHDOG_H__ */
