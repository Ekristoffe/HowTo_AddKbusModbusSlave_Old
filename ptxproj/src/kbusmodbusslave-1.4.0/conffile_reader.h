#ifndef __CONFFILE_READER_H__
#define __CONFFILE_READER_H__


/**
 * @brief Default configuration set
 *
 */

#define DEFAULT_CONFIG_ITEM                 352
#define DEFAULT_CONFIG_PORT                 502
#define DEFAULT_CONFIG_MAX_TCP_CONNECTIONS  5
#define DEFAULT_CONFIG_COUPLER_MODE         0
#define DEFAULT_CONFIG_MODBUS_DELAY_MS      0
#define DEFAULT_CONFIG_KBUS_PRIORITY        60
#define DEFAULT_CONFIG_KBUS_CYCLE_MS        50

int conf_init(void);
int conf_getConfig(void);
void conf_deInit(void);

extern int conf_order_number;
extern int conf_modbus_port;
extern int conf_max_tcp_connections;
extern int conf_operation_mode;
extern int conf_modbus_delay_ms;
extern int conf_kbus_priority;
extern int conf_kbus_cycle_ms;

#endif /* __CONFFILE_READER_H__ */
